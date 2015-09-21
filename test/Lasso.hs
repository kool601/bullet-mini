{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.GL
import Game.Pal
import Physics.Bullet
import Linear

import Control.Monad
import Control.Monad.State
import Control.Lens
import qualified Data.Map as Map
import Data.Map (Map)
import Data.Maybe

import Data.Time

import Types

import Halive.Utils

data World = World
  { _wldPlayer :: Pose
  , _wldCubes  :: Map ObjectID Cube
  }
makeLenses ''World

tuneSpring spring = do
  setSpringLinearLowerLimit spring (-5)
  setSpringLinearUpperLimit spring 5
  setSpringAngularLowerLimit spring (-1)
  setSpringAngularUpperLimit spring 1
  setSpringAngularStiffness spring 100
  setSpringLinearStiffness spring 100
  setSpringLinearDamping spring 0.9
  setSpringAngularDamping spring 0.9
  setSpringLinearBounce spring 10
  setSpringAngularBounce spring 10
  setSpringLinearEquilibrium spring 0
  setSpringAngularEquilibrium spring 0
  setLinearSpringEnabled spring (V3 True True True)
  setAngularSpringEnabled spring (V3 True True True)

main :: IO ()
main = do
  GamePal{..}    <- reacquire 0 $ initGamePal "Bullet" NoGCPerFrame []

  dynamicsWorld  <- createDynamicsWorld mempty

  cubeProg       <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo        <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape      <- makeShape cubeGeo cubeProg

  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)
  glEnable GL_DEPTH_TEST
  glClearColor 0 0 0.1 1

  let initialWorld = World
        { _wldPlayer = Pose (V3 0 20 40) (axisAngle (V3 0 1 0) 0)
        , _wldCubes  = mempty
        }
  void . flip runStateT initialWorld $ do

    _ <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0

    -- Add a moving cube
    movingRigidBody <- addCube dynamicsWorld (RigidBodyID 12) mempty 
        { pcPosition = V3 0 20 5
        , pcRotation = Quaternion 0 (V3 0 1 0)
        }
    wldCubes . at 12 ?= Cube
        { _cubBody = movingRigidBody
        , _cubColor = V4 0 1 1 1
        }
    setRigidBodyKinematic movingRigidBody

    let springID = 99
    springRigidBody <- addCube dynamicsWorld (RigidBodyID springID) mempty 
      { pcPosition = V3 0 20 0
      , pcRotation = Quaternion 0 (V3 0 1 0)
      }
    wldCubes . at (fromIntegral springID) ?= Cube
      { _cubBody = springRigidBody
      , _cubColor = V4 1 0.5 1 1
      }
    spring <- addWorldSpringConstraint dynamicsWorld springRigidBody 
    tuneSpring spring

    whileWindow gpWindow $ do
      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      -- applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer
      processEvents gpEvents $ \e -> 
        closeOnEscape gpWindow e

      -- Move the cube up and down
      y <- (+ 20) . (* 10) . sin . (\x->x::Double) . realToFrac . utctDayTime <$> liftIO getCurrentTime
      setRigidBodyWorldTransform movingRigidBody (V3 0 y 5) (axisAngle (V3 0 1 0) 0)

      setSpringWorldPose spring (V3 0 y 5) (axisAngle (V3 0 1 0) 0)

      
      stepSimulation dynamicsWorld

      -- Render Cubes
      projMat <- makeProjection gpWindow
      viewMat <- viewMatrixFromPose <$> use wldPlayer

      uniformV3 uCamera =<< use (wldPlayer . posPosition)

      let viewProj = projMat !*! viewMat

      cubes <- Map.toList <$> use wldCubes
      withVAO (sVAO cubeShape) $ 
        forM_ cubes $ \(cubeID, cube) -> do
          (position, orientation) <- getBodyState (cube ^. cubBody)

          let model = mkTransformation orientation position
          uniformM44 uModelViewProjection (viewProj !*! model)
          uniformM44 uInverseModel        (fromMaybe model (inv44 model))
          uniformM44 uModel               model
          uniformV4  uDiffuse             (cube ^. cubColor)
          glDrawElements GL_TRIANGLES (vertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

      swapBuffers gpWindow
