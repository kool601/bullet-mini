{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.GL
import Graphics.VR.Pal
import Physics.Bullet
import Linear.Extra

import Control.Monad
import Control.Monad.State
import Control.Lens.Extra
import qualified Data.Map as Map
import Data.Map (Map)
import Data.Maybe

import Data.Time

import Types
import CubeUniforms

import Halive.Utils

data World = World
  { _wldPlayer :: !(Pose GLfloat)
  , _wldCubes  :: !(Map ObjectID Cube)
  , _wldFrames :: !Int
  }
makeLenses ''World

main :: IO ()
main = do
  VRPal{..}    <- reacquire 0 $ initVRPal "Bullet" NoGCPerFrame []

  dynamicsWorld  <- createDynamicsWorld mempty
  

  cubeProg       <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo        <- cubeGeometry (2 :: V3 GLfloat) (V3 1 1 1)
  cubeShape      <- makeShape cubeGeo cubeProg

  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)
  glEnable GL_DEPTH_TEST
  glClearColor 0 0 0.1 1

  let initialWorld = World
        { _wldPlayer = Pose (V3 0 20 40) (axisAngle (V3 0 1 0) 0)
        , _wldCubes  = mempty
        , _wldFrames = 0
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

    -- Add a falling cube
    fallingRigidBody <- addCube dynamicsWorld (RigidBodyID 11) mempty 
        { pcPosition = V3 0 50 0
        , pcRotation = Quaternion 0 (V3 0 1 0)
        }
    wldCubes . at 11 ?= Cube
        { _cubBody = fallingRigidBody
        , _cubColor = V4 1 1 1 1
        }

    -- Add a row of cubes attached to worldspace spring constraints
    forM_ [1..10] $ \i -> do
      springRigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 (-fromIntegral i * 2.1 + 11) 20 0
        , pcRotation = Quaternion 0 (V3 0 1 0)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = springRigidBody
        , _cubColor = V4 1 1 1 1
        }
      spring <- addWorldSpringConstraint dynamicsWorld springRigidBody
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
      -- setSpringLinearEquilibrium spring 0
      -- setSpringAngularEquilibrium spring 0
      setLinearSpringEnabled spring (V3 True True True)
      setAngularSpringEnabled spring (V3 True True True)


    whileWindow gpWindow $ do
      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      -- applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer
      processEvents gpEvents $ \e -> 
        closeOnEscape gpWindow e

      -- Move the cube up and down
      y <- (+ 20) . (* 5) . sin . (\x->x::Double) . realToFrac . utctDayTime <$> liftIO getCurrentTime
      setRigidBodyWorldTransform movingRigidBody (V3 0 y 5) (axisAngle (V3 0 1 0) 0)

      stepSimulation dynamicsWorld 90

      -- Set all cubes to white
      wldCubes . traverse . cubColor .= V4 1 1 1 1
      -- Set all colliding cubes to green
      collisions <- getCollisions dynamicsWorld
      forM_ collisions $ \collision -> do
        let bodyAID = (fromIntegral . unRigidBodyID . cbBodyAID) collision
        let bodyBID = (fromIntegral . unRigidBodyID . cbBodyBID) collision
        when (bodyAID /= 0 && bodyBID /= 0) $ 
          liftIO . putStrLn $ "Cube " ++ show bodyAID ++ " hit Cube " ++ show bodyBID
        wldCubes . at bodyAID . traverse . cubColor .= V4 0 1 0 1
        wldCubes . at bodyBID . traverse . cubColor .= V4 0 1 0 1

      -- Render Cubes

      projMat <- getWindowProjection gpWindow 45 0.1 1000
      viewMat <- viewMatrixFromPose <$> use wldPlayer
      (x,y,w,h) <- getWindowViewport gpWindow
      glViewport x y w h

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
          glDrawElements GL_TRIANGLES (geoVertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

      swapBuffers gpWindow
