{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}

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
import Data.Maybe

import Types

import Halive.Utils

main :: IO ()
main = do
  GamePal{..}    <- reacquire 0 $ initGamePal "Bullet" []

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0

  cubeProg       <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo        <- cubeGeometry (2 :: V3 GLfloat) (V3 1 1 1)
  cubeShape      <- makeShape cubeGeo cubeProg

  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)
  

  glEnable GL_DEPTH_TEST

  glClearColor 0 0 0.1 1

  let initialWorld = World
            (Pose (V3 0 20 40) (axisAngle (V3 0 1 0) 0))
            mempty
  void . flip runStateT initialWorld $ do
    -- Add a falling cube
    rigidBody<- addCube dynamicsWorld (RigidBodyID 11) mempty 
        { pcPosition = V3 0 50 0
        , pcRotation = Quaternion 0 (V3 0 1 0)
        }
    wldCubes . at 11 ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 1 1 1
        }

    forM_ [1..10] $ \i -> do
      rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 (-fromIntegral i * 2.1 + 11) 20 0
        , pcRotation = Quaternion 0 (V3 0 1 0)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 1 1 1
        }
      spring <- addWorldSpringConstraint dynamicsWorld rigidBody
      setSpringLinearLowerLimit spring (-5)
      setSpringLinearUpperLimit spring 5
      setSpringAngularLowerLimit spring (-1)
      setSpringAngularUpperLimit spring 1
      setSpringAngularStiffness spring 100
      setSpringLinearStiffness spring 100
      setSpringLinearDamping spring 0.5
      setSpringAngularDamping spring 0.5
      setSpringLinearBounce spring 10
      setSpringAngularBounce spring 10
      setSpringLinearEquilibrium spring 0
      setSpringAngularEquilibrium spring 0
      setLinearSpringEnabled spring (V3 True True True)
      setAngularSpringEnabled spring (V3 True True True)


    whileWindow gpWindow $ do
      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      -- applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer
      processEvents gpEvents $ \e -> 
        closeOnEscape gpWindow e

      stepSimulation dynamicsWorld

      -- Set all cubes to white
      wldCubes . traverse . cubColor .= V4 1 1 1 1
      -- Set all colliding cubes to green
      collisions <- getCollisions dynamicsWorld
      forM_ collisions $ \collision -> do
        let bodyAID = (fromIntegral . unRigidBodyID . cbBodyAID) collision
        let bodyBID = (fromIntegral . unRigidBodyID . cbBodyBID) collision
        wldCubes . at bodyAID . traverse . cubColor .= V4 0 1 0 1
        wldCubes . at bodyBID . traverse . cubColor .= V4 0 1 0 1

      -- Render Cubes

      projMat <- makeProjection gpWindow
      viewMat <- viewMatrixFromPose <$> use wldPlayer

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
