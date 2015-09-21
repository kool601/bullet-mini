{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.GL
import Linear
import Game.Pal

import Control.Monad
import Control.Monad.State
import Control.Lens
import qualified Data.Map as Map
import Data.Map (Map)

import Data.Maybe

import Types

import Physics.Bullet
import Control.Lens

data World = World
  { _wldPlayer :: Pose
  , _wldCubes  :: Map ObjectID Cube
  }
makeLenses ''World

newWorld :: World
newWorld = World
    (Pose (V3 0 20 60) (axisAngle (V3 0 1 0) 0))
    mempty

main :: IO ()
main = do
  GamePal{..}    <- initGamePal "Bullet" NoGCPerFrame []

  cubeProg       <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo        <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape      <- makeShape cubeGeo cubeProg

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0
  

  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)
  glEnable GL_DEPTH_TEST

  glClearColor 0 0 0.1 1

  void . flip runStateT newWorld $ do

    forM_ [1..10] $ \i -> do
      rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 0 20 0
        , pcRotation = Quaternion 0.5 (V3 0 1 1)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 1 1 1
        }

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
            bodyBID = (fromIntegral . unRigidBodyID . cbBodyBID) collision
            appliedImpulse = cbAppliedImpulse collision
        wldCubes . at bodyAID . traverse . cubColor .= V4 1 appliedImpulse 1 1
        wldCubes . at bodyBID . traverse . cubColor .= V4 1 appliedImpulse 1 1


      projMat <- makeProjection gpWindow
      viewMat <- viewMatrixFromPose <$> use wldPlayer

      let viewProj = projMat !*! viewMat

      -- Begin cube batch

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
