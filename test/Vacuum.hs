{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.GL
import Game.Pal
import Physics.Bullet
import Linear.Extra

import Control.Monad
import Control.Monad.State
import Control.Lens.Extra
import qualified Data.Map as Map
import Data.Map (Map)
import Data.Maybe

import Data.Time

import CubeUniforms

import Halive.Utils

type ObjectID = Int

data Cube = Cube
  { _cubColor :: !(V4 GLfloat)
  , _cubBody  :: !RigidBody
  , _cubScale :: !(V3 GLfloat)
  }
makeLenses ''Cube

data World = World
  { _wldPlayer :: !(Pose GLfloat)
  , _wldCubes  :: !(Map ObjectID Cube)
  }
makeLenses ''World

main :: IO ()
main = do
  GamePal{..}    <- reacquire 0 $ initGamePal "Bullet" NoGCPerFrame []

  dynamicsWorld  <- createDynamicsWorld mempty { gravity = 0 }

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

    _ <- addStaticRoom dynamicsWorld (RigidBodyID 0) 10

    forM_ [1..1000] $ \i -> do
      rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 0 20 0
        , pcRotation = Quaternion 0.5 (V3 0 1 1)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 0 1 1
        , _cubScale = 1
        }

    whileWindow gpWindow $ do
      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      -- applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer
      processEvents gpEvents $ \e -> do
        closeOnEscape gpWindow e
        spaceDown <- (== KeyState'Pressed) <$> getKey gpWindow Key'Space
        let target = V3 0 20 0
        when spaceDown $ do
          cubes <- Map.elems <$> use wldCubes
          forM_ cubes $ \cube -> do
            (position, _orientation) <- getBodyState (cube ^. cubBody)
            -- let magnitude = distance position target
            let orientation = normalize (target - (position :: V3 GLfloat)) :: V3 GLfloat
            --let v = rotate (cube ^. cubPose . posOrientation) ( V3 0 0 ( -3 ) )
            setRigidBodyActive (cube ^. cubBody)
            _ <- applyCentralImpulse (cube ^. cubBody) orientation
            return ()


      
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

          let model = mkTransformation orientation position !*! scaleMatrix (cube ^. cubScale)
          uniformM44 uModelViewProjection (viewProj !*! model)
          uniformM44 uInverseModel        (fromMaybe model (inv44 model))
          uniformM44 uModel               model
          uniformV4  uDiffuse             (cube ^. cubColor)
          glDrawElements GL_TRIANGLES (vertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

      swapBuffers gpWindow
