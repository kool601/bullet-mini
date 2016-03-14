{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}
import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.VR.Pal

import Control.Monad
import Control.Monad.State
import Control.Monad.Reader
import Control.Lens.Extra
import Data.Map (Map)
import qualified Data.Map as Map
import System.Random

import Types
import CubeUniforms

import Physics.Bullet

data World = World
  { _wldPlayer :: !(Pose GLfloat)
  , _wldCubes  :: !(Map ObjectID Cube)
  }
makeLenses ''World

newWorld :: World
newWorld = World
    (Pose (V3 0 20 60) (axisAngle (V3 0 1 0) 0))
    mempty

planeM44 :: M44 GLfloat
planeM44 = transformationFromPose $ newPose 
  & posOrientation .~ axisAngle (V3 1 0 0) (-pi/2)

main :: IO ()
main = do

  let fov = 45
  
  vrPal@VRPal{..} <- initVRPal "Bullet" []

  shader    <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo   <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape <- makeShape cubeGeo shader :: IO (Shape Uniforms)

  planeGeo   <- planeGeometry 1000 (V3 0 0 1) (V3 0 1 0) 1
  planeShape <- makeShape planeGeo shader :: IO (Shape Uniforms)
  

  dynamicsWorld  <- createDynamicsWorld mempty
  --_              <- addGroundPlane dynamicsWorld (CollisionObjectID 0) 0
  boxCollider    <- createBoxShape (1 :: V3 GLfloat)

  floorCollider <- createBoxShape (V3 1000 10 1000 :: V3 GLfloat)
  floorBody <- addRigidBody dynamicsWorld (CollisionObjectID 0) floorCollider mempty 
    { rbPosition = V3 0 2 0
    , rbMass = 0 -- i.e. static
    }
  setRigidBodyKinematic floorBody True

  glEnable GL_DEPTH_TEST

  glClearColor 0 0 0.1 1


  void . flip runStateT newWorld $ do 
    
    forM_ [1..1000] $ \i -> do

      rigidBody <- addRigidBody dynamicsWorld (CollisionObjectID i) boxCollider mempty 
        { rbPosition = V3 0 20 0
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 0 1 1
        }
    whileVR vrPal $ \_ _ _ -> do
      projMat <- getWindowProjection gpWindow fov 0.1 1000
      viewMat <- viewMatrixFromPose <$> use wldPlayer
      (x,y,w,h) <- getWindowViewport gpWindow
      glViewport x y w h
      
      processEvents gpEvents $ \e -> do
        closeOnEscape gpWindow e
        
        onMouseDown e $ \_ -> do

            playerPose <- use wldPlayer
            cursorRay  <- cursorPosToWorldRay gpWindow projMat playerPose

            mBodyID <- mapM (getCollisionObjectID . rrCollisionObject) =<< rayTestClosest dynamicsWorld cursorRay

            forM_ mBodyID $ \bodyID -> do
              liftIO $ putStrLn $ "Clicked Object " ++ (show (unCollisionObjectID bodyID))


              let cubeID = fromIntegral (unCollisionObjectID bodyID)
              [r,g,b] <- liftIO (replicateM 3 randomIO)
              wldCubes . at cubeID . traverse . cubColor .= V4 r g b 1
      
      applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer        

      dt <- getDeltaTime vrPal
      --liftIO $ print dt
      stepSimulationSimple dynamicsWorld dt

      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      

      let viewProj = projMat !*! viewMat

      -- Begin cube batch
      withShape cubeShape $ do
        Uniforms{..} <- asks sUniforms
        uniformV3 uCamera =<< use (wldPlayer . posPosition)
        cubes <- Map.elems <$> use wldCubes

        forM_ cubes $ \cube -> do
          (position, orientation) <- getBodyState (cube ^. cubBody)

          let model = mkTransformation orientation position
          uniformM44 uModelViewProjection (viewProj !*! model)
          uniformM44 uModel               model
          uniformV4  uDiffuse             (cube ^. cubColor)
          drawShape

      --withShape planeShape $ do
      --  Uniforms{..} <- asks sUniforms
      --  uniformV3 uCamera =<< use (wldPlayer . posPosition)
      --  let model = planeM44
      --  uniformM44 uModelViewProjection (viewProj !*! model)
      --  uniformM44 uModel               model
      --  uniformV4  uDiffuse             (V4 0.1 0.0 0.5 1)
      --  drawShape

      swapBuffers gpWindow



