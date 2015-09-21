{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}
import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.GL
import Game.Pal
import Linear

import Control.Monad
import Control.Monad.State
import Control.Lens
import Data.Maybe
import Data.Map (Map)
import qualified Data.Map as Map
import System.Random

import Types

import Physics.Bullet

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
  
  GamePal{..} <- initGamePal "Bullet" NoGCPerFrame []

  cubeProg  <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo   <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape <- makeShape cubeGeo cubeProg
  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0
  

  glEnable GL_DEPTH_TEST

  glClearColor 0 0 0.1 1


  void . flip runStateT newWorld $ do 
    forM_ [1..1000] $ \i -> do
      rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 0 20 0
        , pcRotation = Quaternion 0.5 (V3 0 1 1)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        , _cubColor = V4 1 0 1 1
        }
    whileWindow gpWindow $ do
      
      processEvents gpEvents $ \e -> do
        case e of
          (MouseButton _ _ _) -> do mBodyID <- mapM getRigidBodyID =<< rayTestClosest dynamicsWorld =<< posToRay =<< getCursorPos gpWindow
                                    case mBodyID of
                                      Nothing -> return ()
                                      Just bodyID -> do
                                        liftIO $ putStrLn $ "Clicked Cube " ++ (show (unRigidBodyID bodyID))

                                        let cubeID = fromIntegral (unRigidBodyID bodyID)
                                        [r,g,b] <- liftIO (replicateM 3 randomIO)
                                        wldCubes . at cubeID . traverse . cubColor .= V4 r g b 1
          otherwise           -> closeOnEscape gpWindow e
          
      applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer        

      stepSimulation dynamicsWorld

      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      projMat <- makeProjection gpWindow
      viewMat <- viewMatrixFromPose <$> use wldPlayer
      
      uniformV3 uCamera =<< use (wldPlayer . posPosition)

      let viewProj = projMat !*! viewMat

      -- Begin cube batch
      withVAO (sVAO cubeShape) $ do
        cubes <- Map.elems <$> use wldCubes
        forM_ cubes $ \cube -> do
          (position, orientation) <- getBodyState (cube ^. cubBody)

          let model = mkTransformation orientation position
          uniformM44 uModelViewProjection (viewProj !*! model)
          uniformM44 uInverseModel        (fromMaybe model (inv44 model))
          uniformM44 uModel               model
          uniformV4  uDiffuse             (cube ^. cubColor)
          glDrawElements GL_TRIANGLES (vertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

      swapBuffers gpWindow


posToRay (x, y) = do
  pose <- use wldPlayer
  let position = pose ^. posPosition
  return $ Ray position (rotate (pose ^. posOrientation) (position & _z -~ 1000))
