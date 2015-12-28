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
import qualified Data.Set as Set
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
        ghostShapeSize = 10 :: V3 GLfloat
        ghostShapePos = V3 0 5 0
    
    VRPal{..} <- initVRPal "Bullet" []
    
    shader     <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
    cubeGeo    <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
    cubeShape  <- makeShape cubeGeo shader :: IO (Shape Uniforms)
  
    ghostGeo   <- cubeGeometry ghostShapeSize (V3 1 1 1)
    ghostShape <- makeShape ghostGeo shader :: IO (Shape Uniforms)
  
    planeGeo   <- planeGeometry 1000 (V3 0 0 1) (V3 0 1 0) 1
    planeShape <- makeShape planeGeo shader :: IO (Shape Uniforms)
    
  
    dynamicsWorld <- createDynamicsWorld mempty
    _             <- addGroundPlane dynamicsWorld (CollisionObjectID 0) 0

    ghostBox    <- createBoxShape ghostShapeSize
    ghostObject <- addGhostObject dynamicsWorld (CollisionObjectID 1) ghostBox mempty { rbPosition = ghostShapePos }
    
  
    glEnable GL_DEPTH_TEST
    glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA
  
    glClearColor 0 0 0.1 1
  
  
    void . flip runStateT newWorld $ do 
        boxShape <- createBoxShape (1 :: V3 GLfloat)
        forM_ [10..100] $ \i -> do
          rigidBody <- addRigidBody dynamicsWorld (CollisionObjectID i) boxShape mempty 
            { rbPosition = V3 0 20 0
            , rbRotation = Quaternion 0.5 (V3 0 1 1)
            }
          wldCubes . at (fromIntegral i) ?= Cube
            { _cubBody = rigidBody
            , _cubColor = V4 1 0 1 1
            }
        whileWindow gpWindow $ do
            projMat <- getWindowProjection gpWindow fov 0.1 1000
            viewMat <- viewMatrixFromPose <$> use wldPlayer
            (x,y,w,h) <- getWindowViewport gpWindow
            glViewport x y w h
            
            processEvents gpEvents $ \e -> do
                closeOnEscape gpWindow e
                onMouseDown e $ \_ -> do
        
                    playerPose <- use wldPlayer
                    cursorRay  <- cursorPosToWorldRay gpWindow projMat playerPose
        
                    mBodyID <- mapM (getCollisionObjectID . rrCollisionObject) 
                        =<< rayTestClosest dynamicsWorld cursorRay
        
                    forM_ mBodyID $ \bodyID -> do
                        liftIO $ putStrLn $ "Clicked Object " ++ (show (unCollisionObjectID bodyID))
                        let cubeID = fromIntegral (unCollisionObjectID bodyID)
                        [r,g,b] <- liftIO (replicateM 3 randomIO)
                        wldCubes . at cubeID . traverse . cubColor .= V4 r g b 1
            
            applyMouseLook gpWindow wldPlayer
            applyWASD gpWindow wldPlayer        
      
            stepSimulation dynamicsWorld 90
      
            glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)
        

            overlapping <- getGhostObjectOverlapping ghostObject
            overlappingIDs <- Set.fromList <$> mapM getCollisionObjectID overlapping
      
            let viewProj = projMat !*! viewMat
      
            -- Begin cube batch
            withShape cubeShape $ do
                Uniforms{..} <- asks sUniforms
                uniformV3 uCamera =<< use (wldPlayer . posPosition)
                cubes <- Map.toList <$> use wldCubes
        
                forM_ cubes $ \(cubeID, cube) -> do
                  (position, orientation) <- getBodyState (cube ^. cubBody)
        
                  let model = mkTransformation orientation position
                      cubeCollisionID = CollisionObjectID cubeID
                      finalColor = if Set.member cubeCollisionID overlappingIDs 
                                    then V4 1 1 1 1 
                                    else cube ^. cubColor
                  uniformM44 uModelViewProjection (viewProj !*! model)
                  uniformM44 uInverseModel        (inv44 model)
                  uniformM44 uModel               model
                  uniformV4  uDiffuse             finalColor
                  drawShape
      
            withShape planeShape $ do
                Uniforms{..} <- asks sUniforms
                uniformV3 uCamera =<< use (wldPlayer . posPosition)
                let model = planeM44
                uniformM44 uModelViewProjection (viewProj !*! model)
                uniformM44 uInverseModel        (inv44 model)
                uniformM44 uModel               model
                uniformV4  uDiffuse             (V4 0.1 0.0 0.5 1)
                drawShape
      
            glEnable GL_BLEND
            withShape ghostShape $ do
                Uniforms{..} <- asks sUniforms
                uniformV3 uCamera =<< use (wldPlayer . posPosition)
                let model = transformationFromPose (newPose & posPosition .~ ghostShapePos)
                uniformM44 uModelViewProjection (viewProj !*! model)
                uniformM44 uInverseModel        (inv44 model)
                uniformM44 uModel               model
                uniformV4  uDiffuse             (V4 0.5 0.0 0.5 0.5)
                drawShape
            glDisable GL_BLEND
      
            swapBuffers gpWindow
      
  
