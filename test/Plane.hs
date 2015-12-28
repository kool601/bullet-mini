{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE DeriveDataTypeable #-}
import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.VR.Pal

import Control.Monad
import Control.Monad.State
import Control.Lens.Extra
import Data.Maybe
import Data.Map (Map)
import qualified Data.Map as Map
import System.Random

import Types

import Physics.Bullet

data Uniforms = Uniforms
    { uModelViewProjection :: UniformLocation (M44 GLfloat)
    , uInverseModel        :: UniformLocation (M44 GLfloat)
    , uModel               :: UniformLocation (M44 GLfloat)
    , uCamera              :: UniformLocation (V3  GLfloat)
    , uDiffuse             :: UniformLocation (V4  GLfloat)
    , uCubeHit             :: UniformLocation (V3  GLfloat)
    } deriving (Data)

data World = World
    { _wldPlayer   :: !(Pose GLfloat)
    , _wldCubes    :: !(Map ObjectID Cube)
    , _wldCubeHits :: !(Map ObjectID (V3 GLfloat))
    }
makeLenses ''World

newWorld :: World
newWorld = World
    (Pose (V3 0 20 60) (axisAngle (V3 0 1 0) 0))
    mempty
    mempty

main :: IO ()
main = do
    
    let fov = 45

    VRPal{..} <- initVRPal "Bullet" []

    cubeProg  <- createShaderProgram "test/shared/cube.vert" "test/shared/cubeWithHit.frag"
    cubeGeo   <- cubeGeometry (V3 1 1 0.1) 1
    cubeShape <- makeShape cubeGeo cubeProg

    let Uniforms{..} = sUniforms cubeShape
    useProgram (sProgram cubeShape)

    dynamicsWorld  <- createDynamicsWorld mempty
    _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0

    glEnable GL_DEPTH_TEST

    glClearColor 0 0 0.1 1


    void . flip runStateT newWorld $ do 
        forM_ [1..100] $ \i -> do
            rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
                { pcPosition = V3 0 20 0
                , pcRotation = Quaternion 0.5 (V3 0 1 1)
                , pcScale = V3 1 1 0.1
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
      
                    mRayResult <- rayTestClosest dynamicsWorld cursorRay
                    forM_ mRayResult $ \rayResult -> do
                        bodyID <- getRigidBodyID (rrRigidBody rayResult)
                        mCube <- use (wldCubes . at (fromIntegral (unRigidBodyID bodyID)))
                        forM_ mCube $ \cube -> do

                            putStrLnIO $ "Clicked Cube " ++ (show (unRigidBodyID bodyID))
          
          
                            (position, orientation) <- getBodyState (cube ^. cubBody)
                            let model = mkTransformation orientation position
                                -- Convert the hit location into model space
                                -- pointOnModel = worldPointToModelPoint model (rrLocation rayResult)
                                worldHit = rrLocation rayResult
                            -- printIO pointOnModel
          
                            let cubeID = fromIntegral (unRigidBodyID bodyID)
                            [r,g,b] <- liftIO (replicateM 3 randomIO)
                            wldCubes . at cubeID . traverse . cubColor .= V4 r g b 1

                            wldCubeHits . at cubeID ?= worldHit
            
            applyMouseLook gpWindow wldPlayer
            applyWASD gpWindow wldPlayer        
    
            stepSimulation dynamicsWorld 90
    
            glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)
    
            uniformV3 uCamera =<< use (wldPlayer . posPosition)
    
            let viewProj = projMat !*! viewMat
    
            withVAO (sVAO cubeShape) $ do
                cubes <- Map.toList <$> use wldCubes
                forM_ cubes $ \(cubeID, cube) -> do
                    (position, orientation) <- getBodyState (cube ^. cubBody)
                    
                    mCubeHit <- use (wldCubeHits . at cubeID)
                    forM_ mCubeHit $ \cubeHit ->
                        uniformV3 uCubeHit cubeHit

                    let model = mkTransformation orientation position
                    uniformM44 uModelViewProjection (viewProj !*! model)
                    uniformM44 uInverseModel        (safeInv44 model)
                    uniformM44 uModel               model
                    uniformV4  uDiffuse             (cube ^. cubColor)

                    glDrawElements GL_TRIANGLES (geoVertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr
                    
            swapBuffers gpWindow

