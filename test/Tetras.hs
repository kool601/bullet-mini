{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}
import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal 
import Graphics.GL.Pal.Geometry
import Graphics.VR.Pal hiding (getNow)

import Control.Monad
import Control.Monad.State
import Control.Lens.Extra
import Data.Map (Map)
import qualified Data.Map as Map
import System.Random

import Types
import CubeUniforms
import Physics.Bullet

import qualified Data.Vector.Storable as V

data World = World
  { _wldPlayer :: !(Pose GLfloat)
  , _wldCubes  :: !(Map ObjectID Cube)
  }
makeLenses ''World

newWorld :: World
newWorld = World
    (Pose (V3 0 20 60) (axisAngle (V3 0 1 0) 0))
    mempty


main :: IO ()
main = do
  
    let fov = 45
    
    vrPal@VRPal{..} <- initVRPal "Bullet" []
    
    shader    <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
    uniforms@Uniforms{..} <- acquireUniforms shader


    let tetraData = tetrahedronData 1
    tetraGeo   <- geometryFromData tetraData
    tetraShape <- makeShape tetraGeo shader :: IO (Shape Uniforms)
        
    cubeGeo   <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
    cubeShape <- makeShape cubeGeo shader :: IO (Shape Uniforms)

    (lineVAO, lineBuffer, _lineVertCount) <- makeLine shader


    dynamicsWorld  <- createDynamicsWorld mempty
        
    --let invert = (rotate (axisAngle (V3 1 0 0) pi))
    tetraCollider    <- createConvexHullShape (V.fromList $ gdPositions tetraData)
    
    let floorDims = V3 100 1 100 :: V3 GLfloat
        floorPos  = V3 0 2 0
    floorCollider <- createBoxShape floorDims
    floorBody <- addRigidBody dynamicsWorld (CollisionObjectID 0) floorCollider mempty 
        { rbPosition = floorPos
        , rbMass = 0 -- i.e. static
        }
    setRigidBodyKinematic floorBody True
  
    glEnable GL_DEPTH_TEST
  
    glClearColor 0 0 0.1 1
  
  
    void . flip runStateT newWorld $ do 
        
        forM_ [1..100] $ \i -> do
    
            rigidBody <- addRigidBody dynamicsWorld (CollisionObjectID i) tetraCollider mempty 
                { rbPosition = V3 0 20 0
                , rbMass = 10
                , rbRestitution = 0.1
                }
            [r,g,b] <- liftIO (replicateM 3 randomIO)
            
            wldCubes . at (fromIntegral i) ?= Cube
                { _cubBody = rigidBody
                --, _cubColor = V4 1 0 1 1
                , _cubColor = V4 r g b 1
                }
        whileWindow gpWindow $ do
            playerPose <- use wldPlayer
            (_, events) <- tickVR vrPal (transformationFromPose playerPose)
            projMat <- getWindowProjection gpWindow fov 0.1 1000
            viewMat <- viewMatrixFromPose <$> use wldPlayer
            (x,y,w,h) <- getWindowViewport gpWindow
            glViewport x y w h
            
            forM_ events $ \case
                VREvent _ -> return ()
                GLFWEvent e -> do
                    closeOnEscape gpWindow e
                    
                    onMouseDown e $ \_ -> do
            
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
            stepSimulationSimple dynamicsWorld dt


      
            glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)
      
            let viewProj = projMat !*! viewMat
            
            cameraPos <- use (wldPlayer . posPosition)
            ---- Begin cube batch
            withShape tetraShape $ do
                uniformV3 uCamera cameraPos
                cubes <- Map.elems <$> use wldCubes
        
                forM_ cubes $ \cube -> do
                    (position, orientation) <- getBodyState (cube ^. cubBody)
          
                    let model = mkTransformation orientation position
                    uniformM44 uModelViewProjection (viewProj !*! model)
                    uniformM44 uModel               model
                    uniformV4  uDiffuse             (cube ^. cubColor)
                    drawShape

            -- Draw floor
            withShape cubeShape $ do
                uniformV3 uCamera cameraPos
                let model = mkTransformation (axisAngle (V3 0 1 0) 0) floorPos !*! scaleMatrix floorDims
                uniformM44 uModelViewProjection (viewProj !*! model)
                uniformM44 uModel               model
                uniformV4  uDiffuse             (V4 1 1 1 1)

                drawShape

            -- Debug drawing
            debugDrawPhysics shader uniforms cameraPos viewProj dynamicsWorld lineBuffer lineVAO            

            swapBuffers gpWindow

debugDrawPhysics :: MonadIO m => Program
                              -> Uniforms
                              -> V3 GLfloat
                              -> M44 GLfloat
                              -> DynamicsWorld
                              -> ArrayBuffer (V3 GLfloat)
                              -> VertexArrayObject
                              -> m ()
debugDrawPhysics shader Uniforms{..} cameraPos viewProj dynamicsWorld lineBuffer lineVAO = do
    useProgram shader
    uniformV3  uCamera              cameraPos 
    uniformM44 uModelViewProjection viewProj
    uniformM44 uModel               identity
    
    glDisable GL_DEPTH_TEST
    debugDrawDynamicsWorld dynamicsWorld $ \pt1 pt2 color -> do
        uniformV4  uDiffuse             color
        bufferSubData lineBuffer ([pt1, pt2] :: [V3 GLfloat])
        withVAO lineVAO $ 
            glDrawArrays GL_LINE_STRIP 0 2
    glEnable GL_DEPTH_TEST

makeLine :: Program -> IO (VertexArrayObject, ArrayBuffer (V3 GLfloat), GLsizei)
makeLine shader = do

    let verts = map (\x -> V3 x 0 0) [0..1]
        vertCount = length verts
        normals = replicate vertCount (V3 0 0 1)
    
    positionsBuffer <- bufferData GL_DYNAMIC_DRAW (verts :: [V3 GLfloat])
    normalsBuffer   <- bufferData GL_STATIC_DRAW (normals :: [V3 GLfloat])
  
    vao <- newVAO
    withVAO vao $ do
        withArrayBuffer positionsBuffer $ assignFloatAttribute shader "aPosition" GL_FLOAT 3
        withArrayBuffer normalsBuffer   $ assignFloatAttribute shader "aNormal"   GL_FLOAT 3
  
    return (vao, positionsBuffer, fromIntegral vertCount)

randomVerts :: (MonadIO m, Integral a, Fractional b, Random b) 
            => a -> m [V3 b]
randomVerts lineVertCount = liftIO $ forM [0..lineVertCount-1] $ \i -> do
    let x = fromIntegral i / fromIntegral lineVertCount
        x' = x * 2 - 1
    y <- randomIO
    return (V3 x' y 0)

