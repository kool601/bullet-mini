{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
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

import Types

import Physics.Bullet

main :: IO ()
main = do
  
  GamePal{..} <- initGamePal "Bullet" []

  cubeProg  <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo   <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape <- makeShape cubeGeo cubeProg
  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0
  cubeBodies     <- forM [1..1000] $ \i -> addCube dynamicsWorld (RigidBodyID i) mempty 
    { pcPosition = V3 0 20 0
    , pcRotation = Quaternion 0.5 (V3 0 1 1)
    }

  glEnable GL_DEPTH_TEST

  glClearColor 0 0 0.1 1
  void . flip runStateT newWorld . whileWindow gpWindow $ do
    processEvents gpEvents $ \e -> do
      closeOnEscape gpWindow e
    applyMouseLook gpWindow wldPlayer
    applyWASD gpWindow wldPlayer        

    stepSimulation dynamicsWorld

    glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

    projMat <- makeProjection gpWindow
    viewMat <- viewMatrixFromPose <$> use wldPlayer

    let viewProj = projMat !*! viewMat

    -- Begin cube batch
    
    withVAO (sVAO cubeShape) $ 
      forM_ cubeBodies $ \rigidBody -> do
        (position, orientation) <- getBodyState rigidBody

        let model = mkTransformation orientation position
        uniformM44 uModelViewProjection (viewProj !*! model)
        uniformM44 uInverseModel        (fromMaybe model (inv44 model))
        uniformM44 uModel               model
        uniformV4  uDiffuse             (V4 1 0 1 1)
        glDrawElements GL_TRIANGLES (vertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

    swapBuffers gpWindow
