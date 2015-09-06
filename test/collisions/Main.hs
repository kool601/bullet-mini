{-# LANGUAGE FlexibleContexts, LambdaCase #-}
import Graphics.UI.GLFW.Pal
-- import qualified Graphics.UI.GLFW as GLFW
import Graphics.GL.Pal
import Graphics.GL
import Linear

import Control.Monad
import Control.Monad.State
import System.Random
import Control.Lens
import Foreign (nullPtr)
import qualified Data.Map as Map
import Data.Map (Map)
import Control.Monad.Random

import Types
import Cube
import Movement

import Physics.Bullet



main :: IO ()
main = do
    (win, events) <- createWindow "Bullet" 1024 768

    cubeProg <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
    cube     <- makeCube cubeProg

    dynamicsWorld  <- createDynamicsWorld mempty
    _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0
    cubeBodies     <- forM [1..10] $ \i -> addCube dynamicsWorld (RigidBodyID i) mempty 
        { position = V3 0 20 0
        , rotation = Quaternion 0.5 (V3 0 1 1)
        }

    glEnable GL_DEPTH_TEST

    glClearColor 0 0 0.1 1
    stdGen <- getStdGen
    void . flip runRandT stdGen . flip runStateT newWorld . whileWindow win $ do
        processEvents events $ \e -> do
            closeOnEscape win e
            return ()
            -- keyDown Key'Enter e addCube

        -- applyMouseLook win
        applyMovement win

        stepSimulation dynamicsWorld

        liftIO . print =<< getCollisions dynamicsWorld

        glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

        projMat <- makeProjection win
        viewMat <- playerViewMat

        let viewProj = projMat !*! viewMat

        -- Begin cube batch
        useProgram (cubeShader cube)
        glBindVertexArray (unVertexArrayObject (cubeVAO cube))

        forM_ cubeBodies $ \rigidBody -> do
            (pos, orient) <- getBodyState rigidBody
            let obj = Object pos orient
            let model = mkTransformation (obj ^. objOrientation) (obj ^. objPosition)
            uniformM44 (cubeUniformMVP cube) (viewProj !*! model)
            glDrawElements GL_TRIANGLES (cubeIndexCount cube) GL_UNSIGNED_INT nullPtr

        swapBuffers win
