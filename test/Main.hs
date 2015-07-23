{-# LANGUAGE FlexibleContexts, LambdaCase #-}
import Graphics.UI.GLFW.Pal
-- import qualified Graphics.UI.GLFW as GLFW
import Halive.Utils
import Graphics.GL.Pal
import Graphics.GL
import Linear
import Cube

import Control.Monad
import Control.Monad.State
import System.Random
import Control.Lens
import Foreign (nullPtr)
import qualified Data.Map as Map
import Data.Map (Map)
import Control.Monad.Random
import Types
import Physics.Bullet


newPlayer :: Player
newPlayer = Player (V3 0 20 60) (axisAngle (V3 0 1 0) 0)

newWorld :: World
newWorld = World newPlayer mempty

-- | Get a view matrix for a camera at a given position and orientation
viewMatrix :: (RealFloat a, Conjugate a) => V3 a -> Quaternion a -> M44 a
viewMatrix position orientation = mkTransformation q (rotate q . negate $ position)
    where q = conjugate orientation

-- | Use the aspect ratio from the window to get a proper projection
makeProjection :: (Floating a, MonadIO m) => Window -> m (M44 a)
makeProjection win = do
    (w,h) <- getWindowSize win
    return $ perspective 45 (fromIntegral w / fromIntegral h) 0.01 1000

playerViewMat :: MonadState World m => m (M44 GLfloat)
playerViewMat = do
    playerPos    <- use $ wldPlayer . plrPosition
    playerOrient <- use $ wldPlayer . plrOrientation
    return $ viewMatrix playerPos playerOrient

applyMouseLook :: (MonadIO m, MonadState World m) => Window -> m ()
applyMouseLook win = do
    (x,y) <- getCursorPos win
    wldPlayer . plrOrientation .= axisAngle (V3 0 1 0) (-x/500)
                                * axisAngle (V3 1 0 0) (-y/500)

movePlayer :: MonadState World m => V3 GLfloat -> m ()
movePlayer vec = do
    orient <- use (wldPlayer . plrOrientation)
    wldPlayer . plrPosition += rotate orient vec

applyMovement :: (MonadIO m, MonadState World m) => Window -> m ()
applyMovement win = do
    let pos = 0.1
        neg = -pos
    whenKeyPressed win Key'W           $ movePlayer (V3 0   0   neg)
    whenKeyPressed win Key'S           $ movePlayer (V3 0   0   pos)
    whenKeyPressed win Key'A           $ movePlayer (V3 neg 0   0  )
    whenKeyPressed win Key'D           $ movePlayer (V3 pos 0   0  )
    whenKeyPressed win Key'Space       $ movePlayer (V3 0   pos 0  )
    whenKeyPressed win Key'LeftControl $ movePlayer (V3 0   neg 0  )

whenKeyPressed :: MonadIO m => Window -> Key -> m () -> m ()
whenKeyPressed win key action = getKey win key >>= \case
    KeyState'Pressed -> action
    _                -> return ()

main :: IO ()
main = do
    (win, events) <- reacquire 0 $ createWindow "Bullet" 1024 768

    cubeProg <- createShaderProgram "test/cube.vert" "test/cube.frag"
    cube     <- makeCube cubeProg

    dynamicsWorld  <- createDynamicsWorld mempty
    _              <- addGroundPlane dynamicsWorld 0
    cubeBodies     <- replicateM 1000 $ addCube dynamicsWorld mempty 
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
