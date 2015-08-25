import Physics.Bullet
import Linear
import Control.Monad
import Control.Concurrent

main = do

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld 0

  cubeBodies     <- replicateM 1000 $ addCube dynamicsWorld mempty 
        { position = V3 0 20 0
        , rotation = Quaternion 0.5 (V3 0 1 1)
        }

  replicateM_ 600 $ do
    stepSimulation dynamicsWorld
    forM_ cubeBodies $ \rigidBody -> do
      (pos, orient) <- getBodyState rigidBody
      return (pos, orient)
    threadDelay (floor $ 1/60 * 1e6)

  forM_ cubeBodies (removeCube dynamicsWorld)

  cubeBodies2     <- replicateM 1000 $ addCube dynamicsWorld mempty 
        { position = V3 0 20 0
        , rotation = Quaternion 0.5 (V3 0 1 1)
        }

  replicateM_ 600 $ do
    stepSimulation dynamicsWorld
    forM_ cubeBodies2 $ \rigidBody -> do
      (pos, orient) <- getBodyState rigidBody
      return (pos, orient)
    threadDelay (floor $ 1/60 * 1e6)

  forM_ cubeBodies2 (removeCube dynamicsWorld)

  threadDelay 1000000
