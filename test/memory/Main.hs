import Physics.Bullet
import Linear
import Control.Monad
import Control.Concurrent

{-
  Intended to be run with profiling to test if memory is successfully freed
  when adding and deleting cubes.
-}

main = do

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0

  cubeBodies     <- forM [1..1000] $ \i -> addCube dynamicsWorld (RigidBodyID i) mempty 
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

  cubeBodies2     <- forM [1..1000] $ \i -> addCube dynamicsWorld (RigidBodyID i) mempty 
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
