import Physics.Bullet
import Linear.Extra
import Control.Monad
import Control.Concurrent

{-
  Intended to be run with profiling to test if memory is successfully freed
  when adding and deleting cubes.
-}

main :: IO ()
main = do

  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (CollisionObjectID 0) 0
  boxShape       <- createBoxShape (1 :: V3 GLfloat)

  cubeBodies     <- forM [1..1000] $ \i -> 
    addRigidBody dynamicsWorld (CollisionObjectID i) boxShape mempty 
      { rbPosition = V3 0 20 0
      , rbRotation = Quaternion 0.5 (V3 0 1 1)
      }

  replicateM_ 600 $ do
    stepSimulation dynamicsWorld 90
    forM_ cubeBodies $ \rigidBody -> do
      (pos, orient) <- getBodyState rigidBody
      return (pos, orient)
    threadDelay (floor $ 1/60 * 1e6)

  forM_ cubeBodies (removeRigidBody dynamicsWorld)

  cubeBodies2     <- forM [1..1000] $ \i -> 
    addRigidBody dynamicsWorld (CollisionObjectID i) boxShape mempty 
      { rbPosition = V3 0 20 0
      , rbRotation = Quaternion 0.5 (V3 0 1 1)
      }

  replicateM_ 600 $ do
    stepSimulation dynamicsWorld 90
    forM_ cubeBodies2 $ \rigidBody -> do
      (pos, orient) <- getBodyState rigidBody
      return (pos, orient)
    threadDelay (floor $ 1/60 * 1e6)

  forM_ cubeBodies2 (removeRigidBody dynamicsWorld)

  threadDelay 1000000
