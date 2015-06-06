{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
module Physics.Bullet where
import qualified Language.C.Inline.Cpp as C
import Foreign.C
import Foreign.Ptr
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear
import Control.Monad.Trans

C.context C.cppCtx

C.include "<iostream>"
C.include "<btBulletDynamicsCommon.h>"

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

newtype DynamicsWorld = DynamicsWorld { unDynamicsWorld :: Ptr () }
newtype RigidBody = RigidBody { unRigidBody :: Ptr () }

createDynamicsWorld :: (MonadIO m) => m DynamicsWorld
createDynamicsWorld = DynamicsWorld <$> liftIO [C.block| void * {
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  return dynamicsWorld;
} |]

addGroundPlane :: (MonadIO m) => DynamicsWorld -> m RigidBody
addGroundPlane (DynamicsWorld dynamicsWorld) = RigidBody <$> liftIO [C.block| void * {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

  // Create a ground plane
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
      groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  groundRigidBody->setRestitution(0.5);
  dynamicsWorld->addRigidBody(groundRigidBody);
  return groundRigidBody;
  } |]

addCube :: (MonadIO m, RealFrac a) => DynamicsWorld -> V3 a -> Quaternion a -> m RigidBody
addCube (DynamicsWorld dynamicsWorld) position orientation = RigidBody <$> liftIO [C.block| void * {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

  // Create a box
  btCollisionShape* cubeShape = new btBoxShape(btVector3(1,1,1));

  // MotionStates are for communicating transforms between our engine and Bullet; we're not using them
  // yet so we just use the btDefaultMotionState
  btDefaultMotionState* cubeMotionState =
      new btDefaultMotionState(btTransform(
          btQuaternion($(float qx), $(float qy), $(float qz), $(float qw)), 
          btVector3($(float x), $(float y), $(float z))));
  btScalar mass = 1;
  btVector3 cubeInertia(0, 0, 0);
  cubeShape->calculateLocalInertia(mass, cubeInertia);
  btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(mass, cubeMotionState, cubeShape, cubeInertia);
  btRigidBody* cubeRigidBody = new btRigidBody(cubeRigidBodyCI);
  cubeRigidBody->setRestitution(0.5);
  dynamicsWorld->addRigidBody(cubeRigidBody);
  return cubeRigidBody;
  } |]
  where
    (V3 x y z) = fmap realToFrac position
    (Quaternion qw (V3 qx qy qz)) = fmap realToFrac orientation


applyCentralForce :: Real a => RigidBody -> V3 a -> IO (Ptr ())
applyCentralForce (RigidBody rigidBody) force = [C.block| void * {
    btRigidBody* rigidBody = (btRigidBody *)$(void *rigidBody);
    rigidBody->applyCentralImpulse(btVector3($(float x), $(float y), $(float z)));
  } |]
  where
    (V3 x y z) = realToFrac <$> force


stepSimulation :: MonadIO m => DynamicsWorld -> m ()
stepSimulation (DynamicsWorld dynamicsWorld) =
  liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation(1 / 60.f, 10);
  } |]

getBodyState :: (Fractional a, MonadIO m) => RigidBody -> m (V3 a, Quaternion a)
getBodyState (RigidBody rigidBody) = do
  -- Should probably use a mutable vector per shape and rewrite it each tick to avoid alloc
  -- (can pass it in to inline-c with withPtr_)
  ptr <- liftIO $ newForeignPtr freePtr =<< [C.block| float * {

    btRigidBody* rigidBody = (btRigidBody *)$(void *rigidBody);

    btTransform trans;
    rigidBody->getMotionState()->getWorldTransform(trans);

    btScalar *transformPtr = (btScalar *)malloc(sizeof(btScalar) * 7);
    transformPtr[0] = trans.getOrigin().getX();
    transformPtr[1] = trans.getOrigin().getY();
    transformPtr[2] = trans.getOrigin().getZ();
    transformPtr[3] = trans.getRotation().getX();
    transformPtr[4] = trans.getRotation().getY();
    transformPtr[5] = trans.getRotation().getZ();
    transformPtr[6] = trans.getRotation().getW();

    return transformPtr;
  } |]
  [x,y,z,qx,qy,qz,qw] <- liftIO $ withForeignPtr ptr (peekArray 7)
  
  let position    = V3 (realToFrac x) (realToFrac y) (realToFrac z)
      orientation = Quaternion (realToFrac qw) (V3 (realToFrac qx) (realToFrac qy) (realToFrac qz))
  return (position, orientation)

{-
removeRigidBody dynamicsWorld rigidBody = [C.block| void {
  dynamicsWorld->removeRigidBody(rigidBody);
  delete rigidBody->getMotionState();
  delete rigidBody;

  // Should handle this separately.
  delete rigidBodyShape;
}|]
-}

-- I'm guessing I can get the solver/collisionConfig/dispatcher/broadphase from pointers in the dynamicsWorld
destroyDynamicsWorld :: DynamicsWorld -> IO ()
destroyDynamicsWorld (DynamicsWorld dynamicsWorld) = [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
  //delete dynamicsWorld;
  //delete solver;
  //delete collisionConfiguration;
  //delete dispatcher;
  //delete broadphase;
} |]
