{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
module Physics where
import qualified Language.C.Inline.Cpp as C
import Types
import Control.Monad.State
import Foreign.C
import Foreign.Ptr
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Control.Lens
import Linear
C.context C.cppCtx

C.include "<iostream>"
C.include "<btBulletDynamicsCommon.h>"

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

newtype DynamicsWorld = DynamicsWorld { unDynamicsWorld :: Ptr () }
newtype RigidBody = RigidBody { unRigidBody :: Ptr () }

createDynamicsWorld :: (MonadIO m) => m (DynamicsWorld)
createDynamicsWorld = DynamicsWorld <$> liftIO [C.block| void * {
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    return dynamicsWorld;
} |]

addShapes :: (MonadIO m) => DynamicsWorld -> m (RigidBody)
addShapes (DynamicsWorld dynamicsWorld) = RigidBody <$> liftIO [C.block| void * { 
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

    // Create a ground plane
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(groundRigidBody);

    // Create a box
    btCollisionShape* fallShape = new btBoxShape(btVector3(0.5,0.5,0.5));

    btDefaultMotionState* fallMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 1, 1, 0.5), btVector3(0, 20, 0)));
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    fallRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(fallRigidBody);
    return fallRigidBody;
} |]

stepSimulation :: (MonadState World m, MonadIO m) => DynamicsWorld -> m ()
stepSimulation (DynamicsWorld dynamicsWorld) = 
    liftIO [C.block| void {
        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
        dynamicsWorld->stepSimulation(1 / 60.f, 10);
    } |]

updateBody (RigidBody rigidBody) = do
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

    wldCube . objPosition    .= V3 (realToFrac x) (realToFrac y) (realToFrac z)
    wldCube . objOrientation .= Quaternion (realToFrac qw) (V3 (realToFrac qx) (realToFrac qy) (realToFrac qz))    

{-
removeShape dynamicsWorld rigidBody = [C.block| void {
    dynamicsWorld->removeRigidBody(fallRigidBody);
    delete fallRigidBody->getMotionState();
    delete fallRigidBody;

    dynamicsWorld->removeRigidBody(groundRigidBody);
    delete groundRigidBody->getMotionState();
    delete groundRigidBody;


    delete fallShape;

    delete groundShape;
}|]
-}

-- I'm guessing I can get the solver/collisionConfig/dispatcher/broadphase from pointers in the dynamicsWorld
destroyDynamicsWorld dynamicsWorld = [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    //delete dynamicsWorld;
    //delete solver;
    //delete collisionConfiguration;
    //delete dispatcher;
    //delete broadphase;
} |]