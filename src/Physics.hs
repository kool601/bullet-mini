{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
module Physics where
import qualified Language.C.Inline.Cpp as C
import Types
import Control.Monad.State
import Foreign.Ptr
import Control.Lens
import Linear
C.context C.cppCtx

C.include "<iostream>"
C.include "<btBulletDynamicsCommon.h>"

createDynamicsWorld :: (MonadIO m) => m (Ptr ())
createDynamicsWorld = liftIO $ [C.block| void * {
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    return dynamicsWorld;
} |]

addShapes :: (MonadIO m) => Ptr () -> m (Ptr ())
addShapes dynamicsWorld = liftIO $ [C.block| void * { 
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btCollisionShape* fallShape = new btSphereShape(1);


    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(groundRigidBody);


    btDefaultMotionState* fallMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    fallRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(fallRigidBody);
    return fallRigidBody;
} |]

runPhysics :: (MonadState World m, MonadIO m) => Ptr () -> Ptr () -> m ()
runPhysics dynamicsWorld fallRigidBody = do
    
    newY <- liftIO $ [C.block| float {
        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

        btRigidBody* fallRigidBody = (btRigidBody *)$(void *fallRigidBody);

        dynamicsWorld->stepSimulation(1 / 60.f, 10);

        btTransform trans;
        fallRigidBody->getMotionState()->getWorldTransform(trans);

        std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

        return trans.getOrigin().getY();
    } |]

    wldCube . objPosition . _y .= realToFrac newY
    

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