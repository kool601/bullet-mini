{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}

module Physics.Bullet where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Foreign.Ptr
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear
import Control.Monad.Trans
import Data.Monoid
import Control.Applicative

C.context C.cppCtx

C.include "<btBulletDynamicsCommon.h>"

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

data PhysicsConfig = PhysicsConfig
  { restitution :: Float
  , position    :: V3 Float
  , rotation    :: Quaternion Float
  , scale       :: V3 Float
  , inertia     :: V3 Float
  , mass        :: Float
  , yPos        :: Float
  }


instance Monoid PhysicsConfig where
  mempty = PhysicsConfig 
        { position    = V3 0 0 0
        , scale       = V3 1 1 1
        , inertia     = V3 0 0 0
        , rotation    = axisAngle ( V3 1 0 0 ) 0
        , mass        = 1
        , restitution = 0.5
        , yPos        = 0
        }
  mappend _ b = b


data PhysicsWorldConfig = PhysicsWorldConfig
  { gravity :: Float
  }
instance Monoid PhysicsWorldConfig where
  mempty = PhysicsWorldConfig 
        { gravity = -9.8
        }
  mappend _ b = b

newtype DynamicsWorld = DynamicsWorld { unDynamicsWorld :: Ptr () }
newtype RigidBody = RigidBody { unRigidBody :: Ptr () }

createDynamicsWorld :: (Functor m, MonadIO m) =>  PhysicsWorldConfig -> m DynamicsWorld 
createDynamicsWorld PhysicsWorldConfig{..} = DynamicsWorld <$> liftIO [C.block| void * {

  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0,  $( float g ), 0));

  return dynamicsWorld;

  } |]
  where
    g = realToFrac gravity


-- Ground plane should always be infinite!
addGroundPlane :: (Functor m, MonadIO m) => DynamicsWorld -> Float -> m RigidBody
addGroundPlane dynamicsWorld height =
  addStaticPlane dynamicsWorld mempty { rotation = axisAngle ( V3 1 0 0 ) ((-pi)/2) , yPos = height }


-- Create a static plane using PhysicsConfig
-- Making sure pass in all of the information from the physics config
-- as usable c++ data
addStaticPlane :: (Functor m, MonadIO m) => DynamicsWorld -> PhysicsConfig -> m RigidBody
addStaticPlane (DynamicsWorld dynamicsWorld) PhysicsConfig{..} = RigidBody <$> liftIO [C.block| void * {

  btDiscreteDynamicsWorld* dynamicsWorld = ( btDiscreteDynamicsWorld* ) $( void *dynamicsWorld );

  // Create a ground plane
  btCollisionShape* collider = new btStaticPlaneShape( btVector3( 0 , 0 , 1 ) , $( float yP ));

  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );

  btDefaultMotionState* motionState = new btDefaultMotionState( btTransform( q , p ) );

  btRigidBody::btRigidBodyConstructionInfo
      constructionInfo( 0 , motionState , collider , btVector3( 0 , 0 , 0 ) );

  btRigidBody* rigidBody = new btRigidBody( constructionInfo );  

  rigidBody         -> setRestitution( $(float r) );
  dynamicsWorld     -> addRigidBody( rigidBody );

  return rigidBody;

  } |] 
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation
    r                             = realToFrac     restitution
    yP                            = realToFrac     yPos





setRigidBodyKinematic (RigidBody rigidBody)  = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  rigidBody->setActivationState(DISABLE_DEACTIVATION);
  }|]

setRigidBodyWorldTransform (RigidBody rigidBody) position rotation = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);

  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );
  rigidBody -> getMotionState() -> setWorldTransform( btTransform(q , p) );

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation

addCube :: (Functor m, MonadIO m) => DynamicsWorld -> PhysicsConfig -> m RigidBody

addCube (DynamicsWorld dynamicsWorld) PhysicsConfig{..} = RigidBody <$> liftIO [C.block| void * {

  btDiscreteDynamicsWorld* dynamicsWorld = ( btDiscreteDynamicsWorld* ) $( void *dynamicsWorld );

  // Create a box
  btVector3 s = btVector3( $( float sx ) , $( float sy ) , $( float sz ) );
  btCollisionShape* collider = new btBoxShape( s );

  // MotionStates are for communicating transforms between our engine and Bullet; we're not using them
  // yet so we just use the btDefaultMotionState
  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );

  btDefaultMotionState* motionState = new btDefaultMotionState( btTransform( q , p ) );

  btScalar mass     = $( float m );
  btVector3 inertia = btVector3( $( float ix ) , $( float iy ) , $( float iz ) );

  collider -> calculateLocalInertia( mass , inertia );

  btRigidBody::btRigidBodyConstructionInfo rigidBodyCI( mass , motionState , collider , inertia );
  btRigidBody* rigidBody = new btRigidBody( rigidBodyCI );

  rigidBody         -> setRestitution( $(float r) );
  dynamicsWorld     -> addRigidBody( rigidBody );

  return rigidBody;

  } |]
  where
    (V3 x y z)                    = realToFrac <$> position
    (V3 sx sy sz)                 = realToFrac <$> scale
    (V3 ix iy iz)                 = realToFrac <$> inertia
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation
    r                             = realToFrac     restitution
    m                             = realToFrac     mass

removeCube :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBody -> m ()
removeCube (DynamicsWorld dynamicsWorld) (RigidBody rigidBody) = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );
  btRigidBody*             rigidBody     = (btRigidBody *) $(void *rigidBody);

  dynamicsWorld->removeRigidBody(rigidBody);
  delete rigidBody->getMotionState();
  delete rigidBody;
  
  }|]

applyCentralForce :: (Functor m, MonadIO m, Real a) => RigidBody -> V3 a -> m ()
applyCentralForce (RigidBody rigidBody) force = liftIO [C.block| void {

  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);

  btVector3 force = btVector3( $(float x) , $(float y) , $(float z) );
  rigidBody -> applyCentralImpulse( force );

  } |]
  where
    (V3 x y z) = realToFrac <$> force


stepSimulation :: MonadIO m => DynamicsWorld -> m ()
stepSimulation (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {

    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation(1 / 60.f, 10);

  } |]

getCollisions :: MonadIO m => DynamicsWorld -> m ()
getCollisions (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
  
  int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
  for (int i = 0; i < numManifolds; i++) {

    btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
    const btCollisionObject* obB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

    int numContacts = contactManifold->getNumContacts();
    for (int j = 0; j < numContacts; j++) {

      btManifoldPoint& pt = contactManifold->getContactPoint(j);
      
      if (pt.getDistance()<0.f) {
        printf("Collision!\n");
        const btVector3& ptA = pt.getPositionWorldOnA();
        const btVector3& ptB = pt.getPositionWorldOnB();
        const btVector3& normalOnB = pt.m_normalWorldOnB;
      }
    }
  }


  } |]

getBodyState :: (Fractional a, MonadIO m) => RigidBody -> m (V3 a, Quaternion a)
getBodyState (RigidBody rigidBody) = do

  -- Should probably use a mutable vector per shape and rewrite it each tick to avoid alloc
  -- (can pass it in to inline-c with withPtr_)

  -- Should also just get all bodies in one big array rather than FFI-calling/allocing for each one.

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
