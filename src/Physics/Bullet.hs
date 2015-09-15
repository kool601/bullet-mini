 {-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}

module Physics.Bullet where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Foreign.Ptr
-- import Foreign.StablePtr
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear
import Control.Monad.Trans
import Data.Monoid
-- import Control.Applicative
import Data.IORef

{-
See 

http://bulletphysics.org/Bullet/BulletFull/

https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf

-}

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

data PhysicsConfig = PhysicsConfig
  { pcRestitution :: Float
  , pcPosition    :: V3 Float
  , pcRotation    :: Quaternion Float
  , pcScale       :: V3 Float
  , pcInertia     :: V3 Float
  , pcMass        :: Float
  , pcYPos        :: Float
  }


instance Monoid PhysicsConfig where
  mempty = PhysicsConfig 
        { pcPosition    = V3 0 0 0
        , pcScale       = V3 1 1 1
        , pcInertia     = V3 0 0 0
        , pcRotation    = axisAngle ( V3 1 0 0 ) 0
        , pcMass        = 1
        , pcRestitution = 0.5
        , pcYPos        = 0
        }
  mappend _ b = b


data PhysicsWorldConfig = PhysicsWorldConfig
  { gravity :: Float
  }
-- We're just implementing this for mempty;
-- the mappend instance isn't useful as it just takes the rightmost
-- PhysicsWorldConfig.
instance Monoid PhysicsWorldConfig where
  mempty = PhysicsWorldConfig 
        { gravity = -9.8
        }
  mappend _ b = b

newtype DynamicsWorld    = DynamicsWorld { unDynamicsWorld :: Ptr () }
newtype RigidBody        = RigidBody     { unRigidBody     :: Ptr () } deriving Show
newtype RigidBodyID      = RigidBodyID   { unRigidBodyID   :: CInt   } deriving (Eq, Show, Ord)

newtype SpringConstraint = SpringConstraint { unSpringConstraint :: Ptr () } deriving Show

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
addGroundPlane :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBodyID -> Float -> m RigidBody
addGroundPlane dynamicsWorld rigidBodyID height  =
  addStaticPlane dynamicsWorld rigidBodyID mempty { pcRotation = axisAngle ( V3 1 0 0 ) ((-pi)/2) , pcYPos = height }


-- Create a static plane using PhysicsConfig
-- Making sure pass in all of the information from the physics config
-- as usable c++ data
addStaticPlane :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBodyID -> PhysicsConfig -> m RigidBody
addStaticPlane (DynamicsWorld dynamicsWorld) (RigidBodyID rigidBodyID) PhysicsConfig{..} = liftIO $ 
  RigidBody <$> [C.block| void * {

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

    // Attach the given RigidBodyID
    rigidBody         -> setUserIndex( $(int rigidBodyID) );

    dynamicsWorld     -> addRigidBody( rigidBody );

    return rigidBody;

    } |] 
  where
    (V3 x y z)                    = realToFrac <$> pcPosition
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> pcRotation
    r                             = realToFrac     pcRestitution
    yP                            = realToFrac     pcYPos

addCube :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBodyID -> PhysicsConfig -> m RigidBody
addCube (DynamicsWorld dynamicsWorld) (RigidBodyID rigidBodyID) PhysicsConfig{..} = liftIO $ 
  RigidBody <$> [C.block| void * {

    btDiscreteDynamicsWorld* dynamicsWorld = ( btDiscreteDynamicsWorld* ) $( void *dynamicsWorld );

    // Create a box
    btVector3 s = btVector3( $( float sx ) , $( float sy ) , $( float sz ) );
    btCollisionShape* collider = new btBoxShape( s );

    // MotionStates are for communicating transforms between our engine and Bullet; 
    // we're not using them
    // yet so we just use the btDefaultMotionState to set the initial object pose
    btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
    btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );

    btDefaultMotionState* motionState = new btDefaultMotionState( btTransform( q , p ) );

    // Set the initial mass, inertia and restitiution
    btScalar mass     = $( float m );
    btVector3 inertia = btVector3( $( float ix ) , $( float iy ) , $( float iz ) );

    collider -> calculateLocalInertia( mass , inertia );

    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI( mass , motionState , collider , inertia );
    btRigidBody* rigidBody = new btRigidBody( rigidBodyCI );

    rigidBody         -> setRestitution( $(float r) );

    // Attach the given RigidBodyID
    rigidBody         -> setUserIndex( $(int rigidBodyID) );

    dynamicsWorld     -> addRigidBody( rigidBody );

    return rigidBody;

    } |]
  where
    (V3 x y z)                    = realToFrac <$> pcPosition
    (V3 sx sy sz)                 = realToFrac <$> pcScale
    (V3 ix iy iz)                 = realToFrac <$> pcInertia
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> pcRotation
    r                             = realToFrac     pcRestitution
    m                             = realToFrac     pcMass

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

  }|]
  where
    (V3 x y z) = realToFrac <$> force



  

stepSimulation :: MonadIO m => DynamicsWorld -> m ()
stepSimulation (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {

    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation(1 / 60.f, 10);

  } |]

getRigidBodyID :: MonadIO m => RigidBody -> m RigidBodyID
getRigidBodyID (RigidBody rigidBody) = liftIO $ 
  RigidBodyID <$> [C.block| int {
    btRigidBody* rigidBody = (btRigidBody *)$(void *rigidBody);
    int rigidBodyID = rigidBody->getUserIndex();
    return rigidBodyID;
    }|]

data Collision = Collision
  { cbBodyA :: !RigidBody
  , cbBodyB :: !RigidBody
  , cbBodyAID :: !RigidBodyID
  , cbBodyBID :: !RigidBodyID
  , cbPositionOnA :: !(V3 Float)
  , cbPositionOnB :: !(V3 Float)
  , cbNormalOnB   :: !(V3 Float)
  } deriving Show

getCollisions :: MonadIO m => DynamicsWorld -> m [Collision]
getCollisions (DynamicsWorld dynamicsWorld) = liftIO $ do

  collisionsRef <- newIORef []
  let captureCollision objA objB objAID objBID aX aY aZ bX bY bZ nX nY nZ = do
        let collision = Collision
              { cbBodyA = RigidBody objA
              , cbBodyB = RigidBody objB
              , cbBodyAID = RigidBodyID objAID
              , cbBodyBID = RigidBodyID objBID
              , cbPositionOnA = V3 (realToFrac aX) (realToFrac aY) (realToFrac aZ)
              , cbPositionOnB = V3 (realToFrac bX) (realToFrac bY) (realToFrac bZ)
              , cbNormalOnB   = V3 (realToFrac nX) (realToFrac nY) (realToFrac nZ)
              }
        modifyIORef collisionsRef (collision:)

  [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    
    int numCollisions = 0;

    int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {

      btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();

      // We only return one contact for now. I believe there are up to 4.
      if (numContacts > 0) {
        btManifoldPoint& pt = contactManifold->getContactPoint(0);
        
        if (pt.getDistance()<0.f) {

          const btCollisionObject* obA = contactManifold->getBody0();
          const btCollisionObject* obB = contactManifold->getBody1();
          
          const btVector3& ptA         = pt.getPositionWorldOnA();
          const btVector3& ptB         = pt.getPositionWorldOnB();
          const btVector3& nmB         = pt.m_normalWorldOnB;

          $fun:(void (*captureCollision)(void*, void*, int, int, float, float, float, float, float, float, float, float, float))(
            (void *)obA,
            (void *)obB,
            obA->getUserIndex(),
            obB->getUserIndex(),
            ptA.getX(),
            ptA.getY(),
            ptA.getZ(),
            ptB.getX(),
            ptB.getY(),
            ptB.getZ(),
            nmB.getX(),
            nmB.getY(),
            nmB.getZ()
            );
        }
      } 
    }


    } |]

  readIORef collisionsRef

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


setRigidBodyKinematic :: MonadIO m => RigidBody -> m ()
setRigidBodyKinematic (RigidBody rigidBody)  = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  rigidBody->setActivationState(DISABLE_DEACTIVATION);
  }|]

setRigidBodyWorldTransform :: (Real a, MonadIO m) => RigidBody -> V3 a -> Quaternion a -> m ()
setRigidBodyWorldTransform (RigidBody rigidBody) position rotation = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);

  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );
  rigidBody -> getMotionState() -> setWorldTransform( btTransform(q , p) );

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation


-----------------
-- Springs
-- See http://bulletphysics.org/Bullet/BulletFull/classbtGeneric6DofSpring2Constraint.html
-----------------

-- | Adds a constraint between the body and (I think) its current position in the world
addWorldSpringConstraint (DynamicsWorld dynamicsWorld) (RigidBody rigidBody) = SpringConstraint <$> liftIO [C.block| void* {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );

  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);

  btQuaternion q     = btQuaternion( btVector3(0,1,0), 0 );
  btVector3    p     = btVector3( 0,0,0 );
  btTransform  frame = btTransform( q , p );

  btGeneric6DofSpring2Constraint *spring = new btGeneric6DofSpring2Constraint(*rigidBody, frame, RO_XYZ);

  dynamicsWorld->addConstraint(spring);

  return spring;

  }|]

addSpringConstraint (DynamicsWorld dynamicsWorld) (RigidBody rigidBodyA) (RigidBody rigidBodyB) = SpringConstraint <$> liftIO [C.block| void * {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );
  btRigidBody* rigidBodyA = (btRigidBody *) $(void *rigidBodyA);
  btRigidBody* rigidBodyB = (btRigidBody *) $(void *rigidBodyB);

  btQuaternion q     = btQuaternion( btVector3(0,1,0), 0 );
  btVector3    p     = btVector3( 0,0,0 );
  btTransform  frame = btTransform( q , p );

  RotateOrder rotOrder = RO_XYZ;
  btGeneric6DofSpring2Constraint *spring = new btGeneric6DofSpring2Constraint(
      *rigidBodyA, *rigidBodyB,
      frame, frame,
      RO_XYZ
      );
  
  dynamicsWorld->addConstraint(spring);

  return spring;
  }|]

setSpringLinearLowerLimit (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setLinearLowerLimit(btVector3( $(float x), $(float y), $(float z)));

  }|]

setSpringLinearUpperLimit (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setLinearUpperLimit(btVector3($(float x), $(float y), $(float z)));

  }|]

setSpringAngularLowerLimit (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setAngularLowerLimit(btVector3($(float x), $(float y), $(float z)));

  }|]

setSpringAngularUpperLimit (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setAngularUpperLimit(btVector3($(float x), $(float y), $(float z)));

  }|]

setSpringLinearBounce (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setBounce(0, $(float x));
  spring->setBounce(1, $(float y));
  spring->setBounce(2, $(float z));

  }|]

setSpringAngularBounce (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setBounce(3, $(float x));
  spring->setBounce(4, $(float y));
  spring->setBounce(5, $(float z));

  }|]

setSpringLinearStiffness (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setStiffness(0, $(float x));
  spring->setStiffness(1, $(float y));
  spring->setStiffness(2, $(float z));

  }|]

setSpringAngularStiffness (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setStiffness(3, $(float x));
  spring->setStiffness(4, $(float y));
  spring->setStiffness(5, $(float z));

  }|]

setSpringLinearDamping (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setDamping(0, $(float x));
  spring->setDamping(1, $(float y));
  spring->setDamping(2, $(float z));

  }|]

setSpringAngularDamping (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setDamping(3, $(float x));
  spring->setDamping(4, $(float y));
  spring->setDamping(5, $(float z));

  }|]

setSpringLinearEquilibrium (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setEquilibriumPoint(0, $(float x));
  spring->setEquilibriumPoint(1, $(float y));
  spring->setEquilibriumPoint(2, $(float z));

  }|]

setSpringAngularEquilibrium (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setEquilibriumPoint(3, $(float x));
  spring->setEquilibriumPoint(4, $(float y));
  spring->setEquilibriumPoint(5, $(float z));

  }|]

setLinearSpringEnabled :: MonadIO m => SpringConstraint -> V3 Bool -> m ()
setLinearSpringEnabled (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->enableSpring(0, $(int x'));
  spring->enableSpring(1, $(int y'));
  spring->enableSpring(2, $(int z'));

  }|]
  where V3 x' y' z' = fromIntegral . fromEnum <$> V3 x y z

setAngularSpringEnabled :: MonadIO m => SpringConstraint -> V3 Bool -> m ()
setAngularSpringEnabled (SpringConstraint springConstraint) (V3 x y z) = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->enableSpring(3, $(int x'));
  spring->enableSpring(4, $(int y'));
  spring->enableSpring(5, $(int z'));

  }|]
  where V3 x' y' z' = fromIntegral . fromEnum <$> V3 x y z
