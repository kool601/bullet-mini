{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.DynamicsWorld where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Foreign.Ptr
-- import Foreign.StablePtr
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
-- import Control.Applicative
import Data.IORef
import Data.Binary
import GHC.Generics
import Physics.Bullet.Types
{-
See 

http://bulletphysics.org/Bullet/BulletFull/

https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf

-}

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())



createDynamicsWorld :: (Functor m, MonadIO m) =>  PhysicsWorldConfig -> m DynamicsWorld 
createDynamicsWorld PhysicsWorldConfig{..} = DynamicsWorld <$> liftIO [C.block| void * {

  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(
    dispatcher, broadphase, solver, collisionConfiguration);

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
addStaticPlane (DynamicsWorld dynamicsWorld) (fromIntegral -> rigidBodyID) PhysicsConfig{..} = liftIO $ 
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

    dynamicsWorld     -> addRigidBody( rigidBody, $(short int pcCollisionGroup), $(short int pcCollisionMask) );

    return rigidBody;

    } |] 
  where
    (V3 x y z)                    = realToFrac <$> pcPosition
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> pcRotation
    r                             = realToFrac     pcRestitution
    yP                            = realToFrac     pcYPos

addCube :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBodyID -> PhysicsConfig -> m RigidBody
addCube (DynamicsWorld dynamicsWorld) (fromIntegral -> rigidBodyID) PhysicsConfig{..} = liftIO $ 
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

    dynamicsWorld     -> addRigidBody( 
      rigidBody, 
      $(short int pcCollisionGroup), 
      $(short int pcCollisionMask) );

    return rigidBody;

    } |]
  where
    (V3 x y z)                    = realToFrac <$> pcPosition
    (V3 sx sy sz)                 = realToFrac <$> pcScale * 0.5 -- bullet uses 1/2 extents
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

-- NOTE: We'll need some type safety for RigidBodies, but we've only got cubes at the moment
setCubeScale :: (MonadIO m, Real a) => DynamicsWorld -> RigidBody -> V3 a -> m ()
setCubeScale (DynamicsWorld dynamicsWorld) (RigidBody rigidBody) scale = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );
  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);
  btVector3 scale = btVector3( $(float x) , $(float y) , $(float z) );

  btBoxShape *boxShape = (btBoxShape *)rigidBody->getCollisionShape();

  boxShape->setLocalScaling(scale);
  dynamicsWorld->updateSingleAabb(rigidBody);
  }|]
  where
    (V3 x y z) = realToFrac <$> scale

applyCentralImpulse :: (Functor m, MonadIO m, Real a) => RigidBody -> V3 a -> m ()
applyCentralImpulse (RigidBody rigidBody) force = liftIO [C.block| void {

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
  fromIntegral <$> [C.block| int {
    btRigidBody* rigidBody = (btRigidBody *)$(void *rigidBody);
    int rigidBodyID = rigidBody->getUserIndex();
    return rigidBodyID;
    }|]

getCollisions :: MonadIO m => DynamicsWorld -> m [Collision]
getCollisions (DynamicsWorld dynamicsWorld) = liftIO $ do

  collisionsRef <- newIORef []
  -- let captureCollision objA objB objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
  let captureCollision objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
        let collision = Collision
              -- { cbBodyA = RigidBody objA
              -- , cbBodyB = RigidBody objB
              { cbBodyAID = RigidBodyID (fromIntegral objAID)
              , cbBodyBID = RigidBodyID (fromIntegral objBID)
              , cbPositionOnA    = realToFrac <$> V3 aX aY aZ
              , cbPositionOnB    = realToFrac <$> V3 bX bY bZ
              , cbNormalOnB      = realToFrac <$> V3 nX nY nZ
              , cbAppliedImpulse = realToFrac impulse
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

          btScalar impulse = pt.getAppliedImpulse();

          // fun:(void (*captureCollision)(void*, void*, 
          //                               int, int, 
          //                               float, float, float, 
          //                               float, float, float, 
          //                               float, float, float, 
          //                               float))(
          //   (void *)obA,
          //   (void *)obB,
          $fun:(void (*captureCollision)(int, int, 
                                         float, float, float, 
                                         float, float, float, 
                                         float, float, float, 
                                         float))(
            obA->getUserIndex(),
            obB->getUserIndex(),
            ptA.getX(), ptA.getY(), ptA.getZ(),
            ptB.getX(), ptB.getY(), ptB.getZ(), 
            nmB.getX(), nmB.getY(), nmB.getZ(),
            impulse
            );
        }
      } 
    }


    } |]

  readIORef collisionsRef

data RayResult a = RayResult
  { rrRigidBody :: RigidBody
  , rrLocation  :: V3 a
  , rrNormal    :: V3 a
  }

rayTestClosest :: (Fractional a, Real a, MonadIO m) => DynamicsWorld -> Ray a -> m (Maybe (RayResult a))
rayTestClosest (DynamicsWorld dynamicsWorld) ray = liftIO $ do
  ref <- newIORef Nothing
  let captureRayResult bodyPtr locX locY locZ norX norY norZ = if bodyPtr == nullPtr
        then return ()
        else writeIORef ref $ Just $ RayResult
              { rrRigidBody = RigidBody bodyPtr
              , rrLocation  = realToFrac <$> V3 locX locY locZ
              , rrNormal    = realToFrac <$> V3 norX norY norZ
              }
  [C.block| void {
    btCollisionWorld* world = (btCollisionWorld *)$(void *dynamicsWorld);
    btVector3 from = btVector3($(float fx), $(float fy), $(float fz));
    btVector3 to   = btVector3($(float tx), $(float ty), $(float tz));
    
    btCollisionWorld::ClosestRayResultCallback callback(from, to);
    world->rayTest(from, to, callback);

    btVector3 point  = callback.m_hitPointWorld;
    btVector3 normal = callback.m_hitNormalWorld;
    $fun:(void (*captureRayResult)(void *, float, float, float, 
                                           float, float, float))(
            (void *)callback.m_collisionObject,
            point.getX(),  point.getY(),  point.getZ(),
            normal.getX(), normal.getY(), normal.getZ()
            );
  }|]
  readIORef ref
  where (V3 fx fy fz) = realToFrac <$> rayFrom ray
        (V3 tx ty tz) = realToFrac <$> rayTo ray

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

setRigidBodyActive :: MonadIO m => RigidBody -> m ()
setRigidBodyActive (RigidBody rigidBody)  = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  rigidBody->activate();
  }|]

setRigidBodyKinematic :: MonadIO m => RigidBody -> m ()
setRigidBodyKinematic (RigidBody rigidBody)  = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  // Bullet docs recommend always disabling deactivation for kinematic objects
  rigidBody->setActivationState(DISABLE_DEACTIVATION);
  }|]

-- -- | Allows disabling bullet's "sleep" feature (which is on by default)
-- setRigidBodyAllowDeactivation :: MonadIO m => RigidBody -> Bool -> m ()
-- setRigidBodyAllowDeactivation (RigidBody rigidBody) state = liftIO [C.block| void {
--   btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
--   if ( $(bool state) ) {
--     rigidBody->setActivationState(ENABLE_DEACTIVATION);
--   } else {
--     rigidBody->setActivationState(DISABLE_DEACTIVATION);
--   }
--   }|]

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




-- | Build a cubic room from static planes
addStaticRoom :: (MonadIO m) => DynamicsWorld -> RigidBodyID -> Float -> m ()
addStaticRoom dynamicsWorld bodyID height = do
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 1 0 0 ) ((-pi)/2) , pcYPos = ( height * 0.5 ) }
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 1 0 0 ) (( pi)/2) , pcYPos = ( height * 0.5 ) }
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 0 1 0 ) ((-pi)/2) , pcYPos = ( height * 0.5 ) }
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 0 1 0 ) (( pi)/2) , pcYPos = ( height * 0.5 ) }
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 0 1 0 ) (0 )      , pcYPos = ( height * 0.5 ) }
  _ <- addStaticPlane dynamicsWorld bodyID mempty { pcRotation = axisAngle ( V3 0 1 0 ) (pi)      , pcYPos = ( height * 0.5 ) }
  return ()
