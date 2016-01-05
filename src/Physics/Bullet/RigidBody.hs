{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.RigidBody where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Foreign.ForeignPtr
import Foreign.Marshal.Array
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types
import Physics.Bullet.CollisionShape
import Control.Monad

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"



-- Ground plane should always be infinite!
addGroundPlane :: (Functor m, MonadIO m) => DynamicsWorld -> CollisionObjectID -> Float -> m RigidBody
addGroundPlane dynamicsWorld rigidBodyID height  = do
    collisionShape <- createStaticPlaneShape height
    addRigidBody dynamicsWorld rigidBodyID collisionShape mempty { rbRotation = axisAngle ( V3 1 0 0 ) ((-pi)/2), rbMass = 0 }

-- | Build a cubic room from static planes
addStaticRoom :: (MonadIO m) => DynamicsWorld -> CollisionObjectID -> Float -> m ()
addStaticRoom dynamicsWorld bodyID height = do
    let rotations =
          [ axisAngle (V3 1 0 0) ((-pi)/2)
          , axisAngle (V3 1 0 0) (( pi)/2)
          , axisAngle (V3 0 1 0) ((-pi)/2)
          , axisAngle (V3 0 1 0) (( pi)/2)
          , axisAngle (V3 0 1 0) (0) 
          , axisAngle (V3 0 1 0) (pi)
          ]
    forM_ rotations $ \rotation -> do
        collisionShape <- createStaticPlaneShape height
        addRigidBody dynamicsWorld bodyID collisionShape mempty { rbRotation = rotation, rbMass = 0 }
        
    return ()

addRigidBody :: (Functor m, MonadIO m) => DynamicsWorld -> CollisionObjectID -> CollisionShape -> RigidBodyConfig -> m RigidBody
addRigidBody (DynamicsWorld dynamicsWorld) (fromIntegral -> rigidBodyID) (CollisionShape collisionShape) RigidBodyConfig{..} = liftIO $ 
    RigidBody . CollisionObject <$> [C.block| void * {
      
        btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *) $(void *dynamicsWorld);
        btCollisionShape        *collider      = (btCollisionShape *) $(void *collisionShape);
      
        // MotionStates are for communicating transforms between our engine and Bullet; 
        // we're not using them
        // yet so we just use the btDefaultMotionState to set the initial object pose
        btQuaternion q = btQuaternion($(float qx), $(float qy), $(float qz), $(float qw));
        btVector3    p = btVector3($(float x), $(float y), $(float z));
        
        btDefaultMotionState *motionState = new btDefaultMotionState(btTransform(q, p));
      
        // Set the initial mass, inertia and restitiution
        btScalar mass     = $(float m);
        btVector3 inertia = btVector3($(float ix), $(float iy), $(float iz));
      
        collider->calculateLocalInertia(mass, inertia);
      
        btRigidBody::btRigidBodyConstructionInfo rigidBodyCI( 
            mass, motionState, collider, inertia);
        btRigidBody* rigidBody = new btRigidBody(rigidBodyCI);
      
        rigidBody->setRestitution($(float r));
      
        // Attach the given CollisionObjectID
        rigidBody->setUserIndex($(int rigidBodyID));
      
        dynamicsWorld->addRigidBody( 
            rigidBody, 
            $(short int rbCollisionGroup), 
            $(short int rbCollisionMask) );
        
        return rigidBody;
      
        } |]
    where
      (V3 x y z)                    = realToFrac <$> rbPosition
      (V3 ix iy iz)                 = realToFrac <$> rbInertia
      (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rbRotation
      r                             = realToFrac     rbRestitution
      m                             = realToFrac     rbMass
  
removeRigidBody :: (Functor m, MonadIO m) => DynamicsWorld -> RigidBody -> m ()
removeRigidBody (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> rigidBody) = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $(void *dynamicsWorld);
  btRigidBody*             rigidBody     = (btRigidBody *) $(void *rigidBody);

  dynamicsWorld->removeRigidBody(rigidBody);
  delete rigidBody->getMotionState();
  delete rigidBody;
  
  }|]


setRigidBodyScale :: (MonadIO m, Real a) => DynamicsWorld -> RigidBody -> V3 a -> m ()
setRigidBodyScale (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> rigidBody) scale = liftIO [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );
  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);
  btVector3 scale = btVector3( $(float x) , $(float y) , $(float z) );

  btCollisionShape *shape = (btCollisionShape *)rigidBody->getCollisionShape();

  shape->setLocalScaling(scale);
  dynamicsWorld->updateSingleAabb(rigidBody);
  }|]
  where
    (V3 x y z) = realToFrac <$> scale

applyCentralImpulse :: (Functor m, MonadIO m, Real a) => RigidBody -> V3 a -> m ()
applyCentralImpulse (toCollisionObjectPointer -> rigidBody) force = liftIO [C.block| void {

  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);

  btVector3 force = btVector3( $(float x) , $(float y) , $(float z) );
  rigidBody -> applyCentralImpulse( force );

  }|]
  where
    (V3 x y z) = realToFrac <$> force






getBodyState :: (Fractional a, MonadIO m) => RigidBody -> m (V3 a, Quaternion a)
getBodyState (toCollisionObjectPointer -> rigidBody) = do

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
  
  let position    = realToFrac <$> V3 x y z
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

setRigidBodyActive :: MonadIO m => RigidBody -> m ()
setRigidBodyActive (toCollisionObjectPointer -> rigidBody)  = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  rigidBody->activate();
  }|]

setRigidBodyKinematic :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyKinematic (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() |
                                 btCollisionObject::CF_KINEMATIC_OBJECT);
    // Bullet docs recommend always disabling deactivation for kinematic objects
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
  } else {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() &
                                 ~btCollisionObject::CF_KINEMATIC_OBJECT);
    // Restore the normal activation state (undoes DISABLE_DEACTIVATION)
    rigidBody->activate(true);
  }
  
  }|]

setRigidBodyNoContactResponse :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyNoContactResponse (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() |
                                 btCollisionObject::CF_NO_CONTACT_RESPONSE);
  } else {
    rigidBody->setCollisionFlags(rigidBody->getCollisionFlags() &
                                 ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
  }
  
  }|]

setRigidBodyDisableDeactivation :: MonadIO m => RigidBody -> Bool -> m ()
setRigidBodyDisableDeactivation (toCollisionObjectPointer -> rigidBody) (fromIntegral . fromEnum -> flag) = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);
  bool flag = (bool)$(int flag);

  if (flag) {
    rigidBody->setActivationState(DISABLE_DEACTIVATION);
  } else {
    rigidBody->activate(true);
  }
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
setRigidBodyWorldTransform (toCollisionObjectPointer -> rigidBody) position rotation = liftIO [C.block| void {
  btRigidBody *rigidBody     = (btRigidBody *) $(void *rigidBody);

  btQuaternion q = btQuaternion($(float qx), $(float qy), $(float qz), $(float qw));
  btVector3    p = btVector3($(float x), $(float y), $(float z));
  rigidBody->getMotionState()->setWorldTransform(btTransform(q, p));

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation


