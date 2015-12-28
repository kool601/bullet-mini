{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.SpringConstraint where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types

-----------------
-- Springs
-- See http://bulletphysics.org/Bullet/BulletFull/classbtGeneric6DofSpring2Constraint.html
-----------------

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

-- | Adds a constraint between the body and (I think) its current position in the world
addWorldSpringConstraint :: MonadIO m => DynamicsWorld -> RigidBody -> m SpringConstraint
addWorldSpringConstraint (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer->rigidBody) = SpringConstraint <$> liftIO [C.block| void* {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld *) $( void *dynamicsWorld );

  btRigidBody* rigidBody = (btRigidBody *) $(void *rigidBody);

  btQuaternion q     = btQuaternion( btVector3(0,1,0), 0 );
  btVector3    p     = btVector3( 0,0,0 );
  btTransform  frame = btTransform( q , p );

  btGeneric6DofSpring2Constraint *spring = new btGeneric6DofSpring2Constraint(*rigidBody, frame, RO_XYZ);

  dynamicsWorld->addConstraint(spring);

  return spring;

  }|]

addSpringConstraint :: MonadIO m => DynamicsWorld -> RigidBody -> RigidBody -> m SpringConstraint
addSpringConstraint (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer->rigidBodyA) (toCollisionObjectPointer->rigidBodyB) = SpringConstraint <$> liftIO [C.block| void * {
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

setSpringWorldPose :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> Quaternion a -> m ()
setSpringWorldPose (SpringConstraint springConstraint) position rotation = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );
  
  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );
  btTransform  frame = btTransform( q , p );

  spring->setFrames(frame, spring->getFrameOffsetB());

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation

setSpringLinearLowerLimit :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearLowerLimit (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setLinearLowerLimit(btVector3( $(float x), $(float y), $(float z)));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringLinearUpperLimit :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearUpperLimit (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setLinearUpperLimit(btVector3($(float x), $(float y), $(float z)));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringAngularLowerLimit :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularLowerLimit (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setAngularLowerLimit(btVector3($(float x), $(float y), $(float z)));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringAngularUpperLimit :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularUpperLimit (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setAngularUpperLimit(btVector3($(float x), $(float y), $(float z)));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringLinearBounce :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearBounce (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setBounce(0, $(float x));
  spring->setBounce(1, $(float y));
  spring->setBounce(2, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz


setSpringAngularBounce :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularBounce (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setBounce(3, $(float x));
  spring->setBounce(4, $(float y));
  spring->setBounce(5, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringLinearStiffness :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearStiffness (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setStiffness(0, $(float x));
  spring->setStiffness(1, $(float y));
  spring->setStiffness(2, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringAngularStiffness :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularStiffness (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setStiffness(3, $(float x));
  spring->setStiffness(4, $(float y));
  spring->setStiffness(5, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringLinearDamping :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearDamping (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setDamping(0, $(float x));
  spring->setDamping(1, $(float y));
  spring->setDamping(2, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringAngularDamping :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularDamping (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setDamping(3, $(float x));
  spring->setDamping(4, $(float y));
  spring->setDamping(5, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringLinearEquilibrium :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringLinearEquilibrium (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setEquilibriumPoint(0, $(float x));
  spring->setEquilibriumPoint(1, $(float y));
  spring->setEquilibriumPoint(2, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setSpringAngularEquilibrium :: (Real a, MonadIO m) => SpringConstraint -> V3 a -> m ()
setSpringAngularEquilibrium (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->setEquilibriumPoint(3, $(float x));
  spring->setEquilibriumPoint(4, $(float y));
  spring->setEquilibriumPoint(5, $(float z));

  }|]
  where (V3 x y z) = realToFrac <$> xyz

setLinearSpringEnabled :: MonadIO m => SpringConstraint -> V3 Bool -> m ()
setLinearSpringEnabled (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->enableSpring(0, $(int x'));
  spring->enableSpring(1, $(int y'));
  spring->enableSpring(2, $(int z'));

  }|]
  where V3 x' y' z' = fromIntegral . fromEnum <$> xyz

setAngularSpringEnabled :: MonadIO m => SpringConstraint -> V3 Bool -> m ()
setAngularSpringEnabled (SpringConstraint springConstraint) xyz = liftIO [C.block| void {
  btGeneric6DofSpring2Constraint* spring = (btGeneric6DofSpring2Constraint *) $( void *springConstraint );

  spring->enableSpring(3, $(int x'));
  spring->enableSpring(4, $(int y'));
  spring->enableSpring(5, $(int z'));

  }|]
  where V3 x' y' z' = fromIntegral . fromEnum <$> xyz
