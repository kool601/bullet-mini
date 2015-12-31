{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.CollisionObject where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Linear.Extra
import Data.Monoid
import Control.Monad.Trans
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"



getCollisionObjectID :: (ToCollisionObjectPointer a, MonadIO m) => a -> m CollisionObjectID
getCollisionObjectID (toCollisionObjectPointer -> collisionObject) = liftIO $ 
  fromIntegral <$> [C.block| int {
    btCollisionObject* collisionObject = (btCollisionObject *)$(void *collisionObject);
    int collisionObjectID = collisionObject->getUserIndex();
    return collisionObjectID;
    }|]

-- | Don't use this for rigid bodies! Use setRigidBodyWorldTransform instead, as that updates the MotionState
setCollisionObjectWorldTransform :: (ToCollisionObjectPointer a, Real b, MonadIO m) => a -> V3 b -> Quaternion b -> m ()
setCollisionObjectWorldTransform (toCollisionObjectPointer -> collisionObject) position rotation = liftIO [C.block| void {
  btCollisionObject *collisionObject     = (btCollisionObject *) $(void *collisionObject);

  btQuaternion q = btQuaternion( $( float qx ) , $( float qy ), $( float qz ), $( float qw ) );
  btVector3    p = btVector3( $( float x ) , $( float y ) , $( float z ) );
  collisionObject->setWorldTransform( btTransform(q , p) );

  }|]
  where
    (V3 x y z)                    = realToFrac <$> position
    (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rotation
