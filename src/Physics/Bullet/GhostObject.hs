
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.GhostObject where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

addGhostObject :: (Functor m, MonadIO m) => DynamicsWorld -> CollisionObjectID -> CollisionShape -> RigidBodyConfig -> m GhostObject
addGhostObject (DynamicsWorld dynamicsWorld) (fromIntegral -> collisionObjectID) (CollisionShape collisionShape) RigidBodyConfig{..} = liftIO $ 
    GhostObject . CollisionObject <$> [C.block| void * {
      
        btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *) $(void *dynamicsWorld);
        btCollisionShape        *collider      = (btCollisionShape *) $(void *collisionShape);
      
        btCollisionObject *collisionObject = new btGhostObject();

        btQuaternion q = btQuaternion($(float qx), $(float qy), $(float qz), $(float qw));
        btVector3    p = btVector3($(float x), $(float y), $(float z));

        collisionObject->setWorldTransform(btTransform(q, p));
      
        // Attach the given RigidBodyID
        collisionObject         -> setUserIndex($(int collisionObjectID));
      
        dynamicsWorld           -> addCollisionObject(
            collisionObject, 
            $(short int rbCollisionGroup), 
            $(short int rbCollisionMask));
        
        return collisionObject;
      
        } |]
    where
      (V3 x y z)                    = realToFrac <$> rbPosition
      (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rbRotation



