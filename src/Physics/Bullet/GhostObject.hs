
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
import Foreign
import Foreign.C
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

withArray_ :: (Storable a) => Int -> (Ptr a -> IO ()) -> IO [a]
withArray_ size action = allocaArray size $ \ptr -> do
  _ <- action ptr
  peekArray size ptr

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
        collisionObject->setUserIndex($(int collisionObjectID));

        collisionObject->setCollisionShape(collider);

        collisionObject->setCollisionFlags(collisionObject->getCollisionFlags() |
            btGhostObject::CF_NO_CONTACT_RESPONSE);
      
        dynamicsWorld->addCollisionObject(
            collisionObject, 
            $(short int rbCollisionGroup), 
            $(short int rbCollisionMask));

        
        return collisionObject;
      
        } |]
    where
      (V3 x y z)                    = realToFrac <$> rbPosition
      (Quaternion qw (V3 qx qy qz)) = realToFrac <$> rbRotation


getGhostObjectNumOverlapping :: (Num a, MonadIO m) => GhostObject -> m a
getGhostObjectNumOverlapping (toCollisionObjectPointer -> ghostObject) = liftIO $ 
    fromIntegral <$> [C.block| int {
        btGhostObject *ghostObject = (btGhostObject *)$(void *ghostObject);
        return ghostObject->getNumOverlappingObjects();
    }|]

getGhostObjectOverlapping :: (MonadIO m) => GhostObject -> m [CollisionObject]
getGhostObjectOverlapping ghost@(toCollisionObjectPointer -> ghostObject) = liftIO $ do
    count <- getGhostObjectNumOverlapping ghost
    
    results <- withArray_ (fromIntegral count) $ \ptr -> do
        [C.block| void {
            btGhostObject *ghostObject = (btGhostObject *)$(void *ghostObject);
            btCollisionObject **out = (btCollisionObject **)$(void **ptr);
            for (int i = 0; i < $(int count); i++) {
                out[i] = ghostObject->getOverlappingObject(i);
            }
        }|]
    return $ map CollisionObject results
