{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.PointToPointConstraint where

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


addWorldPointToPointConstraint :: (Real a, MonadIO m)
                               => DynamicsWorld 
                               -> RigidBody -> V3 a
                               -> m PointToPointConstraint
addWorldPointToPointConstraint (DynamicsWorld dynamicsWorld) 
        (toCollisionObjectPointer->rigidBodyA) (fmap realToFrac -> V3 aX aY aZ) = PointToPointConstraint <$> liftIO [C.block| void * {
    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    btRigidBody *rigidBodyA = (btRigidBody *)$(void *rigidBodyA);
    
    btVector3 pivotInA = btVector3($(float aX), $(float aY), $(float aZ));
    
    btPoint2PointConstraint *point2Point = new btPoint2PointConstraint(
        *rigidBodyA, pivotInA
        );
    
    dynamicsWorld->addConstraint(point2Point);
  
    return point2Point;
    }|]

addPointToPointConstraint :: (Real a, MonadIO m)
                          => DynamicsWorld 
                          -> RigidBody -> V3 a
                          -> RigidBody -> V3 a 
                          -> m PointToPointConstraint
addPointToPointConstraint (DynamicsWorld dynamicsWorld) 
        (toCollisionObjectPointer->rigidBodyA) (fmap realToFrac -> V3 aX aY aZ) 
        (toCollisionObjectPointer->rigidBodyB) (fmap realToFrac -> V3 bX bY bZ) = PointToPointConstraint <$> liftIO [C.block| void * {
    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    btRigidBody *rigidBodyA = (btRigidBody *)$(void *rigidBodyA);
    btRigidBody *rigidBodyB = (btRigidBody *)$(void *rigidBodyB);
    
    btVector3 pivotInA = btVector3($(float aX), $(float aY), $(float aZ));
    btVector3 pivotInB = btVector3($(float bX), $(float bY), $(float bZ));
    
    btPoint2PointConstraint *point2Point = new btPoint2PointConstraint(
        *rigidBodyA, *rigidBodyB,
        pivotInA, pivotInB
        );
    
    dynamicsWorld->addConstraint(point2Point);
  
    return point2Point;
    }|]
