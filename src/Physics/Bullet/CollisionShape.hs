{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.CollisionShape where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Linear.Extra
import Data.Monoid
import Control.Monad.Trans
import Physics.Bullet.Types

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"

createStaticPlaneShape :: (MonadIO m, Real a) => a -> m CollisionShape
createStaticPlaneShape planeOffset = liftIO $ CollisionShape <$> [C.block| void * {
    btCollisionShape *collider = new btStaticPlaneShape(btVector3(0, 0, 1) , $(float yP));
    return collider;
    }|]
    where
        yP = realToFrac planeOffset * 0.5 -- bullet uses 1/2 extents

createBoxShape :: (MonadIO m, Fractional a, Real a) => V3 a -> m CollisionShape
createBoxShape size = liftIO $ CollisionShape <$> [C.block| void * {
    btVector3 s = btVector3($(float sx), $(float sy), $(float sz));
    btCollisionShape *collider = new btBoxShape(s);
    return collider;
    }|]
    where
        V3 sx sy sz = realToFrac <$> size * 0.5 -- bullet uses 1/2 extents

createSphereShape :: (MonadIO m, Fractional a, Real a) => a -> m CollisionShape
createSphereShape (realToFrac -> radius) = liftIO $ CollisionShape <$> [C.block| void * {
    btCollisionShape *collider = new btSphereShape($(float radius));
    return collider;
    }|]
