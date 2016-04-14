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
import Data.Vector.Storable
C.context (C.cppCtx <> C.funCtx <> C.vecCtx)

C.include "<btBulletDynamicsCommon.h>"

createStaticPlaneShape :: (MonadIO m, Real a) => a -> m CollisionShape
createStaticPlaneShape planeOffset = liftIO $ CollisionShape <$> [C.block| void * {
    btCollisionShape *shape = new btStaticPlaneShape(btVector3(0, 0, 1) , $(float yP));
    return shape;
    }|]
    where
        yP = realToFrac planeOffset * 0.5 -- bullet uses 1/2 extents

createBoxShape :: (MonadIO m, Fractional a, Real a) => V3 a -> m CollisionShape
createBoxShape size = liftIO $ CollisionShape <$> [C.block| void * {
    btVector3 s = btVector3($(float sx), $(float sy), $(float sz));
    // btCollisionShape *shape = new btBoxShape(btVector3(1,1,1));
    // shape->setLocalScaling(s);
    btCollisionShape *shape = new btBoxShape(s);
    return shape;
    }|]
    where
        V3 sx sy sz = realToFrac <$> size * 0.5 -- bullet uses 1/2 extents

createSphereShape :: (MonadIO m, Fractional a, Real a) => a -> m CollisionShape
createSphereShape (realToFrac -> radius) = liftIO $ CollisionShape <$> [C.block| void * {
    btCollisionShape *shape = new btSphereShape($(float radius));
    return shape;
    }|]

createConvexHullShape :: (MonadIO m) => Vector (V3 Float) -> m CollisionShape
createConvexHullShape points = liftIO $ do
    let pointsFlat = unsafeCast points
    CollisionShape <$> [C.block| void * {

        float* points = $vec-ptr:(float *pointsFlat);
        int numPoints = $vec-len:pointsFlat;

        btConvexHullShape *shape = new btConvexHullShape();
        for (int i = 0; i < numPoints; i+=3) {
            shape->addPoint(btVector3(points[i], points[i+1], points[i+2]));
        }
        return shape;
        }|]