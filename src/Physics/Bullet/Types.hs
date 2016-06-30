{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.Types where

import Foreign.C
import Foreign.Ptr
import Linear.Extra
import Data.Binary
import GHC.Generics

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

newtype CollisionObjectID = CollisionObjectID
    { unCollisionObjectID   :: Word32 }
    deriving (Eq, Show, Ord, Binary, Num, Enum, Real, Integral)

newtype DynamicsWorld = DynamicsWorld
    { unDynamicsWorld :: Ptr () }
    deriving Show
newtype SpringConstraint = SpringConstraint
    { unSpringConstraint :: Ptr () }
    deriving Show
newtype PointToPointConstraint = PointToPointConstraint
    { unPointToPointConstraint :: Ptr () }
    deriving Show
newtype CollisionShape = CollisionShape
    { unCollisionShape :: Ptr () }
    deriving Show
newtype CollisionObject = CollisionObject
    { unCollisionObject :: Ptr () }
    deriving Show
newtype RigidBody = RigidBody
    { unRigidBody :: CollisionObject }
    deriving Show
newtype GhostObject = GhostObject
    { unGhostObject :: CollisionObject }
    deriving Show

data RigidBodyConfig = RigidBodyConfig
    { rbRestitution     :: !Float
    , rbPosition        :: !(V3 Float)
    , rbRotation        :: !(Quaternion Float)
    , rbInertia         :: !(V3 Float)
    , rbMass            :: !Float
    , rbLinearDamping   :: !Float
    , rbAngularDamping  :: !Float
    , rbFriction        :: !Float
    , rbRollingFriction :: !Float
    , rbCollisionGroup  :: !CShort
    , rbCollisionMask   :: !CShort
    }


instance Monoid RigidBodyConfig where
    mempty = RigidBodyConfig
          { rbPosition        = V3 0 0 0
          , rbInertia         = V3 0 0 0
          , rbRotation        = axisAngle ( V3 1 0 0 ) 0
          , rbMass            = 1
          , rbRestitution     = 0.5
          , rbFriction        = 0.5
          , rbRollingFriction = 0
          , rbCollisionGroup  = 1
          , rbCollisionMask   = 1
          , rbLinearDamping   = 0
          , rbAngularDamping  = 0
          }
    mappend _ b = b


data DynamicsWorldConfig = DynamicsWorldConfig
    { dwGravity :: Float
    }
-- We're just implementing this for mempty;
-- the mappend instance isn't useful as it just
-- takes the rightmost DynamicsWorldConfig.
instance Monoid DynamicsWorldConfig where
    mempty = DynamicsWorldConfig
          { dwGravity = -9.8
          }
    mappend _ b = b

-- I've switched from carrying the pointers in this structure
-- so it can be serialized across the network.
-- We could also split it into 2 pieces,
-- since it's nice to query the rigidbody directly.
data Collision = Collision
    -- { cbBodyA          :: !RigidBody
    -- , cbBodyB          :: !RigidBody
    { cbBodyAID        :: !CollisionObjectID
    , cbBodyBID        :: !CollisionObjectID
    , cbPositionOnA    :: !(V3 Float)
    , cbPositionOnB    :: !(V3 Float)
    , cbNormalOnB      :: !(V3 Float)
    , cbAppliedImpulse :: !Float
    } deriving (Show, Generic)
instance Binary Collision

class ToCollisionObjectPointer a where
    toCollisionObjectPointer :: a -> Ptr ()

instance ToCollisionObjectPointer CollisionObject where
    toCollisionObjectPointer = unCollisionObject

instance ToCollisionObjectPointer RigidBody where
    toCollisionObjectPointer = toCollisionObjectPointer . unRigidBody

instance ToCollisionObjectPointer GhostObject where
    toCollisionObjectPointer = toCollisionObjectPointer . unGhostObject

