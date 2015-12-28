{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.Types where

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


data PhysicsConfig = PhysicsConfig
  { pcRestitution    :: Float
  , pcPosition       :: V3 Float
  , pcRotation       :: Quaternion Float
  , pcScale          :: V3 Float
  , pcInertia        :: V3 Float
  , pcMass           :: Float
  , pcYPos           :: Float
  , pcCollisionGroup :: CShort
  , pcCollisionMask  :: CShort
  }


instance Monoid PhysicsConfig where
  mempty = PhysicsConfig 
        { pcPosition    = V3 0 0 0
        , pcScale       = V3 1 1 1
        , pcInertia     = V3 0 0 0
        , pcRotation    = axisAngle ( V3 1 0 0 ) 0
        , pcMass        = 1
        , pcRestitution = 0.5
        , pcYPos        = 0
        , pcCollisionGroup = 1
        , pcCollisionMask  = 1
        }
  mappend _ b = b


data PhysicsWorldConfig = PhysicsWorldConfig
  { gravity :: Float
  }
-- We're just implementing this for mempty;
-- the mappend instance isn't useful as it just takes the rightmost
-- PhysicsWorldConfig.
instance Monoid PhysicsWorldConfig where
  mempty = PhysicsWorldConfig 
        { gravity = -9.8
        }
  mappend _ b = b

newtype DynamicsWorld    = DynamicsWorld { unDynamicsWorld :: Ptr () }
newtype RigidBody        = RigidBody     { unRigidBody     :: Ptr () } deriving Show
newtype RigidBodyID      = RigidBodyID   { unRigidBodyID   :: Word32 } 
  deriving (Eq, Show, Ord, Binary, Num, Enum, Real, Integral)

newtype SpringConstraint = SpringConstraint { unSpringConstraint :: Ptr () } deriving Show


-- I've disabled carrying the pointers in this structure so it can be serialized across the network.
-- We could also split it into 2 pieces, since it's probably often nice to query to the rigidbody directly.
data Collision = Collision
  -- { cbBodyA          :: !RigidBody
  -- , cbBodyB          :: !RigidBody
  { cbBodyAID        :: !RigidBodyID
  , cbBodyBID        :: !RigidBodyID
  , cbPositionOnA    :: !(V3 Float)
  , cbPositionOnB    :: !(V3 Float)
  , cbNormalOnB      :: !(V3 Float)
  , cbAppliedImpulse :: !Float
  } deriving (Show, Generic)
instance Binary Collision
