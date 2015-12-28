{-# LANGUAGE TemplateHaskell, FlexibleContexts, LambdaCase #-}

module Types where

import Physics.Bullet
import Graphics.GL.Pal
import Control.Lens.Extra
import GHC.Word


type ObjectID = Word32

data Cube = Cube
  { _cubColor :: !(V4 GLfloat)
  , _cubBody  :: !RigidBody
  }
makeLenses ''Cube




