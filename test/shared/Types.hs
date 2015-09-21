{-# LANGUAGE TemplateHaskell, FlexibleContexts, LambdaCase #-}

module Types where

import Physics.Bullet
import Graphics.GL.Pal
import Control.Lens.Extra



type ObjectID = Int

data Cube = Cube
  { _cubColor :: !(V4 GLfloat)
  , _cubBody  :: !RigidBody
  }
makeLenses ''Cube




