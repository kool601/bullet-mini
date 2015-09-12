{-# LANGUAGE CPP #-}
#ifdef mingw32_HOST_OS
{-# OPTIONS_GHC -F -pgmF strip-ths #-}
#endif

{-# LANGUAGE TemplateHaskell, FlexibleContexts, LambdaCase, DeriveDataTypeable #-}

module Types where

import Graphics.GL.Pal
import Graphics.GL

import Data.Data
import Linear
import Physics.Bullet

import Control.Lens



type ObjectID = Int

data Cube = Cube
  { _cubColor :: V4 GLfloat
  , _cubBody  :: RigidBody
  }
makeLenses ''Cube



data Uniforms = Uniforms
  { uModelViewProjection :: UniformLocation (M44 GLfloat)
  , uInverseModel        :: UniformLocation (M44 GLfloat)
  , uModel               :: UniformLocation (M44 GLfloat)
  , uCamera              :: UniformLocation (V3  GLfloat)
  , uDiffuse             :: UniformLocation (V4  GLfloat)
  } deriving (Data)


