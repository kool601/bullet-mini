{-# LANGUAGE DeriveDataTypeable #-}
module CubeUniforms where

import Graphics.GL.Pal

data Uniforms = Uniforms
  { uModelViewProjection :: UniformLocation (M44 GLfloat)
  , uModel               :: UniformLocation (M44 GLfloat)
  , uCamera              :: UniformLocation (V3  GLfloat)
  , uDiffuse             :: UniformLocation (V4  GLfloat)
  } deriving (Data)

