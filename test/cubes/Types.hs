{-# LANGUAGE CPP #-}
#ifdef mingw32_HOST_OS
{-# OPTIONS_GHC -F -pgmF strip-ths #-}
#endif

{-# LANGUAGE TemplateHaskell, FlexibleContexts, LambdaCase #-}

module Types where

import Graphics.UI.GLFW.Pal

import Graphics.GL.Pal
import Graphics.GL
import Linear
import Cube

import Control.Monad
import Control.Monad.State
import System.Random
import Control.Lens
import Foreign (nullPtr)
import qualified Data.Map as Map
import Data.Map (Map)
import Control.Monad.Random

type ObjectID = Int

data Object = Object
    { _objPosition    :: V3 GLfloat
    , _objOrientation :: Quaternion GLfloat
    }
data Player = Player
    { _plrPosition    :: V3 GLfloat
    , _plrOrientation :: Quaternion GLfloat
    }
data World = World
    { _wldPlayer :: Player
    , _wldCubes  :: Map ObjectID Object
    }

makeLenses ''World
makeLenses ''Object
makeLenses ''Player