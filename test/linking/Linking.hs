{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Control.Monad
import Control.Monad.State

import Data.Maybe
import Data.Map (Map)
import qualified Data.Map as Map
import Control.Lens
import Physics.Bullet
import Control.Concurrent
import Linear

type CubeID = Int
data Cube = Cube
  { _cubBody :: RigidBody
  }
makeLenses ''Cube

data World = World
  { _wldCubes :: !(Map CubeID Cube)
  }
makeLenses ''World

newWorld :: World
newWorld = World mempty

main :: IO ()
main = do
  
  dynamicsWorld  <- createDynamicsWorld mempty
  _              <- addGroundPlane dynamicsWorld (RigidBodyID 0) 0
    
  void . flip runStateT newWorld $ do 
    forM_ [1..1000] $ \i -> do
      rigidBody <- addCube dynamicsWorld (RigidBodyID i) mempty 
        { pcPosition = V3 0 20 0
        , pcRotation = Quaternion 0.5 (V3 0 1 1)
        }
      wldCubes . at (fromIntegral i) ?= Cube
        { _cubBody = rigidBody
        }
    forever $ do
      
      stepSimulation dynamicsWorld 90

      cubes <- Map.elems <$> use wldCubes
      forM_ cubes $ \cube -> do
        (position, orientation) <- getBodyState (cube ^. cubBody)
        liftIO $ print (position, orientation)
      liftIO $ threadDelay (floor $ 1/60 * 1000000)

