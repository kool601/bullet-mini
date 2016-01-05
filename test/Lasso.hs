{-# LANGUAGE FlexibleContexts, LambdaCase, RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}

import Graphics.UI.GLFW.Pal
import Graphics.GL.Pal
import Graphics.VR.Pal
import Physics.Bullet

import Control.Monad
import Control.Monad.State
import Control.Lens.Extra
import qualified Data.Map as Map
import Data.Map (Map)

import CubeUniforms

import Halive.Utils

type ObjectID = Int

data Cube = Cube
  { _cubColor :: !(V4 GLfloat)
  , _cubBody  :: !RigidBody
  , _cubScale :: !(V3 GLfloat)
  }
makeLenses ''Cube

data World = World
  { _wldPlayer :: !(Pose GLfloat)
  , _wldCubes  :: !(Map ObjectID Cube)
  }
makeLenses ''World

tuneSpring :: MonadIO m => SpringConstraint -> m ()
tuneSpring spring = do
  setSpringLinearLowerLimit   spring ((-5)  :: V3 GLfloat)
  setSpringLinearUpperLimit   spring (5     :: V3 GLfloat)
  setSpringAngularLowerLimit  spring ((-1)  :: V3 GLfloat)
  setSpringAngularUpperLimit  spring (1     :: V3 GLfloat)
  setSpringAngularStiffness   spring (1000   :: V3 GLfloat)
  setSpringLinearStiffness    spring (1000   :: V3 GLfloat)
  setSpringLinearDamping      spring (1   :: V3 GLfloat)
  setSpringAngularDamping     spring (1   :: V3 GLfloat)
  setSpringLinearBounce       spring (1    :: V3 GLfloat)
  setSpringAngularBounce      spring (1     :: V3 GLfloat)
  setSpringLinearEquilibrium  spring (0     :: V3 GLfloat)
  setSpringAngularEquilibrium spring (0     :: V3 GLfloat)
  setLinearSpringEnabled      spring (V3 True True True)
  setAngularSpringEnabled     spring (V3 True True True)

main :: IO ()
main = do
  VRPal{..}    <- reacquire 0 $ initVRPal "Bullet" []

  dynamicsWorld  <- createDynamicsWorld mempty

  boxShape <- createBoxShape (1 :: V3 GLfloat)
  boxShape2 <- createBoxShape (2 :: V3 GLfloat)
  _ <- addGroundPlane dynamicsWorld (CollisionObjectID 0) 0

  cubeProg       <- createShaderProgram "test/shared/cube.vert" "test/shared/cube.frag"
  cubeGeo        <- cubeGeometry (1 :: V3 GLfloat) (V3 1 1 1)
  cubeShape      <- makeShape cubeGeo cubeProg

  let Uniforms{..} = sUniforms cubeShape
  useProgram (sProgram cubeShape)
  -- glEnable GL_DEPTH_TEST
  glEnable GL_BLEND
  glClearColor 0 0 0.1 1

  let initialWorld = World
        { _wldPlayer = Pose (V3 0 20 40) (axisAngle (V3 0 1 0) 0)
        , _wldCubes  = mempty
        }
  void . flip runStateT initialWorld $ do

    

    -- Add a moving cube
    let movingID = 2
    movingRigidBody <- addRigidBody dynamicsWorld (CollisionObjectID movingID) boxShape mempty 
        { rbPosition = V3 0 20 5
        , rbRotation = Quaternion 0 (V3 0 1 0)
        , rbCollisionGroup = 0
        , rbCollisionMask = 0
        }
    wldCubes . at (fromIntegral movingID) ?= Cube
        { _cubBody = movingRigidBody
        , _cubColor = V4 0 1 1 1
        , _cubScale = 1
        }
    setRigidBodyKinematic movingRigidBody True
    setRigidBodyNoContactResponse movingRigidBody True

    let springID = 1
    connectedRigidBody <- addRigidBody dynamicsWorld (CollisionObjectID springID) boxShape2 mempty 
      { rbPosition = V3 0 20 0
      , rbRotation = Quaternion 0 (V3 0 1 0)
      , rbCollisionGroup = 1
      , rbCollisionMask = 1
      }
    wldCubes . at (fromIntegral springID) ?= Cube
      { _cubBody = connectedRigidBody
      , _cubColor = V4 1 0.5 1 0.5
      , _cubScale = 2
      }
    spring <- addSpringConstraint dynamicsWorld movingRigidBody connectedRigidBody
    setRigidBodyDisableDeactivation connectedRigidBody True
    tuneSpring spring

    whileWindow gpWindow $ do
      projMat <- getWindowProjection gpWindow 45 0.1 1000
      viewMat <- viewMatrixFromPose <$> use wldPlayer
      (x,y,w,h) <- getWindowViewport gpWindow
      glViewport x y w h

      glClear (GL_COLOR_BUFFER_BIT .|. GL_DEPTH_BUFFER_BIT)

      -- applyMouseLook gpWindow wldPlayer
      applyWASD gpWindow wldPlayer
      processEvents gpEvents $ \e -> do
        closeOnEscape gpWindow e
        whenMouseDown gpWindow MouseButton'1 $ do
          playerPose <- use wldPlayer
          cursorRay  <- cursorPosToWorldRay gpWindow projMat playerPose
          let newPosition = pointOnRay cursorRay 30

          setRigidBodyWorldTransform movingRigidBody newPosition (axisAngle (V3 0 1 0) 0)

      -- Move the cube up and down
      -- movingY <- (+ 20) . (* 5) . sin . (\x->x::Double) . realToFrac . utctDayTime <$> liftIO getCurrentTime
      -- setRigidBodyWorldTransform movingRigidBody (V3 0 movingY 5) (axisAngle (V3 1 1 0) movingY)

      -- setSpringWorldPose spring (V3 0 y 5) (axisAngle (V3 0 1 0) 0)

      
      stepSimulation dynamicsWorld 90

      -- Render Cubes
      

      uniformV3 uCamera =<< use (wldPlayer . posPosition)

      let viewProj = projMat !*! viewMat

      cubes <- Map.toList <$> use wldCubes
      withVAO (sVAO cubeShape) $ 
        forM_ cubes $ \(_cubeID, cube) -> do
          -- liftIO.print $ cubeID
          (position, orientation) <- getBodyState (cube ^. cubBody)

          let model = mkTransformation orientation position !*! scaleMatrix (cube ^. cubScale)
          uniformM44 uModelViewProjection (viewProj !*! model)
          uniformM44 uInverseModel        (inv44 model)
          uniformM44 uModel               model
          uniformV4  uDiffuse             (cube ^. cubColor)
          glDrawElements GL_TRIANGLES (geoVertCount (sGeometry cubeShape)) GL_UNSIGNED_INT nullPtr

      swapBuffers gpWindow
