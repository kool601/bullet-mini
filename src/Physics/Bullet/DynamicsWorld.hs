{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.DynamicsWorld where

import qualified Language.C.Inline.Cpp as C

import Foreign.Ptr
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Data.IORef
import Data.Time
import Physics.Bullet.Types
import Text.RawString.QQ (r)

{-
See

http://bulletphysics.org/Bullet/BulletFull/

https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf

-}

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"


createDynamicsWorld :: (MonadIO m) =>  DynamicsWorldConfig -> m DynamicsWorld
createDynamicsWorld DynamicsWorldConfig{..} = DynamicsWorld <$> liftIO [C.block| void * {

  btBroadphaseInterface *broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
    dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0,  $( float g ), 0));

  // This is required to use btGhostObjects
  dynamicsWorld->getPairCache()->setInternalGhostPairCallback(
    new btGhostPairCallback());

  return dynamicsWorld;

  } |]
  where
    g = realToFrac dwGravity

stepSimulationSimple :: MonadIO m => DynamicsWorld -> NominalDiffTime -> m ()
stepSimulationSimple (DynamicsWorld dynamicsWorld) (realToFrac -> dt) = liftIO [C.block| void {

    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation($(float dt));

    }|]
stepSimulationWithTimestep :: MonadIO m => DynamicsWorld -> NominalDiffTime -> Int -> Float -> m ()
stepSimulationWithTimestep (DynamicsWorld dynamicsWorld) (realToFrac -> dt) (fromIntegral -> maxSubsteps) (realToFrac -> fixedTimeStep) = liftIO [C.block| void {

    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation($(float dt), $(int maxSubsteps), $(float fixedTimeStep));

    }|]

-- | See computeOverlappingPairs. Adding these two so we can detect GhostObject intersections
-- even when the simulation is paused.
performDiscreteCollisionDetection :: MonadIO m => DynamicsWorld -> m ()
performDiscreteCollisionDetection (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {

    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->performDiscreteCollisionDetection();

    }|]

-- | "the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection
-- (or stepSimulation) it can be useful to use if you perform ray tests without collision detection/simulation"
computeOverlappingPairs :: MonadIO m => DynamicsWorld -> m ()
computeOverlappingPairs (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {

    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->computeOverlappingPairs();

    }|]

updateAABBs :: MonadIO m => DynamicsWorld -> m ()
updateAABBs (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {

    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->updateAabbs();

    }|]

getCollisions :: MonadIO m => DynamicsWorld -> m [Collision]
getCollisions (DynamicsWorld dynamicsWorld) = liftIO $ do

    collisionsRef <- newIORef []
    -- let captureCollision objA objB objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
    let captureCollision objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
          let collision = Collision
                -- { cbBodyA = RigidBody objA
                -- , cbBodyB = RigidBody objB
                { cbBodyAID = CollisionObjectID (fromIntegral objAID)
                , cbBodyBID = CollisionObjectID (fromIntegral objBID)
                , cbPositionOnA    = realToFrac <$> V3 aX aY aZ
                , cbPositionOnB    = realToFrac <$> V3 bX bY bZ
                , cbNormalOnB      = realToFrac <$> V3 nX nY nZ
                , cbAppliedImpulse = realToFrac impulse
                }
          modifyIORef collisionsRef (collision:)

    [C.block| void {
        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

        int numCollisions = 0;

        int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
        for (int i = 0; i < numManifolds; i++) {

            btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();

            // We only return one contact for now. I believe there are up to 4.
            if (numContacts > 0) {
                btManifoldPoint& pt = contactManifold->getContactPoint(0);
                if (pt.getDistance()<0.f) {

                    const btCollisionObject* obA = contactManifold->getBody0();
                    const btCollisionObject* obB = contactManifold->getBody1();

                    const btVector3& ptA         = pt.getPositionWorldOnA();
                    const btVector3& ptB         = pt.getPositionWorldOnB();
                    const btVector3& nmB         = pt.m_normalWorldOnB;

                    btScalar impulse = pt.getAppliedImpulse();

                    //printf("Distance: %f Impulse: %f Normal: %f %f %f \n", pt.getDistance(), impulse,
                    //  nmB.getX(), nmB.getY(), nmB.getZ());

                    // fun:(void (*captureCollision)(void*, void*,
                    //                               int, int,
                    //                               float, float, float,
                    //                               float, float, float,
                    //                               float, float, float,
                    //                               float))(
                    //   (void *)obA,
                    //   (void *)obB,
                    $fun:(void (*captureCollision)(int, int,
                                                   float, float, float,
                                                   float, float, float,
                                                   float, float, float,
                                                   float)) (
                        obA->getUserIndex(),
                        obB->getUserIndex(),
                        ptA.getX(), ptA.getY(), ptA.getZ(),
                        ptB.getX(), ptB.getY(), ptB.getZ(),
                        nmB.getX(), nmB.getY(), nmB.getZ(),
                        impulse
                        );
                }
            }
        }
    }|]

    readIORef collisionsRef

data RayResult a = RayResult
  { rrCollisionObject :: CollisionObject
  , rrLocation        :: V3 a
  , rrNormal          :: V3 a
  }

rayTestClosest :: (RealFloat a, MonadIO m) => DynamicsWorld -> Ray a -> m (Maybe (RayResult a))
rayTestClosest (DynamicsWorld dynamicsWorld) ray = liftIO $ do
    ref <- newIORef Nothing
    let captureRayResult bodyPtr locX locY locZ norX norY norZ = if bodyPtr == nullPtr
          then return ()
          else writeIORef ref $ Just $ RayResult
                  { rrCollisionObject = CollisionObject bodyPtr
                  , rrLocation        = realToFrac <$> V3 locX locY locZ
                  , rrNormal          = realToFrac <$> V3 norX norY norZ
                  }
    [C.block| void {
        btCollisionWorld* world = (btCollisionWorld *)$(void *dynamicsWorld);
        btVector3 from = btVector3($(float fx), $(float fy), $(float fz));
        btVector3 to   = btVector3($(float tx), $(float ty), $(float tz));

        btCollisionWorld::ClosestRayResultCallback callback(from, to);
        world->rayTest(from, to, callback);

        btVector3 point  = callback.m_hitPointWorld;
        btVector3 normal = callback.m_hitNormalWorld;
        $fun:(void (*captureRayResult)(void *, float, float, float,
                                               float, float, float)) (
                (void *)callback.m_collisionObject,
                point.getX(),  point.getY(),  point.getZ(),
                normal.getX(), normal.getY(), normal.getZ()
            );
    }|]
    readIORef ref
    where (V3 fx fy fz) = realToFrac <$> rayOrigin ray
          (V3 tx ty tz) = realToFrac <$> projectRay ray 1000



-- I'm guessing I can get the solver/collisionConfig/dispatcher/broadphase from pointers in the dynamicsWorld
destroyDynamicsWorld :: DynamicsWorld -> IO ()
destroyDynamicsWorld (DynamicsWorld dynamicsWorld) = [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);

  //delete solver;
  //delete collisionConfiguration;
  //delete dispatcher;
  //delete broadphase;

  delete dynamicsWorld;
} |]

C.verbatim [r|

typedef void (*CaptureCollisionPtr)(int, int,
                                    float, float, float,
                                    float, float, float,
                                    float, float, float,
                                    float);

struct GatherContactResultsCallback : public btCollisionWorld::ContactResultCallback {

    GatherContactResultsCallback(btCollisionObject* tgtBody , CaptureCollisionPtr context)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody), funPtr(context) { }

    btCollisionObject* body; // The body the sensor is monitoring
    CaptureCollisionPtr funPtr; // External information for contact processing

    //virtual bool needsCollision(btBroadphaseProxy* proxy) const {
    //}

    virtual btScalar addSingleResult(btManifoldPoint& pt,
        const btCollisionObjectWrapper* colObj0,int partId0,int index0,
        const btCollisionObjectWrapper* colObj1,int partId1,int index1)
    {
        const btCollisionObject* obA = colObj0->m_collisionObject;
        const btCollisionObject* obB = colObj1->m_collisionObject;

        const btVector3& ptA         = pt.getPositionWorldOnA();
        const btVector3& ptB         = pt.getPositionWorldOnB();
        const btVector3& nmB         = pt.m_normalWorldOnB;

        btScalar impulse = pt.getAppliedImpulse();

        //printf("Distance: %f Impulse: %f Normal: %f %f %f \n", pt.getDistance(), impulse,
        //  nmB.getX(), nmB.getY(), nmB.getZ());

        funPtr(
            obA->getUserIndex(),
            obB->getUserIndex(),
            ptA.getX(), ptA.getY(), ptA.getZ(),
            ptB.getX(), ptB.getY(), ptB.getZ(),
            nmB.getX(), nmB.getY(), nmB.getZ(),
            impulse
            );

        // do stuff with the collision point


        return 0; // There was a planned purpose for the return value of addSingleResult, but it is not used so you can ignore it.
    }
};

|]

contactTest :: (MonadIO m, ToCollisionObjectPointer a) => DynamicsWorld -> a -> m [Collision]
contactTest (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> collisionObject) = liftIO $ do
  collisionsRef <- newIORef []
  let captureCollision objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
        let collision = Collision
              -- { cbBodyA = RigidBody objA
              -- , cbBodyB = RigidBody objB
              { cbBodyAID = CollisionObjectID (fromIntegral objAID)
              , cbBodyBID = CollisionObjectID (fromIntegral objBID)
              , cbPositionOnA    = realToFrac <$> V3 aX aY aZ
              , cbPositionOnB    = realToFrac <$> V3 bX bY bZ
              , cbNormalOnB      = realToFrac <$> V3 nX nY nZ
              , cbAppliedImpulse = realToFrac impulse
              }
        modifyIORef collisionsRef (collision:)

  [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
    btCollisionObject* collisionObject = (btCollisionObject *)$(void *collisionObject);

    CaptureCollisionPtr captureCollisionPtr = $fun:(void (*captureCollision)(int, int,
                                       float, float, float,
                                       float, float, float,
                                       float, float, float,
                                       float));
    GatherContactResultsCallback callback(collisionObject, captureCollisionPtr);
    dynamicsWorld->contactTest(collisionObject, callback);

  }|]
  readIORef collisionsRef
