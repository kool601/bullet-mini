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

import Foreign.C
import Foreign.Ptr
import Linear.Extra
import Control.Monad.Trans
import Data.Monoid
import Data.IORef
import Physics.Bullet.Types

{-
See 

http://bulletphysics.org/Bullet/BulletFull/

https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf

-}

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"



createDynamicsWorld :: (Functor m, MonadIO m) =>  DynamicsWorldConfig -> m DynamicsWorld 
createDynamicsWorld DynamicsWorldConfig{..} = DynamicsWorld <$> liftIO [C.block| void * {

  btBroadphaseInterface *broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
    dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0,  $( float g ), 0));

  return dynamicsWorld;

  } |]
  where
    g = realToFrac dwGravity

stepSimulation :: MonadIO m => DynamicsWorld -> Float -> m ()
stepSimulation (DynamicsWorld dynamicsWorld) (realToFrac -> frameRate) = liftIO [C.block| void {
  
    btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
    dynamicsWorld->stepSimulation(1 / $(float frameRate), 10);
  
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

rayTestClosest :: (Fractional a, Real a, MonadIO m) => DynamicsWorld -> Ray a -> m (Maybe (RayResult a))
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
    where (V3 fx fy fz) = realToFrac <$> rayFrom ray
          (V3 tx ty tz) = realToFrac <$> rayTo ray

-- I'm guessing I can get the solver/collisionConfig/dispatcher/broadphase from pointers in the dynamicsWorld
destroyDynamicsWorld :: DynamicsWorld -> IO ()
destroyDynamicsWorld (DynamicsWorld dynamicsWorld) = [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void *dynamicsWorld);
  //delete dynamicsWorld;
  //delete solver;
  //delete collisionConfiguration;
  //delete dispatcher;
  //delete broadphase;
} |]


