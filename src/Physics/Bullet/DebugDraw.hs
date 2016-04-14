{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.DebugDraw where

import qualified Language.C.Inline.Cpp as C

import Foreign.C
import Control.Monad.Trans
import Data.Monoid
import Physics.Bullet.Types
import Text.RawString.QQ (r)
import Linear.Extra
import Data.IORef

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

C.verbatim [r|

typedef void (*DrawFunctionPtr)(float, float, float, 
                                float, float, float, 
                                float, float, float);

class MiniDebugDraw: public btIDebugDraw {
    int m_debugMode;
    DrawFunctionPtr m_drawFunction;
public:
    MiniDebugDraw(DrawFunctionPtr drawPtr);

    virtual void drawLine(const btVector3& from
                         ,const btVector3& to
                         ,const btVector3& color);


    virtual void   drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,
                                    btScalar distance,int lifeTime,const btVector3& color);

    virtual void   reportErrorWarning(const char* warningString);
  
    virtual void   draw3dText(const btVector3& location,const char* textString);
  
    virtual void   setDebugMode(int debugMode);
  
    virtual int    getDebugMode() const { return m_debugMode; }
};

MiniDebugDraw::MiniDebugDraw(DrawFunctionPtr drawPtr) {
    m_debugMode = DBG_DrawWireframe;
    m_drawFunction = drawPtr;
}

void MiniDebugDraw::drawLine(const btVector3& from
                            ,const btVector3& to
                            ,const btVector3& color){
    printf("About to call drawfunk\n");
    m_drawFunction(from.getX(),  from.getY(),  from.getZ(), 
                 to.getX(),    to.getY(),    to.getZ(),
                 color.getX(), color.getY(), color.getZ());
};

void    MiniDebugDraw::setDebugMode(int debugMode)
{
   m_debugMode = debugMode;
}

void    MiniDebugDraw::draw3dText(const btVector3& location,const char* textString)
{
    printf("Debug draw string %s\n", textString);
}

void    MiniDebugDraw::reportErrorWarning(const char* warningString)
{
   printf(warningString);
}

void    MiniDebugDraw::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
   // printf("Draw contact point...\n");
}
|]

addDebugDrawer (DynamicsWorld dynamicsWorld) = do
    debugLinesRef <- newIORef []
    let drawFunctionFlat x1 y1 z1 x2 y2 z2 r g b = do
            print (x1,y1,z1,x2,y2,z2,r,g,b)
            modifyIORef' debugLinesRef 
                (( realToFrac <$> V3 x1 y1 z1
                 , realToFrac <$> V3 x2 y2 z2
                 , realToFrac <$> V4 r g b 1) :)
    [C.block| void {
        btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);

        DrawFunctionPtr drawFunctionPtr = $fun:(void (*drawFunctionFlat)( 
                                           float, float, float, 
                                           float, float, float, 
                                           float, float, float));
        MiniDebugDraw *miniDebugDraw = new MiniDebugDraw(drawFunctionPtr);
        dynamicsWorld->setDebugDrawer(miniDebugDraw);
    }|]

    return debugLinesRef


debugDrawDynamicsWorld :: (RealFrac a, MonadIO m) => DynamicsWorld
                                                  -> IORef ([(V3 a, V3 a, V4 a)]) 
                                                  -> m [(V3 a, V3 a, V4 a)]
debugDrawDynamicsWorld (DynamicsWorld dynamicsWorld) linesRef = liftIO $ do
    [C.block| void {
        btDiscreteDynamicsWorld *dynamicsWorld = (btDiscreteDynamicsWorld *)$(void *dynamicsWorld);
        dynamicsWorld->debugDrawWorld();
    }|]
    lines <- readIORef linesRef
    writeIORef linesRef []
    return lines