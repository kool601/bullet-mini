
#include <iostream>

#include <btBulletDynamicsCommon.h>

extern "C" {
void * inline_c_0_baa1bdeeda083f153617fa23f06c41781c48e73f() {

    btBroadphaseInterface* broadphase = new btDbvtBroadphase();

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    return dynamicsWorld;

}

}

extern "C" {
void * inline_c_1_725c7dcdbe6ee42882f9ba7cd1d0fb0bd65b8374(void * dynamicsWorld_inline_c_0) {
 
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;

    // Create a ground plane
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(groundRigidBody);

    // Create a box
    btCollisionShape* fallShape = new btBoxShape(btVector3(0.5,0.5,0.5));

    btDefaultMotionState* fallMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 1, 1, 0.5), btVector3(0, 20, 0)));
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);
    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
    fallRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(fallRigidBody);
    return fallRigidBody;

}

}

extern "C" {
void inline_c_2_7c002c2c3b163803fc70c275387e913f405ceb7c(void * dynamicsWorld_inline_c_0) {

        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;
        dynamicsWorld->stepSimulation(1 / 60.f, 10);
    
}

}

extern "C" {
float * inline_c_3_6f20a07f25aab076c779b670d99fcf30f9c2dfbe(void * rigidBody_inline_c_0) {


        btRigidBody* rigidBody = (btRigidBody *)rigidBody_inline_c_0;

        btTransform trans;
        rigidBody->getMotionState()->getWorldTransform(trans);

        btScalar *transformPtr = (btScalar *)malloc(sizeof(btScalar) * 7);
        transformPtr[0] = trans.getOrigin().getX();
        transformPtr[1] = trans.getOrigin().getY();
        transformPtr[2] = trans.getOrigin().getZ();
        transformPtr[3] = trans.getRotation().getX();
        transformPtr[4] = trans.getRotation().getY();
        transformPtr[5] = trans.getRotation().getZ();
        transformPtr[6] = trans.getRotation().getW();

        return transformPtr;
    
}

}

extern "C" {
void inline_c_4_737b39d24e32b073f211cd97cb15ea0b50b132f8(void * dynamicsWorld_inline_c_0) {

    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;
    //delete dynamicsWorld;
    //delete solver;
    //delete collisionConfiguration;
    //delete dispatcher;
    //delete broadphase;

}

}
