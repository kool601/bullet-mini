
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
void * inline_c_1_20b65303aeac0d79f07fdecba2c9cca6fdd79415(void * dynamicsWorld_inline_c_0) {
 
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btCollisionShape* fallShape = new btSphereShape(1);


    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setRestitution(0.5);
    dynamicsWorld->addRigidBody(groundRigidBody);


    btDefaultMotionState* fallMotionState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
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
float inline_c_2_9154efb7b517f088caf41dc21e910dbb483cd9e0(void * dynamicsWorld_inline_c_0, void * fallRigidBody_inline_c_1) {

        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;

        btRigidBody* fallRigidBody = (btRigidBody *)fallRigidBody_inline_c_1;

        dynamicsWorld->stepSimulation(1 / 60.f, 10);

        btTransform trans;
        fallRigidBody->getMotionState()->getWorldTransform(trans);

        std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

        return trans.getOrigin().getY();
    
}

}

extern "C" {
void inline_c_3_737b39d24e32b073f211cd97cb15ea0b50b132f8(void * dynamicsWorld_inline_c_0) {

    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)dynamicsWorld_inline_c_0;
    //delete dynamicsWorld;
    //delete solver;
    //delete collisionConfiguration;
    //delete dispatcher;
    //delete broadphase;

}

}
