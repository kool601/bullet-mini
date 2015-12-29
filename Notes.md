Ghost Objects
=============

GhostObjects are convenient, but we can achieve much the same effect via
```
collisionObject->setCollisionFlags(collisionObject->getCollisionFlags() |
            btCollisionObject::CF_NO_CONTACT_RESPONSE);
```

And checking for collisions as usual.


http://gamedev.stackexchange.com/questions/43490/what-is-the-best-way-to-check-if-there-is-overlap-between-player-and-static-non

Getting more precise GhostObject results via btPairCachingGhostObject
http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=3976

"Collision Callbacks and Triggers"
http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Collision_Callbacks_and_Triggers

Interaction of ghost objects and raycasting:
http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=3578


Grabbing
========
Notes on creating nicely grab-and-pullable objects (see last post):
http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7857&view=next


MotionStates
============
"MotionStates are a way for Bullet to do all the hard work for you getting the objects being simulated into the rendering part of your program."
"Bullet manages body interpolation through MotionStates."
http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=MotionStates

