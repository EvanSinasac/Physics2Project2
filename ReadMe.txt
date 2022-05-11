Evan Sinasac - 1081418
INFO6022 Physics & Simulation 2
Project 2

Built and compiled using Visual Studios 2019 in Debug in 64 bit.
gdpPhysics and PhysicsInterface do not compile in Release.

CONTROLS
WASD		- Apply a force on the selected sphere in the direction of the camera orthogonal to the horizontal plane
Arrow Keys	- Camera is facing the origin and the arrow keys rotate the camera around the origin and zoom in/out towards the origin
1/2/3/4/5	- Switch the selected sphere that the user can apply forces to
M		- Switch all spheres to wireframe


PHYSICS
The gdpPhysics and PhysicsInterface projects are the ones we made in class, with their implementations into the main project my own.  This is build directly off of Project 1, with the additions of Rotational Forces and Frictional Forces applied to the spheres in both Sphere-Sphere and Sphere-Plane collisions.  Implementations can be seen in the CollisionHandler.cpp.  After recording the video, I did end up implementing friction (at least how I interpret it should work, not certain if it's right lol) into the Sphere-Sphere collision.  Otherwise, the only other change from the recorded demo is that I changed the speed/force applied to the spheres in InputHandling.cpp to 100 instead of 10 so that they aren't frustratingly slow to move around.


Video: https://youtu.be/x1yrNza20TU