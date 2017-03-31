/* ODE tutorial  by Kosei Demura */
/* Lesson 4 3D Graphics          */
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"
#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // Stop VC++ warnings
#endif
#ifdef dDOUBLE
#define dsDrawBox  dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

#define DENSITY (5.0)

struct MyObject {
  dBodyID body;		// a rigid body
};

dReal radius = 0.25; // radius
dReal length = 1.0;  // length
dReal sides[3] = {0.5,0.5,1.0}; // length of edges
static dWorldID world;  // a dynamic world
static MyObject sphere, box, capsule, cylinder; // objects

// start simulation
static void start()
{
  static float xyz[3] = {5,3,0.5};    // view point [m]
  static float hpr[3] = {-180, 0, 0}; // view direction[°]
  dsSetViewpoint (xyz,hpr);           // set a view point and direction
}

// Simulation loop
static void simLoop (int pause)
{
  const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;    // draw a sphere
  dsSetColor(1,0,0);      // set red color
  dsSetSphereQuality(3);  // set quality of spere. 3 is pretty good
  pos1 = dBodyGetPosition(sphere.body); // get a position
  R1   = dBodyGetRotation(sphere.body); // get an orientation
  dsDrawSphere(pos1,R1,radius);         // draw a sphere

  // draw a cylinder
  dsSetColorAlpha (0,1,0,1);
  pos2 = dBodyGetPosition(cylinder.body);
  R2   = dBodyGetRotation(cylinder.body);
  dsDrawCylinder(pos2,R2,length,radius);
  // draw an capsule   dsSetColorAlpha (1,1,1,1);
  pos2 = dBodyGetPosition(capsule.body);
  R2   = dBodyGetRotation(capsule.body);
  dsDrawCapsule(pos2,R2,length,radius);

  // draw a box
  dsSetColorAlpha (0,0,1,1);
  pos3 = dBodyGetPosition(box.body);
  R3   = dBodyGetRotation(box.body);
  dsDrawBox(pos3,R3,sides);           // draw a line
  dReal posA[3] = {0, 5, 0},  posB[3]={0, 5, 1.9};
  dsDrawLine(posA,posB);
}

int main (int argc, char **argv)
{
  // set drawstuff
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = NULL;
  fn.stop    = NULL;
  fn.path_to_textures = "/home/nort/Exp/SCoPEx/ode/drawstuff/textures";

  dInitODE();              // Initialize ODE
  world = dWorldCreate();  // Create a world

  dMass m;                 // mass parameter
  dMassSetZero (&m);  //set mass parameter to zero

  // sphere
  sphere.body = dBodyCreate (world);     //  create a body
  dReal radius = 0.5;       // radius [m]
  dMassSetSphere (&m,DENSITY,radius); // Calcurate mass parameter
  dBodySetMass (sphere.body,&m);  // Set mass parameter to the body
  dBodySetPosition (sphere.body,0,1, 1); // Set a position

  // Box
  box.body = dBodyCreate (world);
  dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
  dBodySetMass (box.body,&m);
  dBodySetPosition (box.body,0,2,1);

  // Capsule
  capsule.body = dBodyCreate (world);
  dMassSetCapsule(&m,DENSITY,3,radius,length);
  dBodySetMass (capsule.body,&m);
  dBodySetPosition (capsule.body,0,4,1);

  // Cylinder
  cylinder.body = dBodyCreate (world);
  dMassSetCylinder(&m,DENSITY,3,radius,length);
  dBodySetMass (cylinder.body,&m);
  dBodySetPosition (cylinder.body,0,3,1);

  // Simulation loop
  dsSimulationLoop (argc,argv,960,480,&fn);

 dWorldDestroy (world); // destroy the world
  dCloseODE();              // close ODE
  return 0;
}