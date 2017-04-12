/* Demo3 Test basic physics   */
#include <stdio.h>
#include "ode/ode.h"
#include "ode/mass.h"
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

dReal payloadMass = 1.0; // Kg
dReal payloadSize[3] = {1,1,1}; // m
dReal thrust = 0.;
dReal thrustIncrement = 1.0;
dReal impulse = 20.;
dReal impulseIncrement = 20.0;
int push = 0;
dReal stepsPerSec = 20;
dReal stepSize = 1.0/stepsPerSec;
#define GRAVITY (-9.81)

struct MyObject {
  dBodyID body;		// a rigid body
};

static dWorldID world;  // a dynamic world
static MyObject box; // objects
static FILE *ofp;

void dMassSetSphericalShell (dMass *m, dReal total_mass, dReal radius)
{
#define _I(i,j) I[(i)*4+(j)]
    dMassSetZero (m);
    m->mass = total_mass;
    dReal II = REAL(2.0/3.0) * total_mass * radius*radius;
    m->_I(0,0) = II;
    m->_I(1,1) = II;
    m->_I(2,2) = II;

# ifndef dNODEBUG
    dMassCheck (m);
# endif
}

// start simulation
static void start() {
  static float xyz[3] = {
      20,
      0,
      3}; // view point [m]
  static float hpr[3] = {145, 0, 0}; // view direction[°]
  dsSetViewpoint (xyz,hpr);           // set a view point and direction
  puts("Controls:");
  puts("   f - increase thrust");
  puts("   F - decrease thrust");
  puts("   i - increase impulse magnitude");
  puts("   I - decrease impulse magnitude");
  puts("   p - push forward with current impulse");
  puts("   P - push backward with current impulse");
  puts("   Z - zero thrust");
  ofp = fopen("demo3.log", "w");
}

static void stop() {
  fclose(ofp);
}

void command(int c) {
  switch (c) {
    case 'f':
      thrust += thrustIncrement;
      break;
    case 'F':
      thrust -= thrustIncrement;
      break;
    case 'i':
      impulse += impulseIncrement;
      break;
    case 'I':
      impulse -= impulseIncrement;
      break;
    case 'p':
      push++;
      break;
    case 'P':
      push--;
      break;
    case 'z': case 'Z':
      thrust = 0.;
      impulse = 0.;
      push = 0;
      break;
    default: return;
  }
  printf("Thrust = %.1lf Impulse = %.1lf\n", (double)thrust, (double)impulse);
}

static void printd3(FILE *ofp, dVector3 d3) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d3[0], (double)d3[1], (double)d3[2]);
}

static void printdR3(FILE *ofp, const dReal *d) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d[0], (double)d[1], (double)d[2]);
}

// Simulation loop
static void simLoop (int pause)
{
  const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;
  static int tcount = 0;
  dReal netThrust = thrust;

  if (push > 0) {
    netThrust += impulse/stepSize;
  } else if (push < 0) {
    netThrust -= impulse/stepSize;
  }
  dBodyAddForce(box.body, 0, netThrust, 0);
  dWorldStep(world,stepSize);

  // draw a box
  dsSetColorAlpha (0,0,1,1);
  pos3 = dBodyGetPosition(box.body);
  R3   = dBodyGetRotation(box.body);
  dsDrawBox(pos3,R3,payloadSize);
  
  fprintf(ofp, "%8.2lf,%7.1lf", tcount++/stepsPerSec, (double)netThrust);
  printdR3(ofp, pos3);
  fprintf(ofp, "\n");
}

int main (int argc, char **argv) {
  // set drawstuff
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start   = &start;
  fn.step    = &simLoop;
  fn.command = &command;
  fn.stop    = &stop;
  fn.path_to_textures = "/home/nort/Exp/SCoPEx/ode/drawstuff/textures";

  dInitODE();              // Initialize ODE
  world = dWorldCreate();  // Create a world
  // dWorldSetGravity(world,0,0,GRAVITY);

  dMass m;                 // mass parameter
  dMassSetZero (&m);  //set mass parameter to zero

  // Box
  box.body = dBodyCreate (world);
  dMassSetBoxTotal (&m,payloadMass,payloadSize[0],payloadSize[1],payloadSize[2]);
  dBodySetMass (box.body,&m);
  dBodySetPosition (box.body,0,0,3);

  // Simulation loop
  dsSimulationLoop (argc,argv,960,480,&fn);

  dWorldDestroy (world); // destroy the world
  dCloseODE();              // close ODE
  return 0;
}