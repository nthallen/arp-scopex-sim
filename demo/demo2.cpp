/* ODE tutorial  by Kosei Demura */
/* Lesson 4 3D Graphics          */
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

dReal balloonMass = 477.0; // Kg
dReal balloonRadius = 14.58495; // m
dReal balloonAltitude = 20000.0; // m
dReal payloadMass = 444.0; // Kg
dReal payloadSize[3] = {1,1,1}; // m
dReal tetherMass = 10; // Kg
dReal tetherRadius = 0.02;
dReal tetherLength = 3*balloonRadius;  // length
dReal thrust = 0.;
dReal thrustIncrement = 1.0;
dReal impulse = 20.;
dReal impulseIncrement = 20.0;
int push = 0;
dReal stepSize = 0.05;
#define GRAVITY (-9.81)

struct MyObject {
  dBodyID body;		// a rigid body
};

dJointID tetherPayload;
dJointFeedback JFB_tetherPayload;

static dWorldID world;  // a dynamic world
static MyObject sphere, box, cylinder; // objects
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
      1.2*(2*balloonRadius+tetherLength),
      3,
      balloonAltitude-balloonRadius-tetherLength}; // view point [m]
  static float hpr[3] = {-180, 20.5, 0}; // view direction[°]
  dsSetViewpoint (xyz,hpr);           // set a view point and direction
  puts("Controls:");
  puts("   f - increase thrust");
  puts("   F - decrease thrust");
  puts("   i - increase impulse magnitude");
  puts("   I - decrease impulse magnitude");
  puts("   p - push forward with current impulse");
  puts("   P - push backward with current impulse");
  puts("   Z - zero thrust");
  ofp = fopen("demo1.log", "w");
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

// Simulation loop
static void simLoop (int pause)
{
  const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;
  static int tcount = 0;

  dBodyAddForce(sphere.body, 0, 0, (balloonMass-(balloonMass+tetherMass+payloadMass))*GRAVITY);
  dBodyAddForce(cylinder.body, 0, 0, tetherMass*GRAVITY);
  dBodyAddForce(box.body, 0, thrust, payloadMass*GRAVITY);
  if (push > 0) {
    dBodyAddForce(box.body, 0, impulse/stepSize, 0);
    --push;
    printf("Impulse thrust: %.1lf\n", impulse/stepSize);
  } else if (push < 0) {
    dBodyAddForce(box.body, 0, -impulse/stepSize, 0);
    ++push;
    printf("Impulse thrust: %.1lf\n", -impulse/stepSize);
  }
  // const dReal *force3, *torque3;
  // force3 = dBodyGetForce(box.body);
  // torque3 = dBodyGetTorque(box.body);
  // fprintf(ofp, "%7d,%7.3lf,%7.3lf,%7.3lf,%7.3lf,%7.3lf,%7.3lf\n",
    // tcount++,
    // force3[0],force3[1],force3[2],
    // torque3[0],torque3[1],torque3[2]);
  dWorldStep(world,stepSize);
  fprintf(ofp, "%7d", tcount++);
  printd3(ofp, JFB_tetherPayload.f1);
  printd3(ofp, JFB_tetherPayload.t1);
  printd3(ofp, JFB_tetherPayload.f2);
  printd3(ofp, JFB_tetherPayload.t2);
  fprintf(ofp, "\n");

  // draw a sphere
  dsSetColor(0.9,0.9,0.9);      // set red color
  dsSetSphereQuality(3);  // set quality of spere. 3 is pretty good
  pos1 = dBodyGetPosition(sphere.body); // get a position
  R1   = dBodyGetRotation(sphere.body); // get an orientation
  dsDrawSphere(pos1,R1,balloonRadius);         // draw a sphere

  // draw tether
  dsSetColorAlpha (0,1,0,1);
  pos2 = dBodyGetPosition(cylinder.body);
  R2   = dBodyGetRotation(cylinder.body);
  dsDrawCylinder(pos2,R2,tetherLength,tetherRadius);

  // draw a box
  dsSetColorAlpha (0,0,1,1);
  pos3 = dBodyGetPosition(box.body);
  R3   = dBodyGetRotation(box.body);
  dsDrawBox(pos3,R3,payloadSize);
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

  // sphere
  sphere.body = dBodyCreate (world);     //  create a body
  dMassSetSphericalShell(&m,balloonMass,balloonRadius); // Calcurate mass parameter
  dBodySetMass (sphere.body,&m);  // Set mass parameter to the body
  //dBodySetPosition (sphere.body,0,1, balloonRadius + tetherLength + 1); // Set a position
  dBodySetPosition (sphere.body,0,0, balloonAltitude); // Set a position

  // Cylinder
  cylinder.body = dBodyCreate (world);
  dMassSetCylinderTotal(&m,tetherMass,3,tetherRadius,tetherLength);
  dBodySetMass (cylinder.body,&m);
  dBodySetPosition (cylinder.body,0,0,balloonAltitude-balloonRadius-tetherLength/2);
  
  dJointID balloonTether = dJointCreateBall(world,0);
  dJointAttach(balloonTether, sphere.body, cylinder.body);
  dJointSetBallAnchor(balloonTether,0,0,balloonAltitude-balloonRadius);
  dJointEnable(balloonTether);

  // Box
  box.body = dBodyCreate (world);
  dMassSetBoxTotal (&m,payloadMass,payloadSize[0],payloadSize[1],payloadSize[2]);
  dBodySetMass (box.body,&m);
  dBodySetPosition (box.body,0,0,balloonAltitude-balloonRadius-tetherLength-payloadSize[2]/2);
  
  tetherPayload = dJointCreateBall(world,0);
  dJointAttach(tetherPayload, cylinder.body, box.body);
  dJointSetBallAnchor(tetherPayload,0,0,balloonAltitude-balloonRadius-tetherLength);
  dJointEnable(tetherPayload);

  dJointSetFeedback(tetherPayload, &JFB_tetherPayload);

  // Simulation loop
  dsSimulationLoop (argc,argv,960,480,&fn);

  dWorldDestroy (world); // destroy the world
  dCloseODE();              // close ODE
  return 0;
}