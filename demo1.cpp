/* ODE tutorial  by Kosei Demura */
/* Lesson 4 3D Graphics          */
#include <stdio.h>
#include <math.h>
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

dReal Pressure = 50; // hPa
dReal Temperature = 212; // K
dReal R = 8.314; // Pa m^3 mol-1 K-1
dReal air_molar_mass = 28.97; // g/mol
dReal rho_air = air_molar_mass*Pressure*0.1/(R*Temperature); // Kg/m^3
dReal balloonCd = 0.5;
dReal balloonMass = 477.0; // Kg
dReal balloonRadius = 14.58495; // m
dReal balloonAltitude = 20000.0; // m
dReal balloonArea = 3.141592653589793 * balloonRadius * balloonRadius; // m^2
dReal payloadMass = 444.0; // Kg
dReal payloadSize[3] = {1,1,1}; // m
dReal payloadArea = payloadSize[0]*payloadSize[3]; // X x Z, assumes motion in Y direction only
dReal payloadCd = 1.05; // assumes motion in Y direction only
dReal tetherMass = 10; // Kg
dReal tetherRadius = 0.02;
dReal tetherLength = 3*balloonRadius;  // length
dReal thrust = 4.;
dReal thrustIncrement = 0.25;
dReal direction = 90.; // +Y [-180, 180]
dReal directionIncrement = 5; // degrees
dReal angleGain = -0.4/90;
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
  puts("   d - increase direction angle");
  puts("   D - decrease direction angle");
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
    case 'd':
      direction += directionIncrement;
      if (direction > 180) direction -= 360;
      break;
    case 'D':
      direction -= directionIncrement;
      if (direction < -180) direction += 360;
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
  printf("Thrust = %.1lf Direction = %.1lf Impulse = %.1lf\n",
    (double)thrust, (double)direction, (double)impulse);
}

static void printd3(FILE *ofp, dVector3 d3) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d3[0], (double)d3[1], (double)d3[2]);
}

static void printdR3(FILE *ofp, const dReal *d) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d[0], (double)d[1], (double)d[2]);
}

static void print_rot(const char *label, const dReal *rot) {
  int row, col;
  fprintf(ofp, "%s: [", label);
  for (row = 0; row < 3; ++row) {
    fprintf(ofp, "\n  ");
    for (col = 0; col < 4; ++col) {
      fprintf(ofp, " %8.5lf", (double)rot[4*row+col]);
    }
  }
  fprintf(ofp, "]\n");
}

static void print_vel(const char *label, const dReal *v) {
  fprintf(ofp, "%s = [%8.5lf, %8.5lf, %8.5lf]\n", label, (double)v[0], (double)v[1], (double)v[2]);
}

static void dBodyAddDrag(dBodyID ID, dReal Cd, dReal Area) {
  const dReal *V = dBodyGetLinearVel(ID);
  double Vs2 = V[0]*V[0] + V[1]*V[1] + V[2]*V[2];
  if (Vs2 > 1e-6) {
    double Vs = sqrt(Vs2);
    double Fds = 0.5*rho_air*Vs2*Cd*Area;
    dBodyAddForce(ID, -V[0]*Fds/Vs, -V[1]*Fds/Vs, -V[0]*Fds/Vs);
  }
}

// Simulation loop
static void simLoop (int pause)
{
  const dReal *pos1,*R1,*pos2,*R2,*pos3,*R3;
  static int tcount = 0;

  // buoyancy
  dBodyAddForce(sphere.body, 0, 0, -(balloonMass+tetherMass+payloadMass)*GRAVITY);
  // Drag
  dBodyAddDrag(sphere.body, balloonCd, balloonArea);
  dBodyAddDrag(box.body, payloadCd, payloadArea);
  
  // Thrust
  // dBodyAddForce(box.body, 0, thrust, 0);
  const dReal *rot = dBodyGetRotation(box.body);
  dReal boxAngle = atan2(rot[1*4+1],rot[1*4+0])*180/3.141592653589793;
  dReal angleError = boxAngle - direction;
  if (angleError > 180) angleError -= 360;
  else if (angleError < -180) angleError += 360;
  dReal dThrust = angleError * angleGain;
  if (dThrust > 1) dThrust = 1;
  else if (dThrust < -1) dThrust = -1;
  dReal thrust_left = thrust * (1+dThrust) / 2;
  dReal thrust_right = thrust * (1-dThrust) / 2;
  
  dBodyAddRelForceAtRelPos(box.body, 0, thrust_left, 0,
    -payloadSize[0]/2, -payloadSize[1]/2, payloadSize[2]/2);
  dBodyAddRelForceAtRelPos(box.body, 0, thrust_right, 0,
    +payloadSize[0]/2, -payloadSize[1]/2, payloadSize[2]/2);

  const dReal *v = dBodyGetLinearVel(box.body);
  if (push > 0) {
    dBodyAddForce(box.body, 0, impulse/stepSize, 0);
    --push;
    printf("Impulse thrust: %.1lf\n", impulse/stepSize);
  } else if (push < 0) {
    dBodyAddForce(box.body, 0, -impulse/stepSize, 0);
    ++push;
    printf("Impulse thrust: %.1lf\n", -impulse/stepSize);
  }
  dWorldStep(world,stepSize);

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
  
  fprintf(ofp, "%7d", tcount++);
  printdR3(ofp, pos1);
  printdR3(ofp, pos2);
  printdR3(ofp, pos3);
  fprintf(ofp, ",%7.4lf,%7.4lf,%7.4lf,%7.4lf",
    (double)boxAngle, (double)direction,
    (double)thrust_left, (double) thrust_right);
  fprintf(ofp, "\n");
  // print_rot("tether_rot", rot);
  // const dReal *v = dBodyGetLinearVel(box.body);
  // print_vel("payload_vel", v);
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
  dWorldSetGravity(world,0,0,GRAVITY);

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