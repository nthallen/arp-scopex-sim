/* SCoPEx Simulation
 * Initial attempt to better separate simulation and visualization
 */
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "SCoPEx.h"
#include "ode/ode.h"
#include "ode/mass.h"
#include "drawstuff/drawstuff.h"
#include "nortlib.h"

SCoPEx Model;

SCoPEx::SCoPEx() {
  Pressure = 50; // hPa
  Temperature = 212; // K
  R = 8.314; // Pa m^3 mol-1 K-1
  air_molar_mass = 28.97; // g/mol
  rho_air = air_molar_mass*Pressure*0.1/(R*Temperature); // Kg/m^3
  balloonCd = 0.5;
  balloonMass = 477.0; // Kg
  balloonRadius = 14.58495; // m
  balloonAltitude = 20000.0; // m
  balloonArea = 3.141592653589793 * balloonRadius * balloonRadius; // m^2
  payloadMass = 444.0; // Kg
  payloadSize[0] = 1; // m
  payloadSize[1] = 1;
  payloadSize[2] = 1;
  payloadArea = payloadSize[0]*payloadSize[3]; // X x Z, assumes motion in Y direction only
  payloadCd = 1.05; // assumes motion in Y direction only
  tetherMass = 10; // Kg
  tetherRadius = 0.02;
  tetherLength = 3*balloonRadius;  // length
  thrust = 4.;
  thrustIncrement = 0.25;
  direction = 90.; // +Y [-180, 180]
  directionIncrement = 5; // degrees
  PGain = 0.4/90;
  VPGain = 0;
  DGain = 0;
  stepSize = 0.05;
  gondolaVelocityAngle = 90;
  gondolaAngle = 90;
  gondolaSpeed = 0;
  prevAngleError = 0;
  ofp = 0;
  tcount = 0;
  opt_graphics = false;
  opt_commandfile = 0;
  opt_logfile = 0;
  cmdfile = 0;
  nextCmdTime = 0;
  velocityAngleCorrLimit = 45; //*< degrees
}

SCoPEx::~SCoPEx() {}

void dMassSetSphericalShell (dMass *m, dReal total_mass, dReal radius) {
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

void SCoPEx::graphicsStart() {
  static float xyz[3] = {
      (float)1.2*(2*Model.balloonRadius+Model.tetherLength),
      3,
      (float)Model.balloonAltitude-Model.balloonRadius-Model.tetherLength}; // view point [m]
  static float hpr[3] = {-180, 20.5, 0}; // view direction[°]
  dsSetViewpoint (xyz,hpr);           // set a view point and direction
  puts("Controls:");
  puts("   f - increase thrust");
  puts("   F - decrease thrust");
  puts("   d - increase direction angle");
  puts("   D - decrease direction angle");
  puts("   Z - zero thrust");
}

static void graphicsStop() {
}

void SCoPEx::graphicsCommand(int c) {
  switch (c) {
    case 'f':
      Model.thrust += Model.thrustIncrement;
      break;
    case 'F':
      Model.thrust -= Model.thrustIncrement;
      break;
    case 'd':
      Model.direction += Model.directionIncrement;
      if (Model.direction > 180) Model.direction -= 360;
      break;
    case 'D':
      Model.direction -= Model.directionIncrement;
      if (Model.direction < -180) Model.direction += 360;
      break;
    case 'z': case 'Z':
      Model.thrust = 0.;
      break;
    default: return;
  }
  printf("Thrust = %.1lf Direction = %.1lf\n",
    (double)Model.thrust, (double)Model.direction);
}

void SCoPEx::printdR3(const dReal *d) {
  printdRN(d,3);
  // fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d[0], (double)d[1], (double)d[2]);
}

void SCoPEx::printdRN(const dReal *d, int N) {
  int i;
  for (i = 0; i < N; ++i) {
    fprintf(ofp, ",%12.8lf", (double)d[i]);
  }
}

void SCoPEx::dBodyAddDrag(dBodyID ID, dReal Cd, dReal Area) {
  const dReal *V = dBodyGetLinearVel(ID);
  double Vs2 = V[0]*V[0] + V[1]*V[1] + V[2]*V[2];
  if (Vs2 > 1e-6) {
    double Vs = sqrt(Vs2);
    double Fds = 0.5*rho_air*Vs2*Cd*Area;
    dBodyAddForce(ID, -V[0]*Fds/Vs, -V[1]*Fds/Vs, -V[0]*Fds/Vs);
  }
}

dReal SCoPEx::angleDiff(dReal a1, dReal a2) {
  dReal diff = fmod(a1-a2+180,360);
  diff += (diff < 0) ? 180 : -180;
  return diff;
}

void SCoPEx::printTorque(const char *when, const dReal *torque) {
  FILE *save_fp = ofp;
  ofp = stdout;
  printf("%s", when);
  printdR3(torque);
  printf("\n");
  ofp = save_fp;
}

void SCoPEx::Step() {
  if (tcount > 0)
    dWorldStep(world,stepSize);

  if (opt_commandfile) {
    double now = tcount * stepSize;
    while (nextCmdTime <= now) {
      double dT = cmdfile->eval();
      if (dT < 0) {
        run = false;
        break;
      }
      nextCmdTime = now+dT;
    }
  }

  const dReal *gondolaTorque = dBodyGetTorque(payloadID);
  // printTorque("After step", gondolaTorque);
  
  direction = angleDiff(direction,0); // Normalize
  
  // buoyancy
  dBodyAddForce(balloonID, 0, 0, -(balloonMass+tetherMass+payloadMass)*GRAVITY);
  // Drag
  dBodyAddDrag(balloonID, balloonCd, balloonArea);
  dBodyAddDrag(payloadID, payloadCd, payloadArea);
  // printTorque("After drag", gondolaTorque);
  
  const dReal *newPayloadPos = dBodyGetPosition(payloadID);
  dVector3 boxVelocity;
  gondolaSpeed = 0;
  for (int i = 0; i < 3; ++i) {
    boxVelocity[i] = (newPayloadPos[i]-prevPayloadPos[i])/stepSize;
    gondolaSpeed += boxVelocity[i]*boxVelocity[i];
    prevPayloadPos[i] = newPayloadPos[i];
  }
  gondolaSpeed = sqrt(gondolaSpeed);

  const dReal *rot = dBodyGetRotation(payloadID);
  gondolaAngle = atan2(rot[1*4+1],rot[0*4+1])*180/3.141592653589793;

  gondolaVelocityAngle = gondolaSpeed > 0.001 ?
    atan2(boxVelocity[1],boxVelocity[0]) * 180/3.141592653589793 :
    gondolaAngle;
  
  
  // The outer control loop. This is configured such that
  // PGain and DGain define the inner loop, which controls the gondola
  // angle (and hence the thrust angle). VPGain controls the outer
  // loop, which feeds back on velocity angle. All three gains
  // should be positive.
  dReal velocityAngleError = angleDiff(gondolaVelocityAngle, direction);
  dReal velocityAngleCorr = velocityAngleError * VPGain;
  if (velocityAngleCorr > velocityAngleCorrLimit)
    velocityAngleCorr = velocityAngleCorrLimit;
  else if (velocityAngleCorr < -velocityAngleCorrLimit)
    velocityAngleCorr = -velocityAngleCorrLimit;
  
  gondolaAngleSetpoint = direction - velocityAngleCorr;
  dReal angleError = angleDiff(gondolaVelocityAngle, gondolaAngleSetpoint);
  dReal errorChange = angleError - prevAngleError;
  prevAngleError = angleError;
  
  dReal dThrust = angleError * PGain + errorChange * DGain;
  
  if (dThrust > 1) dThrust = 1;
  else if (dThrust < -1) dThrust = -1;
  thrust_left = thrust * (1+dThrust) / 2;
  thrust_right = thrust * (1-dThrust) / 2;
  
  // printf("Thrust: %12.8lf %12.8lf\n", thrust_left, thrust_right);
  dBodyAddRelForceAtRelPos(payloadID, 0, thrust_left, 0,
    -payloadSize[0]/2, -payloadSize[1]/2, 0); //payloadSize[2]/2);
  // printTorque("After left thrust", gondolaTorque);
  dBodyAddRelForceAtRelPos(payloadID, 0, thrust_right, 0,
    +payloadSize[0]/2, -payloadSize[1]/2, 0); // payloadSize[2]/2);
  // printTorque("After right thrust", gondolaTorque);

  ++tcount;
  if (tcount*stepSize > 30 && !opt_commandfile && !opt_graphics) {
    run = false;
  }
}

void SCoPEx::LogBody(dBodyID b) {
  const dReal *vec = dBodyGetPosition(b);
  printdRN(vec, 3);
 vec = dBodyGetLinearVel(b);
  printdRN(vec, 3);
  vec = dBodyGetForce(b);
  printdRN(vec, 3);
  vec = dBodyGetTorque(b);
  printdRN(vec, 3);
  vec = dBodyGetRotation(b);
  printdRN(vec, 12);
}

void SCoPEx::LogJoint(dJointFeedback *j) {
  printdRN(j->f1,3);
  printdRN(j->t1,3);
  printdRN(j->f2,3);
  printdRN(j->t2,3);
}

/* Current log format
function [B, Di] = GetBodyData(D, col)
  Di = col;
  B.T = D(:,1);
  B.Pos = D(:,Di:Di+2); Di = Di + 3;
  B.Vel = D(:,Di:Di+2); Di = Di + 3;
  B.Force = D(:,Di:Di+2); Di = Di + 3;
  B.Torque = D(:,Di:Di+2); Di = Di + 3;
  rotM = D(:,Di:Di+11); Di = Di + 12;

  D = load('scopex.log');
  T = D(:,1);
  Di = 2;
  [gondola,Di] = GetBodyData(D,Di);
  [tether,Di] = GetBodyData(D,Di);
  [balloon,Di] = GetBodyData(D,Di);
  [TGjoint,Di] = GetJointData(D,Di);
  [Thrust,Di] = GetSimVar(D,'Thrust',Di);
  [Gangles,Di] = GetSimVar(D,'GondolaAngles',Di);
 */
void SCoPEx::Log() {
  fprintf(ofp, "%7.2lf", tcount*stepSize);
  LogBody(payloadID);
  // LogBody(tetherID);
  // LogBody(balloonID);
  // LogJoint(&tetherPayloadFB);
  fprintf(ofp,",%12.8lf,%12.8lf", thrust_left, thrust_right);
  fprintf(ofp,"%12.8lf,%12.8lf,%12.8lf", gondolaAngle,
          gondolaVelocityAngle, gondolaSpeed);
  fprintf(ofp, "\n");
}

void SCoPEx::Draw() {
  const dReal *pos1, *R1, *pos2, *R2, *pos3, *R3;
  // draw a sphere
  dsSetColor(0.9,0.9,0.9);      // set red color
  dsSetSphereQuality(3);  // set quality of spere. 3 is pretty good
  pos1 = dBodyGetPosition(balloonID); // get a position
  R1   = dBodyGetRotation(balloonID); // get an orientation
  dsDrawSphere(pos1,R1,balloonRadius);         // draw a sphere

  // draw tether
  dsSetColorAlpha (0,1,0,1);
  pos2 = dBodyGetPosition(tetherID);
  R2   = dBodyGetRotation(tetherID);
  dsDrawCylinder(pos2,R2,tetherLength,tetherRadius);

  // draw a box
  dsSetColorAlpha (0,0,1,1);
  pos3 = dBodyGetPosition(payloadID);
  R3   = dBodyGetRotation(payloadID);
  dsDrawBox(pos3,R3,payloadSize);
}

static void graphicsStep(int pause) {
  Model.Step();
  Model.Log();
  Model.Draw();
}

void SCoPEx::Init(int argc, char **argv) {
  int c;
  opterr = 0; /* disable default error message */
  while ((c = getopt(argc, argv, "gl:f:")) != -1) {
    switch (c) {
      case 'g':
        opt_graphics = true;
        break;
      case 'l':
        opt_logfile = optarg;
        break;
      case 'f':
        opt_commandfile = optarg;
        break;
      case '?':
        nl_error(3, "Unrecognized Option -%c", optopt);
    }
  }
  if (opt_logfile) {
    ofp = fopen(opt_logfile, "w");
  }
  if (opt_commandfile) {
    cmdfile = new commandFile(opt_commandfile);
    cmdfile->addVariable(&thrust, "Thrust");
    cmdfile->addVariable(&direction, "Direction");
    cmdfile->addVariable(&PGain, "PGain");
    cmdfile->addVariable(&VPGain, "VPGain");
    cmdfile->addVariable(&DGain, "DGain");
  }
  dInitODE();              // Initialize ODE
  world = dWorldCreate();  // Create a world
  dWorldSetGravity(world,0,0,GRAVITY);

  dMass m;                 // mass parameter
  dMassSetZero (&m);  //set mass parameter to zero

  // sphere
  balloonID = dBodyCreate (world);     //  create a body
  dMassSetSphericalShell(&m,balloonMass,balloonRadius); // Calcurate mass parameter
  dBodySetMass (balloonID,&m);  // Set mass parameter to the body
  //dBodySetPosition (balloonID,0,1, balloonRadius + tetherLength + 1); // Set a position
  dBodySetPosition (balloonID,0,0, balloonAltitude); // Set a position

  // Cylinder
  tetherID = dBodyCreate (world);
  dMassSetCylinderTotal(&m,tetherMass,3,tetherRadius,tetherLength);
  dBodySetMass (tetherID,&m);
  dBodySetPosition (tetherID,0,0,balloonAltitude-balloonRadius-tetherLength/2);
  
  balloonTether = dJointCreateBall(world,0);
  dJointAttach(balloonTether, balloonID, tetherID);
  dJointSetBallAnchor(balloonTether,0,0,balloonAltitude-balloonRadius);
  dJointEnable(balloonTether);

  // Box
  payloadID = dBodyCreate (world);
  dMassSetBoxTotal (&m,payloadMass,payloadSize[0],payloadSize[1],payloadSize[2]);
  dBodySetMass (payloadID,&m);
  dReal payloadAltitude = balloonAltitude-balloonRadius-tetherLength-payloadSize[2]/2;
  dBodySetPosition (payloadID,0,0,payloadAltitude);
  prevPayloadPos[0] = 0;
  prevPayloadPos[1] = 0;
  prevPayloadPos[2] = payloadAltitude;
  dMatrix3 pRot = {
     0, 1, 0, 0,
    -1, 0, 0, 0,
     0, 0, 1, 0 };
  //dBodySetRotation(payloadID, pRot);
  
  tetherPayload = dJointCreateBall(world,0);
  dJointAttach(tetherPayload, tetherID, payloadID);
  dJointSetBallAnchor(tetherPayload,0,0,balloonAltitude-balloonRadius-tetherLength);
  dJointEnable(tetherPayload);
}

void SCoPEx::Close() {
  if (opt_logfile) {
    fclose(ofp);
    ofp = 0;
  }
  dWorldDestroy (world); // destroy the world
  dCloseODE();           // close ODE
}

void SCoPEx::Loop() {
  if (opt_graphics) {
    // Simulation loop
    // set drawstuff
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start   = &SCoPEx::graphicsStart;
    fn.step    = &graphicsStep;
    fn.command = &SCoPEx::graphicsCommand;
    fn.stop    = &graphicsStop;
    fn.path_to_textures = "/home/nort/Exp/SCoPEx/ode/drawstuff/textures";
    dsSimulationLoop (0,0,960,480,&fn);
  } else {
    run = true;
    while (run) {
      Step();
      Log();
    }
  }
}

int main (int argc, char **argv) {
  Model.Init(argc, argv);
  Model.Loop();
  Model.Close();
  return 0;
}