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
  IGain = 0;
  DGain = 0;
  stepSize = 0.05;
  prevAngleError = 0;
  ofp = 0;
  tcount = 0;
  opt_graphics = false;
  opt_commandfile = 0;
  opt_logfile = 0;
  cmdfile = 0;
  nextCmdTime = 0;
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
      1.2*(2*Model.balloonRadius+Model.tetherLength),
      3,
      Model.balloonAltitude-Model.balloonRadius-Model.tetherLength}; // view point [m]
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

void SCoPEx::printd3(FILE *ofp, dVector3 d3) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d3[0], (double)d3[1], (double)d3[2]);
}

void SCoPEx::printdR3(FILE *ofp, const dReal *d) {
  fprintf(ofp, ",%7.3lf,%7.3lf,%7.3lf", (double)d[0], (double)d[1], (double)d[2]);
}

void SCoPEx::print_rot(const char *label, const dReal *rot) {
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

void SCoPEx::print_vel(const char *label, const dReal *v) {
  fprintf(ofp, "%s = [%8.5lf, %8.5lf, %8.5lf]\n", label, (double)v[0], (double)v[1], (double)v[2]);
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

void SCoPEx::Step() {
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
  if (direction > 180) {
    double nrevs = ceil(direction/360);
    direction = direction - 360*nrevs;
  } else if (direction < -180) {
    double nrevs = ceil(-direction/360);
    direction = direction + 360*nrevs;
  }
  
  // buoyancy
  dBodyAddForce(balloonID, 0, 0, -(balloonMass+tetherMass+payloadMass)*GRAVITY);
  // Drag
  dBodyAddDrag(balloonID, balloonCd, balloonArea);
  dBodyAddDrag(payloadID, payloadCd, payloadArea);
  
  // Thrust
  // dBodyAddForce(payloadID, 0, thrust, 0);
  const dReal *rot = dBodyGetRotation(payloadID);
  boxAngle = atan2(rot[1*4+1],rot[0*4+1])*180/3.141592653589793;
  dReal angleError = boxAngle - direction;
  if (angleError > 180) angleError -= 360;
  else if (angleError < -180) angleError += 360;
  dReal errorChange = angleError - prevAngleError;
  
  dReal dThrust = angleError * PGain + errorChange * DGain;
  
  if (dThrust > 1) dThrust = 1;
  else if (dThrust < -1) dThrust = -1;
  thrust_left = thrust * (1+dThrust) / 2;
  thrust_right = thrust * (1-dThrust) / 2;
  
  dBodyAddRelForceAtRelPos(payloadID, 0, thrust_left, 0,
    -payloadSize[0]/2, -payloadSize[1]/2, payloadSize[2]/2);
  dBodyAddRelForceAtRelPos(payloadID, 0, thrust_right, 0,
    +payloadSize[0]/2, -payloadSize[1]/2, payloadSize[2]/2);

  const dReal *v = dBodyGetLinearVel(payloadID);
  dWorldStep(world,stepSize);
  prevAngleError = angleError;
  ++tcount;
  if (tcount*stepSize > 30 && !opt_commandfile && !opt_graphics) {
    run = false;
  }
}

void SCoPEx::Log() {
  const dReal *pos1,*pos2,*pos3;
  if (opt_logfile) {
    pos1 = dBodyGetPosition(balloonID); // get a position
    pos2 = dBodyGetPosition(tetherID);
    pos3 = dBodyGetPosition(payloadID);
    
    fprintf(ofp, "%7d", tcount);
    printdR3(ofp, pos1);
    printdR3(ofp, pos2);
    printdR3(ofp, pos3);
    fprintf(ofp, ",%7.4lf,%7.4lf,%7.4lf,%7.4lf",
      (double)boxAngle, (double)direction,
      (double)thrust_left, (double) thrust_right);
    fprintf(ofp, "\n");
    // print_rot("tether_rot", rot);
    // const dReal *v = dBodyGetLinearVel(payloadID);
    // print_vel("payload_vel", v);
  }
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
    cmdfile->addVariable(&IGain, "IGain");
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
  dBodySetPosition (payloadID,0,0,balloonAltitude-balloonRadius-tetherLength-payloadSize[2]/2);
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