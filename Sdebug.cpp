#include <stdio.h>
#include "ode/ode.h"

/* debug.cpp. Try basic operations again
 */
class SCoPEx {
  public:
    SCoPEx();
    void Init();
    void Loop();
    void Step();
    void Log();
    void printForces(const char *when);
    void printdRN(const dReal *d, int N);
    int tcount;
    double nextCmdTime;
    // dReal Pressure; // hPa
    // dReal Temperature; // K
    // dReal R; // Pa m^3 mol-1 K-1
    // dReal air_molar_mass; // g/mol
    // dReal rho_air; // Kg/m^3
    // dBodyID balloonID;
    // dReal balloonCd;
    // dReal balloonMass; // Kg
    // dReal balloonRadius; // m
    // dReal balloonAltitude; // m
    // dReal balloonArea; // m^2
    dReal boxAngle;
    dReal boxVelocityAngle;
    dReal boxSpeed;
    dReal thrust_left;
    dReal thrust_right;
    dWorldID world;  // a dynamic world
    FILE *ofp;
    bool run;

    dBodyID payloadID;
    dReal payloadMass; // Kg
    dReal payloadSize[3]; // m
    dReal payloadArea; // X x Z, assumes motion in Y direction only
    dReal payloadCd; // assumes motion in Y direction only
    dReal stepSize;
    static constexpr dReal GRAVITY = -9.81;
};

SCoPEx Model;

SCoPEx::SCoPEx() {
  // Pressure = 50; // hPa
  // Temperature = 212; // K
  // R = 8.314; // Pa m^3 mol-1 K-1
  // air_molar_mass = 28.97; // g/mol
  // rho_air = air_molar_mass*Pressure*0.1/(R*Temperature); // Kg/m^3
  // balloonCd = 0.5;
  // balloonMass = 477.0; // Kg
  // balloonRadius = 14.58495; // m
  // balloonAltitude = 20000.0; // m
  // balloonArea = 3.141592653589793 * balloonRadius * balloonRadius; // m^2
  payloadMass = 444.0; // Kg
  payloadSize[0] = 1; // m
  payloadSize[1] = 1;
  payloadSize[2] = 1;
  payloadArea = payloadSize[0]*payloadSize[3]; // X x Z, assumes motion in Y direction only
  payloadCd = 1.05; // assumes motion in Y direction only
  // tetherMass = 10; // Kg
  // tetherRadius = 0.02;
  // tetherLength = 3*balloonRadius;  // length
  // thrust = 4.;
  // thrustIncrement = 0.25;
  // direction = 90.; // +Y [-180, 180]
  // directionIncrement = 5; // degrees
  // PGain = 0.4/90;
  // IGain = 0;
  // DGain = 0;
  stepSize = 0.05;
  // prevAngleError = 0;
  ofp = fopen("Sdebug.log", "w"); // stdout;
  tcount = -1;
  // opt_graphics = false;
  // opt_commandfile = 0;
  // opt_logfile = 0;
  // cmdfile = 0;
  nextCmdTime = 0;
  // velocityAngleIntegral = 0;
  // velocityAngleIntegralLimit = 45; //*< degrees
  run = true;
}

void SCoPEx::Init() {
  dInitODE();              // Initialize ODE
  world = dWorldCreate();  // Create a world
  dWorldSetGravity(world,0,0,GRAVITY);

  dMass m;                 // mass parameter
  dMassSetZero (&m);  //set mass parameter to zero

  // Box
  payloadID = dBodyCreate (world);
  dMassSetBoxTotal (&m,payloadMass,payloadSize[0],payloadSize[1],payloadSize[2]);
  dBodySetMass (payloadID,&m);
  dReal payloadAltitude = 1000;
  dBodySetPosition (payloadID,0,0,payloadAltitude);
  
  // tetherPayload = dJointCreateBall(world,0);
  // dJointAttach(tetherPayload, tetherID, payloadID);
  // dJointSetBallAnchor(tetherPayload,0,0,balloonAltitude-balloonRadius-tetherLength);
  // dJointEnable(tetherPayload);
}

void SCoPEx::printdRN(const dReal *d, int N) {
  int i;
  if (ofp) {
    for (i = 0; i < N; ++i) {
      fprintf(ofp, ",%7.3lf", (double)d[i]);
    }
  }
}

void SCoPEx::printForces(const char *when) {
  const dReal *gondolaTorque = dBodyGetTorque(payloadID);
  fprintf(ofp, "%s: tau", when);
  printdRN(gondolaTorque,3);
}

void SCoPEx::Step() {
  if (tcount >= 0)
    dWorldQuickStep(world,stepSize);

  // buoyancy
  // fprintf(ofp, "Adding anti-gravity %.4lf\n", -payloadMass*GRAVITY);
  dBodyAddForce(payloadID, 0, 0, -(payloadMass)*GRAVITY);
  // thrust
  // fprintf(ofp, "Adding thrust\n");
  dBodyAddForce(payloadID, 0, 1, 0); // at CG
  dBodyAddForceAtRelPos(payloadID, 0, -.1, 0, 0, 0, payloadSize[2]/2);

  ++tcount;
  if (tcount*stepSize > 30) {
    run = false;
  }
}

/* Current log format
  D = load('debug.log');
  T = D(:,1);
  Di = 2;
  gondolaPos = D(:,Di:Di+2); Di = Di + 3;
  gondolaVel = D(:,Di:Di+2); Di = Di + 3;
  gondolaForce = D(:,Di:Di+2); Di = Di + 3;
  gondolaTorque = D(:,Di:Di+2); Di = Di + 3;
  rotM = D(:,Di:Di+11); Di = Di + 12;
 */
void SCoPEx::Log() {
  const dReal *pos3 = dBodyGetPosition(payloadID);
  
  fprintf(ofp, "%7.2lf", tcount*stepSize);
  printdRN(pos3, 3);
  const dReal *vel = dBodyGetLinearVel(payloadID);
  printdRN(vel, 3);
  vel = dBodyGetForce(payloadID);
  printdRN(vel, 3);
  vel = dBodyGetTorque(payloadID);
  printdRN(vel, 3);
  const dReal *rotM = dBodyGetRotation(payloadID);
  printdRN(rotM, 12);
  fprintf(ofp, "\n");
}

void SCoPEx::Loop() {
  while (run) {
    Step();
    Log();
  }
}

int main(int argc, char **argv) {
  Model.Init();
  Model.Loop();
  //Model.Close();
  return 0;
}