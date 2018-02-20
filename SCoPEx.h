#ifndef SCOPEX_H_INCLUDED
#define SCOPEX_H_INCLUDED
#include "ode/ode.h"
#include "commandfile.h"

class SCoPEx {
  public:
    SCoPEx();
    ~SCoPEx();
    void Init(int argc, char **argv);
    void Loop();
    void Step();
    void Log();
    void Draw();
    void Close();
    static void graphicsStart();
    static void graphicsCommand(int c);
  private:
    void LogBody(dBodyID b);
    void LogJoint(dJointFeedback *j);
    void printdR3(const dReal *d);
    void printdRN(const dReal *d, int N);
    void printTorque(const char *when, const dReal *torque);
    void dBodyAddDrag(dBodyID ID, dReal Cd, dReal Area);
    dReal angleDiff(dReal a1, dReal a2);
    int tcount;
    double nextCmdTime;
    dReal Pressure; // hPa
    dReal Temperature; // K
    dReal R; // Pa m^3 mol-1 K-1
    dReal air_molar_mass; // g/mol
    dReal rho_air; // Kg/m^3
    dBodyID balloonID;
    dReal balloonCd;
    dReal balloonMass; // Kg
    dReal balloonRadius; // m
    dReal balloonAltitude; // m
    dReal balloonArea; // m^2
    dReal boxAngle;
    dReal boxVelocityAngle;
    dReal boxSpeed;
    dReal thrust_left;
    dReal thrust_right;
    
    bool opt_graphics;
    const char *opt_commandfile;
    commandFile *cmdfile;
    const char *opt_logfile;
    bool run;

    dWorldID world;  // a dynamic world
    FILE *ofp;

    dBodyID payloadID;
    dReal payloadMass; // Kg
    dReal payloadSize[3]; // m
    dReal payloadArea; // X x Z, assumes motion in Y direction only
    dReal payloadCd; // assumes motion in Y direction only
    
    // These are variable for the direction control
    // direction, which is commanded, specifies the desired velocity direction
    // The setpoint for gondola angle control
    dReal gondolaAngleSetpoint;
    dReal velocityAngleIntegral;
    dReal velocityAngleIntegralLimit;
    dReal prevAngleError;
    dVector3 prevPayloadPos;

    dBodyID tetherID;
    dReal tetherMass; // Kg
    dReal tetherRadius;
    dReal tetherLength;  // length

    dJointID balloonTether;
    dJointID tetherPayload;
    dJointFeedback tetherPayloadFB;
    
    dReal thrust;
    dReal thrustIncrement;
    dReal direction; // +Y [-180, 180]
    dReal directionIncrement; // degrees
    dReal PGain;
    dReal IGain;
    dReal DGain;
    dReal stepSize;
    static constexpr dReal GRAVITY = -9.81;
};

#endif
