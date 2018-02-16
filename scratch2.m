%%
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
% Copied from scopex.cpp
D = load('scopex.log');
stepSize = 0.05;
T = D(:,1)*stepSize;
Di = 2;
balloonPos = D(:,Di:Di+2); Di = Di + 3;
tetherPos = D(:,Di:Di+2); Di = Di + 3;
gondolaPos = D(:,Di:Di+2); Di = Di + 3;
gondolaAngle = D(:,Di); Di = Di + 1;
gondolaVelocityAngleSetpoint = D(:,Di); Di = Di + 1;
thrust = D(:,Di:Di+1); Di = Di + 2;
gondolaVelocityAngle = D(:,Di); Di = Di + 1;
gondolaSpeed = D(:,Di); Di = Di + 1;
rotM = D(:,Di:Di+11); Di = Di + 12;
gondolaVel = D(:,Di:Di+2); Di = Di + 3;
gondolaForce = D(:,Di:Di+2); Di = Di + 3;
gondolaTorque = D(:,Di:Di+2); Di = Di + 3;
%%
nsubplotst(T,gondolaVel,'gondolaVel');
nsubplotst(T,gondolaPos,'gondolaPos');
%%
nsubplotst(T,gondolaForce,'gondolaForce');
nsubplotst(T,rotM(:,1:3),'gondolaRot');
%%
nsubplotst(T,gondolaTorque,'gondolaTorque');
