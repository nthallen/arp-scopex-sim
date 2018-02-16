%%
  D = load('Sdebug.log');
  T = D(:,1);
  Di = 2;
  gondolaPos = D(:,Di:Di+2); Di = Di + 3;
  gondolaVel = D(:,Di:Di+2); Di = Di + 3;
  gondolaForce = D(:,Di:Di+2); Di = Di + 3;
  gondolaTorque = D(:,Di:Di+2); Di = Di + 3;
  rotM = D(:,Di:Di+11); Di = Di + 12;
%%
nsubplotst(T,gondolaPos,'Position');
nsubplotst(T,gondolaVel,'Velocity');
%%
nsubplotst(T,gondolaForce,'Force');
nsubplotst(T,gondolaTorque,'Torque');
