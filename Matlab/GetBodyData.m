function [B, Di] = GetBodyData(D, col, name)
  Di = col;
  B.name = name;
  B.T = D(:,1);
  B.Pos = D(:,Di:Di+2); Di = Di + 3;
  B.Vel = D(:,Di:Di+2); Di = Di + 3;
  B.Force = D(:,Di:Di+2); Di = Di + 3;
  B.Torque = D(:,Di:Di+2); Di = Di + 3;
  B.rotM = D(:,Di:Di+11); Di = Di + 12;
