function [Th, Di] = GetSimVar(D, name, col)
  Di = col;
  Th.name = name;
  Th.T = D(:,1);
  Th.(name) = D(:,Di:Di+1); Di = Di + 2;
