function [Th, Di] = GetSimVar(D, name, col, ncol)
  Di = col;
  Th.name = name;
  Th.T = D(:,1);
  Th.(name) = D(:,Di:Di+ncol-1); Di = Di + ncol;
