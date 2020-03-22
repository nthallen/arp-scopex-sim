function [F2,F3] = SolveForces(name,F1,a1,a2,a3)
M = [sin(a2) sin(a3)
     cos(a2) cos(a3)];
Col = -F1 * [sin(a1);cos(a1)];
F = M\Col;
F2 = F(1);
F3 = F(2);
CheckForces(name,F1,a1,F2,a2,F3,a3);
