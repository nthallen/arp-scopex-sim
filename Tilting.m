function [dF,rotA] = Tilting(L1s, L2s)
if nargin < 2
    L1s = 10;
    L2s = 10.02;
end
if length(L1s) ~= length(L2s)
    error('Input vectors must be of same length');
end
dF = zeros(length(L1s),1);
rotA = zeros(length(L1s),1);
for i=1:length(L1s)
    
%%
% Tilting
% Start with coordinate system where center of the top of
% gondola is at the origin with the center of mass at distance
% d directly below. The Gondola is width w with two ropes
% length L1 and L2 connected at either side and suspended from
% a single point. The assumption is that the whole diagram will need
% to rotate to get the center of mass directly below the suspension
% point, and I want to figure out what the necessary angle of rotation is.

% Let P1 and P2 be the two ends of the top bar of the gondola where
% the ropes of length L1 and L2 connect respectively. Drop a perpendicular
% line from the suspension point S to P1P2 and call the intersection point X.
% Let x be the distance from P1 to X and let h be the length of SX.
% Then x^2 + h^2 = L1^2 and
% (w-x)^2 + h^2 = L2^2 => w^2 - 2wx + x^2 + h^2 = L2^2
% => w^2 - 2wx + L1^2 = L2^2 => x = (w^2 + L1^2 - L2^2)/(2w)
% and h = sqrt(L1^2 - x^2) and the rotation angle is atan(x/(h+d));
L1 = L1s(i);
L2 = L2s(i);
w = 1;
d = 1;
M = 600;
g = 9.8;

x = (w^2 + L1^2 - L2^2)/(2*w);
h = sqrt(L1^2 - x^2);
Arot = atan((x-w/2)/(h+d));
Ac1 = atan(w/(2*d));
Ag1 = pi/2 + Ac1 + Arot;
Ag2 = pi/2 - Ac1 + Arot;
Agrav = -pi/2;
Fgrav = M*g;
As1 = Arot;
As2 = pi+Arot;
A1 = asin(h/L1) + Arot;
A2 = (pi-asin(h/L2)) - Arot;
[Tg1,Tg2] = SolveForces('CoG',Fgrav, Agrav, Ag1, Ag2);
[T1,Ts1] = SolveForces('P1',-Tg1, Ag1, A1, As1);
[T2,Ts2] = SolveForces('P2',-Tg2, Ag2, A2, As2);
CheckForces('S',-T1,A1,-T2,A2,Fgrav,pi/2);

dF(i) = (T1-T2)/g;
rotA(i) = rad2deg(Arot);
end
