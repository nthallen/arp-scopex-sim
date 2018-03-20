%%
% Shape estimation
% Rodger's BAL002 describes a shape that is not very explicit and the
% approximation formulas do not agree with BLR004
% But also, BLR004 does not estimate the cross-sectional area
% Based on the drawing, I will try to approximate the cross-sectional area
% as consiting of a central rectangle with two semicircles on each side and
% a triangular base. The triangle would actually intersect the circles at
% the tangent point. The radius for the semicircles is unknown, but appears
% to be approximately 1/3 of the diameter.
% I will try to determine how the radius affects the calculated volume.
maxVolume = 8839.1; % m^3
maxDiameter = 27.184; % m
maxHeight = 25.721; % m
k = 1/3;
r = k*maxDiameter; % semicircle radius
r1 = maxDiameter/2 - r; % rectangle half-width
% square of distance from bottom of triangle to center of semicircle
d12 = (maxHeight-r).^2 + r1.^2;
d1 = sqrt(d12);
d2 = sqrt(d12 + r.^2);
theta = atan(r1/(maxHeight-r))+asin(r/d1);
d3 = d2*cos(theta);
