%%
clear all
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
%%
r = k*maxDiameter; % semicircle radius
r1 = maxDiameter/2 - r; % rectangle half-width
% square of distance from bottom of triangle to center of semicircle
d12 = (maxHeight-r).^2 + r1.^2;
d1 = sqrt(d12);
d2 = sqrt(d12 - r.^2);
theta = atan(r1/(maxHeight-r))+asin(r/d1);
d3 = d2*cos(theta);
hc = maxHeight - r;
h = linspace(0,maxHeight,101);
v1 = h <= d3;
w = zeros(size(h));
w(v1) = h(v1)*tan(theta);
v2 = h >= d3;
h1 = h - maxHeight + r;
w(v2) = sqrt(r^2 - h1(v2).^2)+r1;
%%
figure;
plot(h,w);
set(gca,'dataaspectratio',[1 1 1]);
%%
figure;
plot([w fliplr(-w)]', [h fliplr(h)]');
set(gca,'dataaspectratio',[1 1 1]);
title(sprintf('r = %.1f', r));
%%
dh = mean(diff(h));
wb = (w(1:end-1)+w(2:end))/2;
ab = pi * wb.^2;
vol = sum(ab)*dh;
%%
ks = linspace(0.1,0.5,10);
volumes = zeros(size(ks));
for i = 1:length(ks)
  k = ks(i);
  [volumes(i),h,w] = balloonVolume(maxHeight,maxDiameter, k);
  fprintf('k = %.2f Volume = %.1f\n', k, volumes(i));
  plot(w,h);
  hold on;
end
hold off;
kopt = interp1(volumes,ks,maxVolume);
title(sprintf('k_{opt} = %.3f', kopt));
%%
[volume,h,w, area] = balloonVolume(maxHeight,maxDiameter, kopt);
clf;
plot(w,h);
set(gca,'dataaspectratio',[1 1 1]);
title(sprintf('Volume = %.1f m^3 Area = %.1f m^2', volume, area));
Df = maxDiameter/maxVolume^(1/3);
Hf = maxHeight/maxDiameter;
Af = area / (maxHeight * maxDiameter);
fprintf(1,'Diameter = %.4f * Volume^(1/3)\n', Df);
fprintf(1,'Height   = %.4f * Diameter\n', Hf);
fprintf(1, 'Area     = %.4f * Height * Diameter\n', Af);

