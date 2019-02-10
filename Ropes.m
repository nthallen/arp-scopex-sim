function Ropes(r1, r2, L, N)
% Ropes(r1, r2, L, N);
% r1 Half distance between upper connections m
% r2 Half distance between lower connections m
% L  Vertical distance between upper and lower connections m
% N  Total number of twisting revolutions (defaults to 2)
%%
cd C:\Users\nort\Documents\Documents\Exp\SCoPEx\scopex-sim
%%
% SCoPEx ascender model
% r1 = 0.5; % Half distance between upper connections m
% r2 = 0.05; % Half distance between lower connections m
% L = 100; % Distance between upper and lower connections m
if nargin < 4
    N = 2;
end
r3 = 0.006; % Rope radius m
M = 600; % Payload mass in Kg
g = 9.8; % m/s^2
theta = [0:.01:1]*pi; % radians
debug = 0;
%%
% Intermediate distance
dp = r1*sin(theta);
% Calculate baseline displacement
d = sqrt(dp.^2 + (r2 - r1 * cos(theta)).^2);
% Angle from vertical
phi = asin(d/L);
% Perdendicular distance from the radial at r2
dp2 = r2*sin(theta);
% Angle from radial
phitau = asin(dp2./d);
% Height
height = sqrt(L.^2 - d.^2);
% Rope Tension
Tr = M*g*L./height;
% Horizontal force
F = M*g*d./height;
% Torque
tau = r1*F.*dp2./d; % r1*F_hor*sin(phitau)
%%
if debug
figure;
ax = [ nsubplot(4,1,1) nsubplot(4,1,2) nsubplot(4,1,3) nsubplot(4,1,4)];
plot(ax(1),theta,tau);
ylabel(ax(1),'Torque');
plot(ax(2),theta,Tr);
ylabel(ax(2),'Rope Tension');
plot(ax(3),theta,d);
ylabel(ax(3),'Baseline');
plot(ax(4),theta,dp2);
ylabel(ax(4),'Perp from Radial');

title(ax(1),sprintf('r_1 = %.2f r_2 = %.2f', r1, r2));
set(ax(2:2:end),'YAxisLocation','right');
set(ax(1:end-1),'XTickLabels',[]);
end
%%
% After the ropes cross
thetatwist = [0:.01:2*N]*pi;
% Baseline offset due to twisting
Htwist = thetatwist*r3;
% Full baseline offset with twist
Btwist = (r1+r2) + Htwist;
% Rope angle from vertical with twist
phitwist = asin(Btwist./L);
% Length of Rope in the twist
Ltwist = Htwist*L./Btwist;
% The length of rope that is twisted per radian
% This is the radius of curvature
rad_of_twist = Ltwist./thetatwist;
% Height
Ht_twist = sqrt(L.^2 - Btwist.^2);
% Rope Tension
Tr_twist = M*g*L./Ht_twist;
%%
if debug
figure;
ax = [ nsubplot(4,1,1) nsubplot(4,1,2) nsubplot(4,1,3) nsubplot(4,1,4) ];
plot(ax(1),thetatwist/(2*pi),Ltwist);
ylabel(ax(1),'Ltwist m');
plot(ax(2),thetatwist/(2*pi),rad2deg(phitwist));
ylabel(ax(2),'\phi_{twist} deg');
plot(ax(3),thetatwist/(2*pi),rad_of_twist);
ylabel(ax(3),'radius m');
plot(ax(4),thetatwist/(2*pi),Tr_twist);
ylabel(ax(4),'Tension N');

title(ax(1),sprintf('r_1 = %.2f r_2 = %.2f', r1, r2));
set(ax(2:2:end),'YAxisLocation','right');
set(ax(1:end-1),'XTickLabels',[]);
end
%%
% Calculate Torque after twisting
F_hz_twist = Tr_twist .* sin(phitwist);
phitau_twist = asin(r3/sqrt(r1^2+r3^2));
tau_twist = F_hz_twist .* r1 .* r3 ./ sqrt(r1^2+r3^2);
% tau_twist = Tr_twist * r1 * r3 .* sin(phitwist) / sqrt(r1^2+r3^2);
% Calculate Horizontal tension in the twist
% Horizontal force per unit length of rope
fpl_twist = Tr_twist./rad_of_twist;
% Total frictional force in the rope
% Calculate friction after twisting
ffr_twist = fpl_twist .* Ltwist;
% Distance from platform to the twist
LtoTwist = r2 * L ./Btwist;
%%
if debug
figure;
ax = [ nsubplot(4,1,1) nsubplot(4,1,2) nsubplot(4,1,3) nsubplot(4,1,4) ];
plot(ax(1),thetatwist/(2*pi),tau_twist);
ylabel(ax(1),'\tau Nm');
plot(ax(2),thetatwist/(2*pi),fpl_twist);
ylabel(ax(2),'N/m');
plot(ax(3),thetatwist/(2*pi),ffr_twist);
ylabel(ax(3),'friction');
plot(ax(4),thetatwist/(2*pi),LtoTwist);
ylabel(ax(4),'L to Twist m');

title(ax(1),sprintf('r_1 = %.2f r_2 = %.2f', r1, r2));
set(ax(2:2:end),'YAxisLocation','right');
set(ax(1:end-1),'XTickLabels',[]);
end
%%
% Locate the point in the first half rotation when the ropes first
% touch. That's where r1*sin(phitau) == r3 for the first time
V = theta > pi/2;
theta_contact = interp1(r1*sin(phitau(V)), theta(V), r3);
%%
% Summary plot
Revs0 = theta/(2*pi);
Revs1 = theta_contact/(2*pi) + thetatwist/(2*pi);
figure;
ax = [ nsubplot(4,1,1) nsubplot(4,1,2) nsubplot(4,1,3) nsubplot(4,1,4) ];
plot(ax(1),Revs0,tau,Revs1,tau_twist);
ylabel(ax(1),'\tau Nm');
plot(ax(2),Revs0,Tr,Revs1,Tr_twist);
ylabel(ax(2),'Tension N');
plot(ax(3),Revs1,ffr_twist);
ylabel(ax(3),'friction');
plot(ax(4),Revs1,LtoTwist);
ylabel(ax(4),'L to Twist m');

title(ax(1),sprintf('r_1 = %.2f r_2 = %.2f', r1, r2));
set(ax(2:2:end),'YAxisLocation','right');
set(ax(1:end-1),'XTickLabels',[]);
linkaxes(ax,'x');
%%
% Debug plot
if debug
Revs0 = theta/(2*pi);
Revs1 = theta_contact/(2*pi) + thetatwist/(2*pi);
figure;
ax = [ nsubplot(4,1,1) nsubplot(4,1,2) nsubplot(4,1,3) nsubplot(4,1,4) ];
plot(ax(1),Revs0,tau,Revs1,tau_twist);
ylabel(ax(1),'\tau Nm');
plot(ax(2),Revs0,F,Revs1,F_hz_twist);
ylabel(ax(2),'F_{hor} N');
plot(ax(3),Revs0,phitau,Revs1,phitau_twist + 0*Revs1);
ylabel(ax(3),'A_{radial} rad');
plot(ax(4),Revs0,dp2,Revs1,r3+0*Revs1);
ylabel(ax(4),'d_{perp} m');

title(ax(1),sprintf('r_1 = %.2f r_2 = %.2f', r1, r2));
set(ax(2:2:end),'YAxisLocation','right');
end
