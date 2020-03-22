%%
cd C:\Users\nort\Documents\Documents\Exp\SCoPEx\scopex-sim
clear all
close all
%%
PGain = 4e-4;
DGain = 1;
IGain = 0;
VIGain = 0;
GAIL = 0.2;
VACL = 0;

gainiter = 'I';
exponent=9;

scale = 10^(-exponent);
for pi = [1 2 5]
  if gainiter == 'P'
    PGain = pi*scale;
  elseif gainiter == 'D'
    DGain = pi*scale;
  elseif gainiter == 'I'
    IGain = pi*scale;
  end
  write_scp('PGain', PGain, '%.1e', ...
    'IGain', IGain, '%.1e', ...
    'DGain', DGain, '%.1e', ...
    'VIGain', VIGain, '%.6f', ...
    'gondolaAngleIntegralLimit', GAIL, '%.3f', ...
    'velocityAngleCorrLimit', VACL, '%.0f');
  system('/cygwin64/bin/bash ./mdoit.sh');
  scopex_anal('scopex.log', sprintf('%sGain:%.1e', gainiter, pi*scale), ...
    sprintf('%s%d_%d', gainiter, exponent, pi));
end
% fprintf(1,'Done\n');
%%
system('/cygwin64/bin/bash ./doit');
%%
Data = scopex_anal('scopex.log', 'Trial');

%% This stuff has moved to scopex_anal()
filename = 'scopex.log';
%formatSpec = '%9f%13f%13f%15f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%12f%13f%13f%f%[^\n\r]';
formatSpec = (ones(32,1) * '%f')';
formatSpec = [ char(formatSpec(:))', '%[^\n\r]'];
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', ',', 'WhiteSpace', '', 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
D = [dataArray{1:end-1}];
clearvars filename formatSpec fileID dataArray ans;
%
T = D(:,1);
Di = 2;
[gondola,Di] = GetBodyData(D,Di,'Gondola');
%[tether,Di] = GetBodyData(D,Di,'Tether');
%[balloon,Di] = GetBodyData(D,Di,'Balloon');
%[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
[Thrust,Di] = GetSimVar(D,'Thrust',Di,2);
[GAngles,Di] = GetSimVar(D,'GondolaAngles',Di,5);
%
%PlotBodyData(gondola);
%PlotBodyData(tether);
%PlotBodyData(balloon);
%PlotJointData(TGjoint);
%PlotSimVar(Thrust);
%PlotSimVar(GAngles);
%
figure;
velocityAngleError = ...
  mod(GAngles.GondolaAngles(:,2)-GAngles.GondolaAngles(:,4)+180,360)-180;
gondolaAngleError = ...
  mod(GAngles.GondolaAngles(:,1)-GAngles.GondolaAngles(:,4)+180,360)-180;
GAset = ...
  mod(GAngles.GondolaAngles(:,5)-GAngles.GondolaAngles(:,4)+180,360)-180;
plot(GAngles.T, velocityAngleError, GAngles.T, gondolaAngleError, ...
   GAngles.T, GAset);
legend('velocity','gondola','GAset');
title('Difference from direction');
%
figure;
idx = 1:600:length(gondola.T);
U = cosd(GAngles.GondolaAngles(idx,1));
V = sind(GAngles.GondolaAngles(idx,1));
quiver(gondola.Pos(idx,1),gondola.Pos(idx,2),U,V);
hold on;
h = plot(gondola.Pos(:,1),gondola.Pos(:,2));
set(h,'linewidth',2);
set(gca,'DataAspectRatio',[1,1,1]);
hold off;
title('Payload Position');
xlabel('meters'); ylabel('meters');
legend('Position','Payload Direction','location','south');
shg;
%%
% This is analysis for wind sensor
Data = scopex_load('scopex.log');
RotM = Data.gondola.rotM;
azi = atan2(RotM(:,3),RotM(:,7))*180/pi;
azi(azi>179) = azi(azi>179)-360;
zen = acos(RotM(:,11))*180/pi;
figure;
ax = [ nsubplot(2, 1, 1), nsubplot(2,1,2)];
plot(ax(1), Data.gondola.T, azi);
set(ax(1),'xticklabel', [],'YAxisLocation','Right');
ylabel(ax(1),'Azimuth deg');
plot(ax(2), Data.gondola.T, zen);
ylabel(ax(2),'Zenith deg');
linkaxes(ax,'x');
%%
% Use with data from previous section to calculate position of
% wind sensor tether point
% This assumes gondola central structure of 1m cube with the center
% of mass at the center. The forward direction on the gondola is
% positive Y, and the wind sensor will be placed at the bottom center
% of the leading face.
wind_tether_offset = [0; 0.5; -0.5];
% Apply the rotation matrix to the offset to get the rotational
% component of the position.
wind_tether_rotated = zeros(size(RotM,1),3);
for i = 1:size(RotM,1)
    M = reshape(RotM(i,:),4,3)';
    wind_tether_rotated(i,:) = (M(:,1:3)*wind_tether_offset)';
end
wind_tether_position = Data.gondola.Pos + wind_tether_rotated;
gondola_position = Data.gondola.Pos;
T = Data.gondola.T;
save wind_tether_position.mat wind_tether_position wind_tether_rotated gondola_position T
%%
scopex_anal('scopex.log','Feed Forward');
