%%
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
clear all
close all
%%
PGain = 1.4e-4;
DGain = 1;
IGain = 0;
VIGain = 0;
GAIL = 0.2;
VACL = 0;

gainiter = 'I';
exponent=7;

scale = 10^(-exponent);
for pi = [1 10 100] %[1 2 5]
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
scopex_anal('scopex.log', 'Trial');

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
