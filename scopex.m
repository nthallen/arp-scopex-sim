%%
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
clear all
close all
%%
% Copied from scopex.cpp
D = load('scopex.log');
%%
filename = 'scopex.log';
formatSpec = '%7f%13f%13f%15f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%13f%12f%13f%f%[^\n\r]';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);
D = [dataArray{1:end-1}];
clearvars filename formatSpec fileID dataArray ans;
%%
T = D(:,1);
Di = 2;
[gondola,Di] = GetBodyData(D,Di,'Gondola');
%[tether,Di] = GetBodyData(D,Di,'Tether');
%[balloon,Di] = GetBodyData(D,Di,'Balloon');
%[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
[Thrust,Di] = GetSimVar(D,'Thrust',Di,2);
[GAngles,Di] = GetSimVar(D,'GondolaAngles',Di,3);
%%
PlotBodyData(gondola);
%PlotBodyData(tether);
%PlotBodyData(balloon);
%PlotJointData(TGjoint);
PlotSimVar(Thrust);
PlotSimVar(GAngles);
%%
figure;
plot(gondola.Pos(:,1),gondola.Pos(:,2));
idx = 1:600:length(gondola.T);
U = cosd(GAngles.GondolaAngles(idx,1));
V = sind(GAngles.GondolaAngles(idx,1));
hold on;
quiver(gondola.Pos(idx,1),gondola.Pos(idx,2),U,V);
hold off;
set(gca,'DataAspectRatio',[1,1,1]);
title('Payload Position');
xlabel('meters'); ylabel('meters');
legend('Position','Payload Direction');
shg;
