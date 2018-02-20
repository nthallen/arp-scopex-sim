%%
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
clear all
close all
%%
% Copied from scopex.cpp
D = load('scopex.log');
T = D(:,1);
Di = 2;
[gondola,Di] = GetBodyData(D,Di,'Gondola');
[tether,Di] = GetBodyData(D,Di,'Tether');
[balloon,Di] = GetBodyData(D,Di,'Balloon');
[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
[Thrust,Di] = GetSimVar(D,'Thrust',Di);
%%
PlotBodyData(gondola);
PlotBodyData(tether);
PlotBodyData(balloon);
PlotJointData(TGjoint);
PlotSimVar(Thrust);
