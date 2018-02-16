%%
clear all
close all
%%
D = load('Sdebug.log');
T = D(:,1);
Di = 2;
D = load('Sdebug.log');
T = D(:,1);
Di = 2;
[gondola,Di] = GetBodyData(D,Di,'Gondola');
[tether,Di] = GetBodyData(D,Di,'Tether');
[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
%%
PlotBodyData(gondola);
PlotBodyData(tether);
PlotJointData(TGjoint);
%%
nsubplotst(T, [tether.Torque(:,1),TGjoint.t1(:,1),tether.Torque(:,1)+TGjoint.t1(:,1)], 'Tether Torques');
%%
M = reshape(rotM(end,:),4,3)';
R = M(:,1:3);
Y = [0;1;0];
RY = R*Y;
YR = Y'*R;
