%%
% Tether dynamics
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
clear all
close all
%%
Data = scopex_anal('scopex.log', 'Trial');
%%
Zangle = acosd(Data.tether.rotM(:,4*2+3));
%Azi1 = atan2d(Data.tether.rotM(:,4*2+2),Data.tether.rotM(:,4*2+1));
Azi = atan2d(Data.tether.rotM(:,4*1+3),Data.tether.rotM(:,4*0+3));
%%
Zangle = acosd(Data.gondola.rotM(:,4*2+3));
Azi = atan2d(Data.gondola.rotM(:,4*1+3),Data.gondola.rotM(:,4*0+3));
%%
figure;
ax = [nsubplot(2,1,1) nsubplot(2,1,2)];
plot(ax(1),Data.tether.T, Zangle);
set(ax(1),'XTickLabel',[],'YAxisLocation','Right');
ylabel(ax(1),'Zangle');
plot(ax(2),Data.tether.T,Azi);
ylabel(ax(2),'Azimuth');
linkaxes(ax,'x');
%%
figure;
polar(Azi*pi/180,Zangle);
%%
% What's up with rotM? I can't tell whether pre-multiplying or post
% is the right thing. If it's orthonormal, then the transpose should be
% the inverse. Let's check: tether(20000) [T==1000] gives different
% azimuth depending on interpretation.
M = reshape(Data.tether.rotM(20000,:),4,3)';
M = M(:,1:3);
Z = [0 0 1];
MZ = (M*Z')';
ZM = Z*M;
