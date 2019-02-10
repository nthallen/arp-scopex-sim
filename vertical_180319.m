%%
% vertical 180319.m
cd C:\Users\nort.ARP\Documents\Exp\SCoPEx\work
clear all
close all
%%
Data = scopex_load('scopex.log');
Z = Data.gondola.Pos(:,3);
T = Data.gondola.T;
Zmax = max(Z);
V = Z > Zmax-60;
Zmean = mean(Z(V));
%%
% Alternate strategy to locate interesting point
% Go to first local max, find min after that and set min value below that
dZi = find(diff(Z)<0,1);
Zmax = min(Z(dZi:end)) - 20;
V = Z > Zmax;
Zmean = mean(Z(V));
%%
figure;
ax = [ subplot(2,1,1) subplot(2,1,2) ];
plot(ax(1),T/3600,Z/1000);
xlabel(ax(1),'Hours');
ylabel(ax(1),'km');
title(ax(1),'Simulated Ascent');

plot(ax(2), T(V)/3600,Z(V)-Zmean); shg
xlabel(ax(2), 'Hours');
ylabel(ax(2), 'meters');
title(ax(2), 'Oscillations at Float: Period ~2.5 min');
xl = xlim(ax(2));
xl(1) = min(T(V))/3600-1/60;
xl(2) = 3;
xlim(ax(2), xl);
%%
% This is for estimating the period
P = [ cursor_info.Position ];
x = sort(P(1:2:end));
figure;
plot(diff(x)/60,'*');
%%
figure;
ax(1) = subplot(2,1,1);
plot(T/3600,Z/1000);
xlabel('Hours');
ylabel('km');
title('Simulated Ascent');
[ xl, yl, hh ] = add_inset;
xl = xl*3600;
VI = T >= xl(1) & T <= xl(2);
minTVI = min(T(VI));
ZVImean = mean(Z(VI));
plot(hh, T(VI) - minTVI, Z(VI)-ZVImean);
%%
Data = scopex_load('scopex.log');
PlotSimVar(Data.HeVol);
