%%
D = load('demo1.log');
T = D(:,1)/20;
Yb = D(:,3);
Yp = D(:,9);
plot(T,Yb,T,Yp);
shg;
%%
D = load('demo3.log');
T = D(:,1);
Yp = D(:,4);
plot(T,Yp);
shg;
%%
V = [0; diff(Yp)./diff(T)];
plot(T,V,'.');
shg;
%%
A = [0; diff(V)./diff(T)];
plot(T,A,'.');
shg;
%%
%%
D = load('demo1nodrag.log');
T = D(:,1)/20;
Yb = D(:,3);
Yp = D(:,9);
D2 = load('demo1drag.log');
T2 = D2(:,1)/20;
Yb2 = D2(:,3);
Yp2 = D2(:,9);
plot(T,Yb,T,Yp,T2,Yb2,T2,Yp2);
shg;
%%
Vnodrag = [0; diff(Yp)./diff(T)];
Vdrag = [0; diff(Yp2)./diff(T2)];
plot(T, Vnodrag, T2, Vdrag);
shg;
%%
%close all;
D = load('SingleStep.log');
T = D(:,1)/20;
Yp = D(:,9);
Xp = D(:,8);
ThP = D(:,11);
% Yb = D(:,3);
% Xb = D(:,2);
figure;
plot(Xp,Yp);
idx = 1:600:length(T);
U = cosd(ThP(idx));
V = sind(ThP(idx));
hold on;
quiver(Xp(idx),Yp(idx),U,V);
hold off;
set(gca,'DataAspectRatio',[1,1,1]);
title('Payload Position');
xlabel('meters'); ylabel('meters');
legend('Position','Payload Direction');
shg;
%
ThPset = D(:,12);
ThPerr = ThP-ThPset;
V = ThPerr > 180;
ThPerr(V) = ThPerr(V) - 360;
V = ThPerr < -180;
ThPerr(V) = ThPerr(V) + 360;

figure;
plot(T,ThPset+ThPerr,T,ThPset);
ylabel('Payload Angle, deg');
xlabel('Elapsed Seconds');
legend('Observed','Command');
shg;
