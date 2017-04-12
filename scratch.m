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
