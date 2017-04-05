%%
D = load('demo1.log');
T = D(:,1)/20;
Yb = D(:,3);
Yp = D(:,9);
plot(T,Yb,T,Yp);
shg;
