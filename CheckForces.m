function CheckForces(name,F1,a1,F2,a2,F3,a3)
x = F1*cos(a1) + F2*cos(a2) + F3*cos(a3);
y = F1*sin(a1) + F2*sin(a2) + F3*sin(a3);
fprintf(1,'%s dx = %f  dy = %f\n', name, x, y);
