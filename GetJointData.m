function [J, Di] = GetJointData(D, col, name1, name2)
  Di = col;
  J.name1 = name1;
  J.name2 = name2;
  J.T = D(:,1);
  J.f1 = D(:,Di:Di+2); Di = Di + 3;
  J.t1 = D(:,Di:Di+2); Di = Di + 3;
  J.f2 = D(:,Di:Di+2); Di = Di + 3;
  J.t2 = D(:,Di:Di+2); Di = Di + 3;
