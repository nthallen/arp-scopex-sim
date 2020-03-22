function PlotJointData(J)
  jname = sprintf('%s/%s Joint', J.name1, J.name2);
  nsubplotst(J.T,J.t1,sprintf('%s Torque on %s',jname, J.name1));
  nsubplotst(J.T,J.f1,sprintf('%s Force on %s',jname, J.name1));
  nsubplotst(J.T,J.t2,sprintf('%s Torque on %s',jname, J.name2));
  nsubplotst(J.T,J.f2,sprintf('%s Force on %s',jname, J.name2));
