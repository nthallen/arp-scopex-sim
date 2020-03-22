function PlotBodyData(B)
nsubplotst(B.T,B.Pos,sprintf('%s Position',B.name));
nsubplotst(B.T,B.Vel,sprintf('%s Velocity',B.name));
nsubplotst(B.T,B.Force,sprintf('%s Force',B.name));
nsubplotst(B.T,B.Torque,sprintf('%s Torque',B.name));
nsubplotst(B.T,B.rotM(:,6:7),sprintf('%s Rotation',B.name));
