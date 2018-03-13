function scopex_anal(filename, titles, imagename)
  formatSpec = (ones(32,1) * '%f')';
  formatSpec = [ char(formatSpec(:))', '%[^\n\r]'];
  fileID = fopen(filename,'r');
  dataArray = textscan(fileID, formatSpec, 'Delimiter', ',', 'WhiteSpace', '', 'EmptyValue' ,NaN, 'ReturnOnError', false);
  fclose(fileID);
  D = [dataArray{1:end-1}];
  clearvars filename formatSpec fileID dataArray ans;
  %
  T = D(:,1);
  Di = 2;
  [gondola,Di] = GetBodyData(D,Di,'Gondola');
  %[tether,Di] = GetBodyData(D,Di,'Tether');
  %[balloon,Di] = GetBodyData(D,Di,'Balloon');
  %[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
  [Thrust,Di] = GetSimVar(D,'Thrust',Di,2);
  [GAngles,Di] = GetSimVar(D,'GondolaAngles',Di,5);
  %
  %PlotBodyData(gondola);
  %PlotBodyData(tether);
  %PlotBodyData(balloon);
  %PlotJointData(TGjoint);
  %PlotSimVar(Thrust);
  %PlotSimVar(GAngles);
  
  % Figure showing summary of angles and errors
  figure;
  velocityAngleError = ...
    mod(GAngles.GondolaAngles(:,2)-GAngles.GondolaAngles(:,4)+180,360)-180;
  gondolaAngleError = ...
    mod(GAngles.GondolaAngles(:,1)-GAngles.GondolaAngles(:,4)+180,360)-180;
  GAset = ...
    mod(GAngles.GondolaAngles(:,5)-GAngles.GondolaAngles(:,4)+180,360)-180;
  plot(GAngles.T, velocityAngleError, GAngles.T, gondolaAngleError, ...
    GAngles.T, GAset);
  legend('velocity','gondola','GAset');
  title(sprintf('Difference from direction: %s', titles));
  set(gcf,'name',sprintf('Errs:%s',titles));
  if nargin >= 3
    print(gcf, '-dpng', '-r300', ['PID_err_' imagename '.png']);
  end

  % Quiver plot showing trajectory
  figure;
  idx = 1:600:length(gondola.T);
  U = cosd(GAngles.GondolaAngles(idx,1));
  V = sind(GAngles.GondolaAngles(idx,1));
  quiver(gondola.Pos(idx,1),gondola.Pos(idx,2),U,V);
  hold on;
  h = plot(gondola.Pos(:,1),gondola.Pos(:,2));
  set(h,'linewidth',2);
  set(gca,'DataAspectRatio',[1,1,1]);
  hold off;
  title(sprintf('Payload Position: %s', titles));
  xlabel('meters'); ylabel('meters');
  legend('Payload Direction','Position','location','southeast');
  set(gcf,'name',sprintf('Pos:%s', titles));
  shg;
  if nargin >= 3
    print(gcf, '-dpng', '-r300', ['PID_traj_' imagename '.png']);
  end
  
  figure;
  plot(GAngles.T,GAngles.GondolaAngles(:,3));
  ylabel('Speed m/s');
  xlabel('Seconds');
  title(sprintf('Speed:%s',titles));
  set(gcf,'name',sprintf('Speed:%s', titles));
  if nargin >= 3
    print(gcf, '-dpng', '-r300', ['PID_speed_' imagename '.png']);
  end
