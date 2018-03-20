function Data = scopex_load(filename)
  formatSpec = (ones(1+24+24+2+5,1) * '%f')';
  formatSpec = [ char(formatSpec(:))', '%[^\n\r]'];
  fileID = fopen(filename,'r');
  dataArray = textscan(fileID, formatSpec, 'Delimiter', ',', 'WhiteSpace', '', 'EmptyValue' ,NaN, 'ReturnOnError', false);
  fclose(fileID);
  D = [dataArray{1:end-1}];
  % clearvars filename formatSpec fileID dataArray ans;
  %
  % T = D(:,1);
  Di = 2;
  [gondola,Di] = GetBodyData(D,Di,'Gondola');
  [tether,Di] = GetBodyData(D,Di,'Tether');
  %[balloon,Di] = GetBodyData(D,Di,'Balloon');
  %[TGjoint,Di] = GetJointData(D,Di,'Tether','Gondola');
  [Thrust,Di] = GetSimVar(D,'Thrust',Di,2);
  [GAngles,~] = GetSimVar(D,'GondolaAngles',Di,5);
  Data.gondola = gondola;
  Data.tether = tether;
  Data.Thrust = Thrust;
  Data.GAngles = GAngles;
