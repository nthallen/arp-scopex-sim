%%
% This script was used to convert the tables in
% model_atmos1.txt into the format required for
% model_atmos.cpp. There was interactive text
% processing to convert model_atmos.txt to
% model_atmos1.csv, and then I used csv2mat to
% convert model_atmos1.csv to model_atmos1.mat.
% Finally, I ran this script to generate the
% C++ table contents to paste into model_atmos.cpp.
load model_atmos1.mat
hPa = press/100;
lnhPa = log(hPa);
for i=1:length(alt)
  fprintf(1,'%.1f,%.1f,%.2f,%.5f\n', ...
    alt(i),temp(i),hPa(i),lnhPa(i));
end
