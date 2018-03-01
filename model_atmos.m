%%
load model_atmos1.mat
hPa = press/100;
lnhPa = log(hPa);
for i=1:length(alt)
  fprintf(1,'%.1f,%.1f,%.2f,%.5f\n', ...
    alt(i),temp(i),hPa(i),lnhPa(i));
end
