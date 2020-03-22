function write_scp(varargin)
  ifd = fopen('Tune_base.scp', 'r');
  ofd = fopen('Tune.scp', 'w');
  for i=1:3:length(varargin)
    name = varargin{i};
    value = varargin{i+1};
    fmt = varargin{i+2};
    fmtfmt = sprintf('0 Set %%s %s\\n', fmt);
    fprintf(ofd,fmtfmt, name, value);
  end
  tline = fgets(ifd);
  while ischar(tline)
    fprintf(ofd,'%s',tline);
    tline = fgets(ifd);
  end
  fclose(ifd);
  fclose(ofd);
  
