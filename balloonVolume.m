function [volume,hout,wout,area] = balloonVolume(maxHeight, maxDiameter, k, resolution)
  if nargin < 4
    resolution = 101;
    if nargin < 3
      k = 1/3;
    end
  end
  r = k*maxDiameter; % semicircle radius
  r1 = maxDiameter/2 - r; % rectangle half-width
  % square of distance from bottom of triangle to center of semicircle
  d12 = (maxHeight-r).^2 + r1.^2;
  d1 = sqrt(d12);
  d2 = sqrt(d12 - r.^2);
  theta = atan(r1/(maxHeight-r))+asin(r/d1);
  d3 = d2*cos(theta);
  hc = maxHeight - r;
  h = linspace(0,maxHeight,resolution);
  v1 = h <= d3;
  w = zeros(size(h));
  w(v1) = h(v1)*tan(theta);
  v2 = h >= d3;
  h1 = h - maxHeight + r;
  w(v2) = sqrt(r^2 - h1(v2).^2)+r1;
  dh = mean(diff(h));
  wb = (w(1:end-1)+w(2:end))/2;
  ab = pi * wb.^2;
  volume = sum(ab)*dh;
  if nargout >= 3
    hout = [h fliplr(h)]';
    wout = [w fliplr(-w)]';
  end
  if nargout >= 4
    area = sum(w)*2*dh;
  end
