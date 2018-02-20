function nsubplotst(X,Y,ttl)
  ax = nsubplots(X,Y);
  title(ax(1),ttl);
  set(gcf,'name',ttl);
  return
