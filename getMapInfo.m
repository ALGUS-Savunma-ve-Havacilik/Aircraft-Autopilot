function [map n_landmarks] = getMapInfo(path)
n_landmarks=0;
map=[];
fid = fopen(path,'r');
if fid <= 0
  disp(sprintf('Failed to open map file "%s"\n',path));
  return
end

while 1
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    values = sscanf(line, '%f');
    map=[map values];
    n_landmarks=n_landmarks+1;
end
fclose(fid);

