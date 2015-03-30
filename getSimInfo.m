function [simlines total_t]=getSimInfo(path)
fid = fopen(path,'r');
simlines=[];
total_t=0;
if fid <= 0
  disp(sprintf('Failed to open sim file "%s"\n',path));
  return
end

while 1
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    values = sscanf(line, '%f');
    simlines=[simlines; values'];
    total_t=total_t+1;
end
fclose(fid);
