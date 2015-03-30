function info = getPoseInfo(true_pose,map)
n_landmarks=size(map,2);
info=zeros(1,3*n_landmarks);

for i=1:n_landmarks
    diff=map(:,i)-true_pose(:,1:3)';
    range=sqrt(sum(diff.^2,1));
    heading=atan2(diff(2),diff(1));
    pitch=atan2(diff(3),sqrt(diff(1)^2+diff(2)^2));
    i_ini=i*3-2;
    info(i_ini:i_ini+2)=[range heading pitch];
end