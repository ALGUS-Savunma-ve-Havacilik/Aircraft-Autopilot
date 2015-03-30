function h = meas_model(S,map,j)

M=size(S,2);
diff=repmat(map(:,j),1,M)-S(1:3,:);

theta=atan2(diff(2,:),diff(1,:))-S(4,:);


%limiting to [-pi,pi)
theta=mod(theta+pi,2*pi)-pi;


diff=diff.^2;
dist=sqrt(sum(diff));

pitch=atan2(diff(3,:),dist);
pitch=mod(pitch+pi,2*pi)-pi;
h=[dist ; theta ; pitch];
end