function h = meas_model2(S,map,j,filter_idx)

M=size(S,2);
if (filter_idx==1)
diff=repmat(map(1:2,j),1,M)-S(1:2,:);
theta=atan2(diff(2,:),diff(1,:))-S(3,:);
%limiting to [-pi,pi)
theta=mod(theta+pi,2*pi)-pi;
diff=diff.^2;
dist=sqrt(sum(diff));
h=[dist;theta];

else
diff=repmat(map(3,j),1,M)-S(1,:);
pitch=atan2(diff(1,:),dist);
pitch=mod(pitch+pi,2*pi)-pi;
h=[dist ; theta ; pitch];

else
    
end