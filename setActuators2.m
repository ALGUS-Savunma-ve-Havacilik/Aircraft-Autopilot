function [v  bearing_omega pitch_omega]=setActuators2(S1,S2,map,t_waypoint_pass,t)
target_idx=find(t < t_waypoint_pass,1,'first');
mu1=mean(S1(1:2,:),2);
mu2=mean(S2(1,:),2);
diff=map(1:3,target_idx)- [mu1(1:2);mu2(1)];
v=sqrt(sum(diff.^2,1))/(t_waypoint_pass(target_idx)-t);
bearing_omega=(atan2(diff(2),diff(1))-mu1(3));
pitch_omega=atan2(diff(3),sqrt(diff(1)^2+diff(2)^2))-mu2(2);
