function [v  bearing_omega pitch_omega]=setActuators(S,map,t_waypoint_pass,t)
target_idx=find(t < t_waypoint_pass,1,'first');
mu=mean(S(1:5,:),2);
diff=map(:,target_idx)-mu(1:3);
v=sqrt(sum(diff.^2,1))/(t_waypoint_pass(target_idx)-t);
bearing_omega=(atan2(diff(2),diff(1))-mu(4));
pitch_omega=atan2(diff(3),sqrt(diff(1)^2+diff(2)^2))-mu(5);
