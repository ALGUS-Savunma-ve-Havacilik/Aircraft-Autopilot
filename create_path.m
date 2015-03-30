function [simlines way_pass_t] = create_path(initial_pose,map,total_t)
n_landmarks=size(map,2);
simlines=zeros(total_t,5+3*n_landmarks);
waypoints=map;
if ~isempty(initial_pose)
    waypoints=[initial_pose(1:3,1) map];
end
vectors=diff(waypoints')';
distances=sqrt(sum(vectors(1:3,:).^2,1));
distance_tot=sum(distances(:));
unit_vectors=vectors./repmat(distances,3,1);
way_pass_t=zeros(n_landmarks,1);
line_idx=1;
for i=1:size(vectors,2)
    ratio=distances(i)/distance_tot;
    n_meas=floor(ratio*total_t);
    if (n_meas~=0)
         dist_unit=distances(i)/n_meas;
    end
    
    %Heading and pitch for the airway(edge).
    x_diff=vectors(1,i);
    y_diff=vectors(2,i);
    alt_diff=vectors(3,i);
    heading=atan2(y_diff,x_diff);
    pitch=atan2(alt_diff,sqrt(x_diff^2+y_diff^2));
    for j=1:n_meas
        x=waypoints(:,i)+(j-1)*dist_unit*unit_vectors(:,i);
        if(j==1 && i~=1)
            way_pass_t(i)=line_idx;
        end
        true_pose=[x' heading pitch];
        simlines(line_idx,:)=[true_pose getPoseInfo(true_pose,map)];
        line_idx=line_idx+1;
    end    
end
%Correction Measure
way_pass_t=[way_pass_t(2:end); total_t+1];


