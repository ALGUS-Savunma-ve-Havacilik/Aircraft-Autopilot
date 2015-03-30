function runAutopilot(mapfile,init_pose,total_t,verbose)

%map=create_map(width,height,base_alt,offset_alt,n_landmarks);
%map=[0 5;0 5;0 0];
%dataset_base='../Data/';

[map, n_landmarks]=getMapInfo(mapfile);
[simlines, way_pass_t]=create_path(init_pose,map,total_t);

if (mod(size(simlines,2)-5,n_landmarks) ~= 0)
    disp('Format Error in sim file or map file !');
    return;
end
waypoints=map;
if ~isempty(init_pose)
    waypoints=[init_pose(1:3,1) map];
end

%waypoints=[init_pose(1:3,1) map];
bounds=[min(waypoints(1,:)) max(waypoints(1,:));
        min(waypoints(2,:)) max(waypoints(2,:));
        min(waypoints(3,:)) max(waypoints(3,:))];

    
%Initialization
n_particles=200;   
[S,R,Q,Lambda_psi] = init(bounds,init_pose,n_particles,total_t);


%Plotting the display

fig=figure(1);
clf(fig);
drawMap(map);
x_margin = 0.2*(max(waypoints(1,:)) - min(waypoints(1,:)));
y_margin = 0.2*(max(waypoints(2,:)) -  min(waypoints(2,:)));
axis([min(waypoints(1,:))- x_margin max(waypoints(1,:))+x_margin ...
      min(waypoints(2,:))- y_margin max(waypoints(2,:))+y_margin]);
title('Lateral Navigation (LNAV)');
xlabel('x - coordinate');
ylabel('y - coordinate');

hold on;

figAlt=figure(2);
clf(figAlt);

waypoint_pass=[way_pass_t' ; map(3,:)];
drawMap(waypoint_pass);
title('Vertical Navigation (VNAV)');
y_margin=0.2*(max(waypoints(3,:)) -  min(waypoints(3,:)));
if (y_margin~=0)
    axis([0 total_t+2 min(waypoints(3,:))-y_margin max(waypoints(3,:))+y_margin]);
else
    axis([0 total_t+2 -1 1]);
end 
xlabel('Time step t');
ylabel('Altitude - z')
hold on;

true_poses=simlines(:,1:5);
sim_out=simlines(:,6:end);
parts=repmat((1:total_t),n_particles,1);
mu=zeros(3,total_t);

%run for t steps
for t=1:total_t
    z=reshape(sim_out(t,:),3,n_landmarks);
    mu(:,t)=mean(S(1:3,:),2);
    figure(fig);
    if(verbose)
        plot(S(1,:),S(2,:),'g.');
        pcov=make_covariance_ellipses( mu(1:2,t), cov( S(1:2,:)'));
        plot(pcov(1,:),pcov(2,:),'LineWidth',2);
    end
   
    plot(mu(1,t),mu(2,t),'r*');
    figure(figAlt);
    if(verbose)    
        plot(parts(:,t),S(3,:),'g.');
    end
    plot(parts(:,t),mu(3,t),'r*');
    [S,~] = mcl(S,R,Q,z,waypoint_pass(1,:),map,Lambda_psi,1,t);
end
figure(fig);
plot(true_poses(:,1),true_poses(:,2),'k.');

figure(figAlt);
plot((1:total_t),true_poses(:,3),'k.');

if (verbose>=2)
    lnav_err=sum((mu(1:2,:)- true_poses(:,1:2)') ,2);
    vnav_err=sum((mu(3,:)- true_poses(:,3)'),2);
    fprintf('Mean LNAV Error x %f\n',abs(lnav_err(1))/total_t);
    fprintf('Mean LNAV Error y %f\n',abs(lnav_err(2))/total_t);
    fprintf('Mean VNAV Error z %f\n',abs(vnav_err)/total_t);
end

