function runAutopilot2(mapfile,init_pose,total_t,verbose)

%map=create_map(width,height,base_alt,offset_alt,n_landmarks);
%map=[0 5;0 5;0 0];
dataset_base='../Data/';

[map n_landmarks]=getMapInfo(strcat(dataset_base,mapfile));

[simlines way_pass_t]=create_path(init_pose,map,total_t);

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
S1=[S(1:2,:);S(4,:);S(6,:)];
S2=[S(3,:);S(5,:);S(6,:)];


%Plotting the display

fig=figure(1);
clf(fig);
drawMap(map);
x_margin = 0.2*(max(waypoints(1,:)) - min(waypoints(1,:)));
y_margin = 0.2*(max(waypoints(2,:)) -  min(waypoints(2,:)));
axis([min(waypoints(1,:))- x_margin max(waypoints(1,:))+x_margin ...
      min(waypoints(2,:))- y_margin max(waypoints(2,:))+y_margin]);
title('Lateral Navigation (LNAV)');
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
hold on;

true_poses=simlines(:,1:5);
sim_out=simlines(:,6:end);
sim2=sim_out(:,3:3:end);
sim1=setdiff(sim_out,sim2);

parts=repmat((1:total_t),n_particles,1);
mu1 = zeros(2,total_t);
mu2 = zeros(1,total_t);
%run for t steps
for t=1:total_t
    %Getting Measurements from simulation
    z1=reshape(sim1(t,:),2,n_landmarks);
    z2=sim2(t,:);
    figure(fig);
    if(verbose)
        plot(S1(1,:),S1(2,:),'g.');
    end
    mu1(:,t)=mean(S1(1:2,:),2);
    plot(mu1(1,t),mu1(2,t),'r*');
    figure(figAlt);
    if(verbose)    
        plot(parts(:,t),S2(1,:),'g.');
    end
    mu2(t)=mean(S2(1,:),2);
    plot(parts(:,t),mu2(1,t),'r*');
    
    
    %Monte Carlo Localization
    [S1,S2] = mcl2(S1,S2,R,Q,z1,z2,waypoint_pass(1,:),map,Lambda_psi,1,t);
    
    
end
figure(fig);
plot(true_poses(:,1),true_poses(:,2),'k.');

figure(figAlt);
plot((1:total_t+1),true_poses(:,3),'k.');

%showFinalInfo();
%lnav_err=sum((mu(1:2,:)- true_poses(:,1:2)').^2 ,2);
%fprintf('LNAV Error %f\n',lnav_err);


