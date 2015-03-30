function [S,R,Q,Lambda_psi] = init(bounds,start_pose,n_particles,total_t)
M = n_particles;

if ~isempty(start_pose)
    S = [repmat(start_pose,1,M); 1/M*ones(1,M)];
else
    S = [rand(1,M) * (bounds(1,2)-bounds(1,1))+bounds(1,1);
         rand(1,M) * (bounds(2,2)-bounds(2,1))+bounds(2,1);
         rand(1,M) * (bounds(3,2)-bounds(3,1))+bounds(3,1); %altitude
         rand(1,M)*2*pi-pi;     %heading init
         rand(1,M)*2*pi-pi;     %pitch init         
         1/M*ones(1,M)];
end
var=1/total_t^2;
R = diag([var var var var var]); %process noise covariance matrix
Q = diag([0.5;0.5;0.5]); % measurement noise covariance matrix
Lambda_psi = 0.0001;

end
