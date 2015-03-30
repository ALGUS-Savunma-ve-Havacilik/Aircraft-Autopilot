function [S1,S2] = mcl2(S1,S2,R,Q,z1,z2,t_waypoint_pass,map,Lambda_psi,delta_t,t)

%set next action particle filter 1
[v bearing_omega pitch_omega]=setActuators(S1,map,t_waypoint_pass,t);
%v=0.1;
%pitch_omega=0;
%bearing_omega=0;

[S1_bar,S2_bar] = predict2(S1,S2,v,bearing_omega,pitch_omega,R,delta_t);

[outlier,Psi] = associate(S_bar,z,map,Lambda_psi,Q);

outliers = sum(outlier);
if outliers
    display(sprintf('warning, %d measurements were labeled as outliers, t=%d',sum(outlier), t));
end

S_bar = weight(S_bar,Psi,outlier); %Weight particles

RESAMPLE_MODE = 1; %0=Multinomial re-sampling, 1=Systematic Re-sampling
switch RESAMPLE_MODE
    case 0
        S = multinomial_resample(S_bar);
    case 1
        S = systematic_resample(S_bar);
end
end