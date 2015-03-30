function [S_bar] = predict(S,v,heading_omega,pitch_omega,R,delta_t)
%Both Heading & Pitch lie in [-pi,pi]
 M=size(S,2);
 update=[v*cos(S(4,:)); 
         v*sin(S(4,:));
         v*sin(S(5,:)); %Altitude
         repmat(heading_omega,1,M);
         repmat(pitch_omega,1,M)];
     
 S_bar(1:5,:) = S(1:5,:) + delta_t * update;
 S_bar(4:5,:) = mod(S_bar(4:5,:)+pi,2*pi)-pi; %constraining to [-pi,pi]
 %diffusion
 S_bar(1:5,:) = S_bar(1:5,:) + randn(5,M).*repmat(sqrt(diag(R)),1,M);
 S_bar(6,:)=S(6,:);
end