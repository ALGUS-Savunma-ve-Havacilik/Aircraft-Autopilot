function [S1_bar S2_bar] = predict2(S1,S2,v,heading_omega,pitch_omega,R,delta_t)
%Both Heading & Pitch lie in [-pi,pi]
 M1=size(S1,2);
 M2=size(S2,2);
 update1=[v*cos(S1(1,:)); 
          v*sin(S1(2,:));         
          repmat(heading_omega,1,M1)
         ];
 
 update2=[v*sin(S2(2,:));repmat(pitch_omega,1,M2)];
 
 S1_bar(1:3,:) = S1(1:3,:) + delta_t * update1;
 S1_bar(4,:) = mod(S1_bar(4,:)+pi,2*pi)-pi; %constraining to [-pi,pi]
 
 S2_bar(1:2,:)= S2(1:2,:) + delta_t * update2;
 S1_bar(2,:) = mod(S1_bar(2,:)+pi,2*pi)-pi; %constraining to [-pi,pi]
 
 %diffusion
 S1_bar(1:3,:) = S1_bar(1:3,:) + randn(3,M).*repmat(sqrt(diag(R(1:3,1:3))),1,M);
 S1_bar(4,:)=S1(4,:);
 
 S2_bar(1:2,:) = S2_bar(1:2,:) + randn(2,M).*repmat(sqrt(diag(R(1:2,1:2))),1,M);
 S2_bar(3,:)=S2(3,:);
end