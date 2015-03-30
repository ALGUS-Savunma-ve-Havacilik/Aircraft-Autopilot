function [outlier,Psi] = associate(S_bar,z,map,Lambda_psi,Q)

n=size(z,2);
M=size(S_bar,2);
K=size(map,2);
Q_inv=inv(Q);
Q_det=det(Q);
phi=zeros(K,M);
Psi=zeros(1,n,M);

for i=1:n            %for each measure
   rep_meas=repmat(z(:,i),1,M);  
   
   for k=1:K            %for each landmark
       h=meas_model(S_bar,map,k);
       nu=rep_meas-h;
       nu(2,:)=mod(nu(2,:)+pi,2*pi)-pi; %Bearing
       nu(3,:)=mod(nu(3,:)+pi,2*pi)-pi; %Pitch      
       phi(k,:)=((2*pi*sqrt(Q_det))^-1)*diag(exp(-0.5*nu'*Q_inv*nu));
   end
   
     
   Psi(1,i,:)=reshape(max(phi),1,1,M);
    
end

outlier=mean(Psi,3) <= Lambda_psi;

end