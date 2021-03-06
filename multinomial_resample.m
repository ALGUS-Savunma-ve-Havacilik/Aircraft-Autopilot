% function S = multinomial_resample(S_bar)
% This function performs systematic re-sampling
% Inputs:   
%           S_bar(t):       4XM
% Outputs:
%           S(t):           4XM
function S = multinomial_resample(S_bar)
% FILL IN HERE

cdf = cumsum(S_bar(6,:));
M = size(S_bar,2);
S = zeros(6,M);
for m = 1 : M
    r_m = rand;
    i = find(cdf >= r_m,1,'first');
    S(1:5,m) = S_bar(1:5,i);
end
S(6,:) = 1/M*ones(1,M);

end
