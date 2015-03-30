function map = create_map(width,height,base_alt,offset_alt,n_landmarks)
%% Creates a map of size width x height with randomely n landmarks (fixes)
r_mat=randn([3 n_landmarks]);
map=r_mat.*repmat([width height offset_alt]', 1 ,n_landmarks);
map(3,:)=map(3,:)+base_alt;

end