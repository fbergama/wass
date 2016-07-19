function [mesh, R, T] = load_camera_mesh_and_align_plane( datadir, idx, scale, planefile )
%LOAD_CAMERA_MESH_AND_ALIGN_PLANE

workdir = sprintf( '%s%06d_wd/', datadir, idx);
prevloc = cd(workdir);
addpath(prevloc);

mesh_cam = load_camera_mesh();

n_pts = size(mesh_cam,2);

plane = importdata(planefile);
assert( size(plane,1)==4, 'invalid plane file');

cd(prevloc)


%% Compute RT from plane 

a=plane(1);b=plane(2);c=plane(3);d=plane(4);
q = (1-c)/(a*a + b*b);
R=eye(3);T=zeros(3,1);
R(1,1) = 1-a*a*q;
R(1,2) = -a*b*q;
R(1,3) = -a;
R(2,1) = -a*b*q;
R(2,2) = 1-b*b*q;
R(2,3) = -b;
R(3,1) = a;
R(3,2) = b;
R(3,3) = c;
T(1)=0;
T(2)=0;
T(3)=d;
    
%% Rotate, translate

mesh=R*mesh_cam + repmat(T,1,n_pts);

mesh=mesh';
% apply scale
mesh = mesh*scale;
% Invert z axis
mesh(:,3)=mesh(:,3)*-1.0;


end

