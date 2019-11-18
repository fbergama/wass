function  plot_camera_as_ply( workdir, camid )
%PLOT_CAMERAS_AS_PLY Creates a ply file showing the camera position and
%                    axes on the scene
%
%  Parameters: 
%              workdir: any WASS output workdir (like 000000_wd)
%                camid: camera id (0 or 1)
%
%   A new file <workdir>/camera_<id>.ply is produced

K = importdata( sprintf("%s/K%d_small.txt",workdir,camid) );
R = importdata( sprintf("%s/Cam%d_poseR.txt",workdir,camid) );
T = importdata( sprintf("%s/Cam%d_poseT.txt",workdir,camid) );

R = R';
T = -R*T;

I = imread( sprintf("%s/%08d_s.png",workdir,camid) );
W = size(I,2);
H = size(I,1);

%disp(K);
%disp(R);
%disp(T);


frame_pts = [ 0 W W 0 ;...
              0 0 H H ;...
              1 1 1 1 ];

axes_pts = [ 0 1 0 0 ;...
             0 0 1 0 ;...
             0 0 0 100 ];
         

axes_pts = R * axes_pts + T;
frame_pts = R * inv(K) * frame_pts + T;


fid = fopen( sprintf("%s/camera_%d.ply",workdir,camid), 'w' );
fprintf(fid,'ply\nformat ascii 1.0\ncomment camera\n');
fprintf(fid,'element vertex %d\n', size(frame_pts,2)+size(axes_pts,2) );
fprintf(fid,'property float x\n');
fprintf(fid,'property float y\n');
fprintf(fid,'property float z\n');
fprintf(fid,'element edge 7\n');                    
fprintf(fid,'property int vertex1\n');                  
fprintf(fid,'property int vertex2\n');                  
fprintf(fid,'end_header\n');
% vertices
for ii=1:size(axes_pts,2)
    fprintf(fid,'%f %f %f\n', axes_pts(:,ii));
end
for ii=1:size(frame_pts,2)
    fprintf(fid,'%f %f %f\n', frame_pts(:,ii));
end
% edges
fprintf(fid,"0 1\n");
fprintf(fid,"0 2\n");
fprintf(fid,"0 3\n");
fprintf(fid,"4 5\n");
fprintf(fid,"5 6\n");
fprintf(fid,"6 7\n");
fprintf(fid,"7 4\n");
fclose(fid);
end

