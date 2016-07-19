function mesh_cam = load_camera_mesh( )
%LOAD_CAMERA_MESH Low-level function to load camera mesh from the current
%                 directory. Compressed/Non-compressed files are handled

fid=0;
if exist('mesh_cam.xyzC','file')
    compressed = 1; fid = fopen( 'mesh_cam.xyzC', 'r');
elseif exist( 'mesh_cam.xyzbin','file')
    compressed = 0; fid = fopen( 'mesh_cam.xyzbin', 'r');
end

assert( fid>0, 'mesh_cam.xyzbin or mesh_cam.xyzC not found');

frewind(fid);
npts = fread( fid, 1, 'uint32');

if compressed

    fprintf('Loading compressed data...\n');
    
    limits = fread( fid, 6, 'double');
    
    Rinv = fread( fid, [3 3], 'double' ); Rinv=Rinv';
    Tinv = fread( fid, 3, 'double');
    
    mesh_cam = double( fread( fid, [3 npts], 'uint16') );
    mesh_cam = mesh_cam ./ repmat( limits(1:3),1,size(mesh_cam,2) ) + repmat( limits(4:6),1,size(mesh_cam,2) );
    mesh_cam = Rinv*mesh_cam + repmat( Tinv,1,size(mesh_cam,2));
    
else
    mesh_cam = fread( fid, [3 npts], 'float32');
end

fclose(fid);


end

