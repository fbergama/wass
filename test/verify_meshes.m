function status = verify_meshes( input_frames, CONFIG_DIR, M3D_DIR )
%VERIFY_MESHES 

for ii=1:numel(input_frames)
    wdir = input_frames{ii}.wd;
    [mesh, R, T] = load_camera_mesh_and_align_plane( [wdir,'/../'], ii-1, 1.0, 'plane.txt');
    mesh=mesh';
    assert( size(mesh,2)>3E6, sprintf('Mesh %s has less than 3E6 points',wdir));
    
    % load gt
    ptgt = pcread( sprintf('%s/%06d_3d.ply',M3D_DIR, ii-1 ) );
    meshgt = R * double(ptgt.Location') + repmat(T,1,size(ptgt.Location,1) );
    % invert meshgt z
    meshgt(3,:) = meshgt(3,:)*-1;
    
    %figure;
    %scatter3(meshgt(1,:), meshgt(2,:), meshgt(3,:),'ob' );
    %hold on;
    %scatter3(mesh(1,1:100:end), mesh(2,1:100:end), mesh(3,1:100:end),'.r' );
    
    Fgt = scatteredInterpolant(meshgt(1,:)',meshgt(2,:)',meshgt(3,:)');
    
    fprintf('Sampling gt elevations...\n' );
    gtelevations = Fgt( mesh(1,:), mesh(2,:) );
    abserr=abs(gtelevations-mesh(3,:));
    
    % remove outliers
    inliers = find(abserr<prctile( abserr, 99.9 ));
    fprintf('%d inliers. Mean absolute error %f\n', numel(inliers), mean(abserr(inliers)) );
    assert( mean(abserr(inliers))<0.02, sprintf('Mean absolute error > 0.02'));
    
    fprintf('%s........... test passed\n', wdir );
end


end

