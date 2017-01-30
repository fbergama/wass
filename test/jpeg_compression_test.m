%%
clc;clear;close all;
addpath([pwd(),'/../matlab/']);

EXE_DIR      = [pwd(),'/../dist/bin/'];
if ispc
    PREPARE_EXE  = [EXE_DIR,'wass_prepare.exe'];
    MATCH_EXE    = [EXE_DIR,'wass_match.exe'];
    AUTOCAL_EXE  = [EXE_DIR,'wass_autocalibrate.exe'];
    STEREO_EXE   = [EXE_DIR,'wass_stereo.exe'];
else
    PREPARE_EXE  = [EXE_DIR,'wass_prepare'];
    MATCH_EXE    = [EXE_DIR,'wass_match'];
    AUTOCAL_EXE  = [EXE_DIR,'wass_autocalibrate'];
    STEREO_EXE   = [EXE_DIR,'wass_stereo'];
end


CONFIG_DIR   = [pwd(),'/WASS_TEST/W07/config/'];
DATA_DIR      = [pwd(),'/output_w07/000002_wd/'];
DATA_DIR_COMP = [pwd(),'/output_w07/000092_wd/'];



%%

assert( system( [STEREO_EXE, ' ', CONFIG_DIR, 'stereo_config.txt ', DATA_DIR] ) == 0, 'component exited with non-zero return code');

%%
%JPEG_QUALITY = 100;

for JPEG_QUALITY = [100 99 95 90 80 70 ]

copyfile([DATA_DIR,'undistorted/00000000.png'],[DATA_DIR_COMP,'undistorted/00000000.png'] );
copyfile([DATA_DIR,'undistorted/00000001.png'],[DATA_DIR_COMP,'undistorted/00000001.png'] );

% Compress
I=imread([DATA_DIR_COMP,'undistorted/00000000.png']);
imwrite(I,[DATA_DIR_COMP,'undistorted/00000000.jpg'],'Quality',JPEG_QUALITY);
I=imread([DATA_DIR_COMP,'undistorted/00000000.jpg']);
imwrite(I,[DATA_DIR_COMP,'undistorted/00000000.png']);

I=imread([DATA_DIR_COMP,'undistorted/00000001.png']);
imwrite(I,[DATA_DIR_COMP,'undistorted/00000001.jpg'],'Quality',JPEG_QUALITY);
I=imread([DATA_DIR_COMP,'undistorted/00000001.jpg']);
imwrite(I,[DATA_DIR_COMP,'undistorted/00000001.png']);

%%
assert( system( [STEREO_EXE, ' ', CONFIG_DIR, 'stereo_config.txt ', DATA_DIR_COMP] ) == 0, 'component exited with non-zero return code');

%%

copyfile([DATA_DIR,'plane.txt'],[DATA_DIR_COMP,'plane.txt'] );
[mesh, ~, ~] = load_camera_mesh_and_align_plane( [DATA_DIR,'/../'], 2, 2.5, 'plane.txt');
[mesh_c, ~, ~] = load_camera_mesh_and_align_plane( [DATA_DIR,'/../'], 92, 2.5, 'plane.txt');

%%
Fgt = scatteredInterpolant(mesh(:,1),mesh(:,2),mesh(:,3));
elevations = Fgt( mesh_c(:,1), mesh_c(:,2) );
abserr=abs(elevations-mesh_c(:,3));
%%

STEP=1;
hhfig = figure;
hold on;
scatter( mesh_c(1:STEP:end,1), mesh_c(1:STEP:end,2),50.0,abserr(1:STEP:end),'.');
axis ij;
axis equal;
caxis([0 0.1]);
colorbar;
title(sprintf('JPEG quality: %d', JPEG_QUALITY) );
drawnow;
ii = getframe(hhfig);
ii = ii.cdata;
close all;
imwrite(ii,sprintf('qual_%d.png',JPEG_QUALITY));

end
