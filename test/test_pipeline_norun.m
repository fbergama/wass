%%
clc;clear;close all;
addpath([pwd(),'/../matlab/']);


TEST_ROOT    = [pwd(),'/WASS_TEST'];
CONFIG_DIR   = [TEST_ROOT,'/synth/config/'];
M3D_DIR      = [TEST_ROOT,'/synth/3D/'];
INPUT_C0_DIR = [TEST_ROOT,'/synth/input/cam0/'];
INPUT_C1_DIR = [TEST_ROOT,'/synth/input/cam1/'];
OUT_DIR      = [pwd(),'/output/'];


%%
% Sanity checks

assert( exist(OUT_DIR','dir')==7, sprintf('%s does not exists.', OUT_DIR ) );
assert( exist(TEST_ROOT','dir')==7, sprintf('%s does not exists.', TEST_ROOT ) );
assert( exist(CONFIG_DIR','dir')==7, sprintf('%s does not exists.', CONFIG_DIR ) );
assert( exist(M3D_DIR','dir')==7, sprintf('%s does not exists.', M3D_DIR ) );
assert( exist(INPUT_C0_DIR','dir')==7, sprintf('%s does not exists.', INPUT_C0_DIR ) );
assert( exist(INPUT_C1_DIR','dir')==7, sprintf('%s does not exists.', INPUT_C1_DIR ) );

%%
% List frames

input_frames = cell(0);

cam0_frames = dir(INPUT_C0_DIR);
kk=1;
for ii=1:numel(cam0_frames)
    if cam0_frames(ii).bytes > 0 && cam0_frames(ii).isdir == 0
        input_frames{kk} = struct('Cam0', [INPUT_C0_DIR,cam0_frames(ii).name], ...
                                  'wd', sprintf('%s%06d_wd/',OUT_DIR,kk-1) );
        kk=kk+1;
    end
end
clear('cam0_frames');
cam1_frames = dir(INPUT_C1_DIR);
kk=1;
for ii=1:numel(cam1_frames)
    if cam1_frames(ii).bytes > 0 && cam1_frames(ii).isdir == 0
        input_frames{kk}.Cam1 = [INPUT_C1_DIR, cam1_frames(ii).name];
        kk=kk+1;
    end
end

fprintf('%d stereo frames found.\n', numel( input_frames ) );

%%
% Check matches

status = verify_matcher( input_frames, CONFIG_DIR );
assert( strcmp(status, 'ok' ), ['wass_match failed: ',status]);

%%
% Check 3D data

fprintf('***************************************************\n');
fprintf('**  Verifying 3D point clouds                 *****\n');
fprintf('***************************************************\n');
verify_meshes( input_frames, CONFIG_DIR, M3D_DIR);
fprintf('***************************************************\n');
fprintf(' ALL TESTS OK!\n');


