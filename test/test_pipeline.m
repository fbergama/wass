%%
clc;clear;close all;
addpath([pwd(),'/../matlab/']);

EXE_DIR      = [pwd(),'/../dist/bin/'];
if ispc
    PREPARE_EXE  = [EXE_DIR,'wass_prepare.exe'];
    MATCH_EXE    = [EXE_DIR,'wass_match.exe'];
    AUTOCAL_EXE  = [EXE_DIR,'wass_autocalibrate.exe'];
    STEREO_EXE   = [EXE_DIR,'wass_stereo.exe'];
    ENV_SET      = '';
else
    PREPARE_EXE  = [EXE_DIR,'wass_prepare'];
    MATCH_EXE    = [EXE_DIR,'wass_match'];
    AUTOCAL_EXE  = [EXE_DIR,'wass_autocalibrate'];
    STEREO_EXE   = [EXE_DIR,'wass_stereo'];
    ENV_SET      = 'LD_LIBRARY_PATH="" && ';
end

TEST_ROOT    = [pwd(),'/WASS_TEST'];
CONFIG_DIR   = [TEST_ROOT,'/synth/config/'];
M3D_DIR      = [TEST_ROOT,'/synth/3D/'];
INPUT_C0_DIR = [TEST_ROOT,'/synth/input/cam0/'];
INPUT_C1_DIR = [TEST_ROOT,'/synth/input/cam1/'];
OUT_DIR      = [pwd(),'/output/'];


%%
% Sanity checks

assert( exist(EXE_DIR','dir')==7, sprintf('%s does not exists.', EXE_DIR ) );
assert( exist(PREPARE_EXE','file')==2, sprintf('%s does not exists.', PREPARE_EXE ) );
assert( exist(MATCH_EXE','file')==2, sprintf('%s does not exists.', MATCH_EXE ) );
assert( exist(AUTOCAL_EXE','file')==2, sprintf('%s does not exists.', AUTOCAL_EXE ) );
assert( exist(STEREO_EXE','file')==2, sprintf('%s does not exists.', STEREO_EXE ) );

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
% Prepare output directory

if exist(OUT_DIR','dir')==7
    fprintf('%s already exists, removing it\n', OUT_DIR );
    assert( rmdir( OUT_DIR, 's' )==1, 'Cannot remove');
end
fprintf('Creating %s\n', OUT_DIR );
mkdir( OUT_DIR );
    
%% 
% Run WASS prepare

fprintf('***************************************************\n');
fprintf('**  RUNNING wass_prepare                      *****\n');
fprintf('***************************************************\n');

tic;

for ii=1:numel(input_frames)
    assert( system( [ENV_SET, PREPARE_EXE, ' --workdir ', input_frames{ii}.wd, ' --calibdir ', CONFIG_DIR, ...
            ' --c0 ', input_frames{ii}.Cam0, ' --c1 ', input_frames{ii}.Cam1] ) == 0, 'component exited with non-zero return code');
end

fprintf('***************************************************\n');
fprintf(' Done in %f secs.\n', toc );

%% 
% Run WASS match

fprintf('***************************************************\n');
fprintf('**  RUNNING wass_match                        *****\n');
fprintf('***************************************************\n');

tic;

for ii=1:numel(input_frames)
    assert( system( [ENV_SET, MATCH_EXE, ' ', CONFIG_DIR, 'matcher_config.txt ', input_frames{ii}.wd] ) == 0, 'component exited with non-zero return code');
end

fprintf('***************************************************\n');
fprintf(' Done in %f secs.\n', toc );

status = verify_matcher( input_frames, CONFIG_DIR );
assert( strcmp(status, 'ok' ), ['wass_match failed: ',status]);

%%
% Run WASS autocalibrate

fprintf('***************************************************\n');
fprintf('**  RUNNING wass_autocalibrate                *****\n');
fprintf('***************************************************\n');

tic;

% create workspaces file
fid = fopen( [OUT_DIR,'/workspaces.txt'], 'w' );
for ii=1:numel(input_frames)
    fwrite(fid, [input_frames{ii}.wd,10] );
end
fclose(fid);

assert( system( [ENV_SET, AUTOCAL_EXE, ' ', OUT_DIR,'/workspaces.txt'] ) == 0, 'component exited with non-zero return code');

%% 
% Run WASS stereo

fprintf('***************************************************\n');
fprintf('**  RUNNING wass_stereo                       *****\n');
fprintf('***************************************************\n');

tic;

for ii=1:numel(input_frames)
    assert( system( [ENV_SET, STEREO_EXE, ' ', CONFIG_DIR, 'stereo_config.txt ', input_frames{ii}.wd] ) == 0, 'component exited with non-zero return code');
end

fprintf('***************************************************\n');
fprintf(' Done in %f secs.\n', toc );


%%
% Check 3D data

fprintf('***************************************************\n');
fprintf('**  Verifying 3D point clouds                 *****\n');
fprintf('***************************************************\n');
verify_meshes( input_frames, CONFIG_DIR, M3D_DIR);
fprintf('***************************************************\n');
fprintf(' ALL TESTS OK!\n');


