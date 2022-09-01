%%
% Helper script to process a dataset with the WASS pipeline


clc;
clear;
close all;
delete(gcp('nocreate'));

%%

% Sequence settings
MAX_FRAMES = -1;            % use -1 to process all the frames
MAX_FRAMES_TO_MATCH = 50;   % generally no need to change this


DO_PREPARE=1;               % execute the prepare step: 0:no/1:yes
DO_MATCH=1;                 % execute the match step: 0:no/1:yes
DO_AUTOCALIBRATE=1;         % execute the autocalibrate step: 0:no/1:yes
DO_STEREO=1;                % execute the dense sterep step: 0:no/1:yes


%--------------------------------------------------------------------------------------

%% Stereo sequence and Config. files

DATA_ROOT    = [pwd(),'/'];                         % Data root dir

%-----------------------------
%%%%% Config. files DIR
%-----------------------------
CONFIG_DIR   = [DATA_ROOT,'config/'];               % dir containing the intrinsic calibration and
                                                    % configuration files

%-----------------------------
%%%%% cam 0 DIR
%-----------------------------
INPUT_C0_DIR = [DATA_ROOT,'/input/cam1_left/'];     % Cam1 image data

%-----------------------------
%%%%% cam 1 DIR
%-----------------------------
INPUT_C1_DIR = [DATA_ROOT,'/input/cam2_right/'];    % Cam0 image data

%-----------------------------
%%%%% Output DIR
%-----------------------------
OUT_DIR      = [DATA_ROOT,'/output/'];              % Output dir


%--------------------------------------------------------------------------------------

%% WASS Executables
%%%%% WASS dir
wass_dir='~/wass';          % WASS pipeline location


%-----------------------------
%%%%% matlab path
%-----------------------------
addpath([wass_dir '/matlab/']);
%-----------------------------
%%%%% WASS EXE. Dir
%-----------------------------
EXE_DIR      = [wass_dir,'/dist/bin/'];
%-----------------------------
%%%%% WASS EXECUTABLE files
%-----------------------------
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
assert( exist(EXE_DIR','dir')==7, sprintf('%s does not exists.', EXE_DIR ) );
assert( exist(PREPARE_EXE','file')==2, sprintf('%s does not exists.', PREPARE_EXE ) );
assert( exist(MATCH_EXE','file')==2, sprintf('%s does not exists.', MATCH_EXE ) );
assert( exist(AUTOCAL_EXE','file')==2, sprintf('%s does not exists.', AUTOCAL_EXE ) );
assert( exist(STEREO_EXE','file')==2, sprintf('%s does not exists.', STEREO_EXE ) );

assert( exist(DATA_ROOT','dir')==7, sprintf('%s does not exists.', DATA_ROOT ) );
assert( exist(CONFIG_DIR','dir')==7, sprintf('%s does not exists.', CONFIG_DIR ) );
assert( exist(INPUT_C0_DIR','dir')==7, sprintf('%s does not exists.', INPUT_C0_DIR ) );
assert( exist(INPUT_C1_DIR','dir')==7, sprintf('%s does not exists.', INPUT_C1_DIR ) );

%% List frames
%-----------------------------
input_frames = cell(0);

%-----------------------------
%%%%% cam0 frames
%----------------------------
cam0_frames = dir( [INPUT_C0_DIR,'/*tif']);

fprintf('%d stereo frames found.\n', numel( cam0_frames ) );
if MAX_FRAMES<=0
    MAX_FRAMES = numel(cam0_frames);
end

kk=1;
for ii=1:numel(cam0_frames)
    if cam0_frames(ii).bytes > 0 && cam0_frames(ii).isdir == 0
        input_frames{kk} = struct('Cam0', [INPUT_C0_DIR,cam0_frames(ii).name], ...
                                  'wd', sprintf('%s%06d_wd/',OUT_DIR,kk-1) );
        kk=kk+1;
        if numel( input_frames ) == MAX_FRAMES
            break
        end
    end
end
%clear('cam0_frames');

%-----------------------------
%%%%% cam1 frames
%-----------------------------
cam1_frames = dir([INPUT_C1_DIR,'/*tif']);
kk=1;
for ii=1:numel(input_frames)
    if cam1_frames(ii).bytes > 0 && cam1_frames(ii).isdir == 0
        input_frames{kk}.Cam1 = [INPUT_C1_DIR, cam1_frames(ii).name];
        kk=kk+1;
    end
end

fprintf('%d stereo frames found.\n', numel( input_frames ) );



if DO_PREPARE==1 && exist(OUT_DIR','dir')==7
    fprintf('%s already exists, aborting.\n', OUT_DIR );
    return
end

%-----------------------------
%%%%% Make OUT_DIR
%-----------------------------
fprintf('Creating %s\n', OUT_DIR );
mkdir( OUT_DIR );

%% Run WASS prepare
%-----------------------------

if DO_PREPARE==1

    fprintf('***************************************************\n');
    fprintf('**  RUNNING wass_prepare                      *****\n');
    fprintf('***************************************************\n');

    tic;

    %-----------------------------
    %%%%% Prepare files and folders in the OUT_DIR
    %-----------------------------
    % copy intr. param. in each folder + png file of undistorted images

    for ii=1:numel(input_frames)
        assert( system( [ENV_SET, PREPARE_EXE, ' --workdir ', input_frames{ii}.wd, ' --calibdir ', CONFIG_DIR, ...
                ' --c0 ', input_frames{ii}.Cam0, ' --c1 ', input_frames{ii}.Cam1] ) == 0, 'component exited with non-zero return code');
    end

    fprintf('***************************************************\n');
    fprintf(' Done in %f secs.\n', toc );

else
    fprintf("Skipping wass_prepare step\n");
end

%% Run WASS match
%-----------------------------

% WASS match is used to estimate the extrinsic parameters of the stereo
% cameras

if DO_MATCH==1

    fprintf('***************************************************\n');
    fprintf('**  RUNNING wass_match                        *****\n');
    fprintf('***************************************************\n');

    tic;

    frames_to_match=1:numel(input_frames);
    if MAX_FRAMES_TO_MATCH<numel(frames_to_match)
        frames_to_match = randperm(numel(input_frames));
        frames_to_match = frames_to_match(1:MAX_FRAMES_TO_MATCH);
        frames_to_match = sort(frames_to_match);
    end

    for jj=1:numel(frames_to_match)
        ii=frames_to_match(jj);
        assert( system( [ENV_SET, MATCH_EXE, ' ', CONFIG_DIR, 'matcher_config.txt ', input_frames{ii}.wd] ) == 0, 'component exited with non-zero return code');
    end

    fprintf('***************************************************\n');
    fprintf(' Done in %f secs.\n', toc );

else
    fprintf("Skipping wass_match step\n");
end

%-----------------------------
%%%%% check (not necessary for real data..the number of matches can be small, around 200 per pair)
%-----------------------------
%status = verify_matcher( input_frames, CONFIG_DIR );
%assert( strcmp(status, 'ok' ), ['wass_match failed: ',status]);

%% Run WASS autocalibrate
%-----------------------------

if DO_AUTOCALIBRATE==1
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

else
    fprintf("Skipping wass_autocalibrate step\n");
end

%% Run WASS stereo
%-----------------------------

if DO_STEREO
    fprintf('***************************************************\n');
    fprintf('**  RUNNING wass_stereo                       *****\n');
    fprintf('***************************************************\n');

    tic;

    for ii=1:numel(input_frames)
        assert( system( [ENV_SET, STEREO_EXE, ' ', CONFIG_DIR, 'stereo_config.txt ', input_frames{ii}.wd] ) == 0, 'component exited with non-zero return code');

        if ii>10
            delete([input_frames{ii}.wd,'/mesh.ply'])
            delete([input_frames{ii}.wd,'/plane_refinement_inliers.xyz'])
            delete([input_frames{ii}.wd,'/00000000_features.png'])
            delete([input_frames{ii}.wd,'/00000001_features.png'])
            delete([input_frames{ii}.wd,'/stereo.jpg'])
            delete([input_frames{ii}.wd,'/stereo_input.jpg'])
            %delete([input_frames{ii}.wd,'/undistorted'])
        end
    end

    fprintf('***************************************************\n');
    fprintf(' Done in %f secs.\n', toc );
else
    fprintf("Skipping wass_stereo step\n");
end


%% Collect all planes (for further processing)
%-----------------------------

fprintf('***************************************************\n');
fprintf('**  Gathering plane data                      *****\n');
fprintf('***************************************************\n');

tic;

planes = [];
fprintf('...  0%%\n');
for ii=1:numel(input_frames)
    try
        planefile = [input_frames{ii}.wd, '/plane.txt'];
        pl = importdata( planefile );
        planes = [ planes ; pl' ];

        if mod(ii,20)==0
            fprintf('...%3d%%\n',uint32(ii*100/numel(input_frames)));
        end

    catch err
        fprintf('WARN: %s has no plane file!\n',input_frames{ii}.wd);
    end
end
fprintf('...100%%\n');

%writematrix(planes,[OUT_DIR,'planes.txt'],'Delimiter', ' ' );
dlmwrite([OUT_DIR,'planes.txt'],planes,'Delimiter', ' ' );

fprintf('***************************************************\n');
fprintf(' Done in %f secs.\n', toc );


