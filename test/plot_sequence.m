%%
clc;clear;close all;
addpath([pwd(),'/../matlab/']);

outdir = [pwd(),'/output_W07/'];
framesdir =  [pwd(),'/frames/'];
mkdir( framesdir );

%%

for iidx = 0:100
    
    workdir = sprintf('%s/%06d_wd', outdir, iidx );
    workdirzip = [workdir,'.zip'];
    
    was_unzipped = 0;
        
    if exist(workdir','dir') ~= 7
            
        if exist(workdirzip','file')==2
            prev = cd(outdir);
            unzip(workdirzip);
            cd(prev);
            was_unzipped = 1;
        else
            fprintf( '%s does not exist, exiting\n', workdir);
            break;
        end
    end
    
    fprintf( '%s\n',workdir);

    prev = cd(workdir);
    mesh = load_camera_mesh();
    plane = importdata('plane.txt');
    P1 = importdata('P1cam.txt');
    img = imread('undistorted/00000001.png');
    cd(prev);

    %%

    Npts = size(mesh,2);
    elevations = dot( [mesh;ones(1,size(mesh,2))], repmat(plane,1,Npts) );
    % fix scale and z direction
    elevations = -elevations * 2.50;

    %%
    % project
    
    pt2d = P1 * [mesh;ones(1,size(mesh,2))];
    pt2d = pt2d ./ repmat( pt2d(3,:),3,1);

    %%
    
    close all;
    iptsetpref('ImshowBorder','loose');
    imgrgb = repmat(img,[1,1,3]);

    hhfig = figure;
    imshow(imgrgb);
    hold on;
    hs = scatter( pt2d(1,:), pt2d(2,:),1,elevations,'.');
    hs.MarkerEdgeAlpha=0.05;
    caxis( [-1.3 1.3] );
    title(sprintf('Frame %06d - %07d points      http://www.dais.unive.it/wass', iidx, Npts) );
    c = colorbar;
    ylabel(c,'Elevation (m)');

    axis ij;
    colormap jet;
   

    crop = 20;
    hhfig.Position = [0 0 1440+crop*2 1080+crop*2];
    drawnow;
    ii = getframe(hhfig);
    ii = ii.cdata;
    ii = ii( crop-10:(end-crop-10), crop+5:(end-crop-5), : );
    close all;

    imwrite(ii,sprintf('%s/frm%06d.png',framesdir,iidx));

    if was_unzipped
        rmdir( workdir, 's' );
    end
end
