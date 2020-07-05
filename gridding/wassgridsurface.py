import argparse
import configparser
import scipy.io as sio
import scipy.interpolate
import os
import sys
import glob
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from os import path
from tqdm import tqdm

import colorama
colorama.init()
from colorama import Fore, Back, Style

from wass_utils import load_camera_mesh, align_on_sea_plane, align_on_sea_plane_RT, compute_sea_plane_RT, filter_mesh_outliers

WASSGRIDSURFACE_VERSION = "0.1"



def setup( wdir, meanplane, baseline, outdir, area_center, area_size, N, Iw=None, Ih=None ):
    meshname = path.join(wdir,"mesh_cam.xyzC")
    print("Loading ", meshname )

    R = np.loadtxt(path.join(wdir,'Cam0_poseR.txt'))
    T = np.loadtxt(path.join(wdir,'Cam0_poseT.txt'))
    P0Cam =  np.vstack( (np.loadtxt( path.join(wdir,"P0cam.txt"))  ,[0, 0, 0, 1] ) )

    
    if (Iw is None or Ih is None) and path.exists( path.join(wdir,"undistorted","00000000.png") ):
        I = cv.imread(  path.join(wdir,"undistorted","00000000.png"), cv.IMREAD_ANYCOLOR )
        Iw,Ih = I.shape[1],I.shape[0]

    if Iw is None or Ih is None:
        print("Unable to determine the camera image size. Please set it manually with -Iw,-Ih program arguments")
        sys.exit(-3)

    print("Camera image size: "+Fore.RED+("%dx%d"%(Iw,Ih))+Fore.RESET+" (override with -Iw -Ih program arguments)")

    Rpl, Tpl = compute_sea_plane_RT( meanplane )
    mesh = load_camera_mesh(meshname)
    mesh_aligned = align_on_sea_plane( mesh, meanplane) * baseline 

    Ri = Rpl.T
    Ti = -Rpl.T@Tpl
    RTplane = np.vstack( (np.hstack( (Ri,Ti) ),[0,0,0,1]) )

    toNorm = np.array( [[ 2.0/Iw, 0     , -1, 0],
                        [ 0     , 2.0/Ih, -1, 0],
                        [ 0,      0,       1, 0],
                        [ 0,      0,       0, 1]], dtype=np.float )

    SCALEi = 1.0/baseline
    P0plane = toNorm @ P0Cam @ RTplane @ np.diag((SCALEi,SCALEi,-SCALEi, 1))

    area_size_m = np.floor( area_size / 2);
    xmin = area_center[0]-area_size_m;
    xmax = area_center[0]+area_size_m;
    ymin = area_center[1]-area_size_m;
    ymax = area_center[1]+area_size_m;
    zmax = np.quantile( mesh_aligned[2,:],0.98 )*1.5 
    zmin = np.quantile( mesh_aligned[2,:],0.02 )*1.5

    # Let zmin/zmax be symmetric around 0
    if zmax>zmin:
        zmin=-zmax
    else:
        zmax=-zmin

    # Meshgrid
    XX,YY = np.meshgrid( np.linspace(xmin,xmax,N), np.linspace(ymin,ymax,N) );
    x_spacing = XX[0,1]-XX[0,0];
    y_spacing = YY[1,0]-YY[0,0];
    assert( abs(x_spacing - y_spacing) < 1E-5 );

    Nm = int( N/2 )

    kx_ab = np.array( [float(i)/N*(2.0*np.pi/x_spacing)  for i in range(-Nm,Nm)] );
    ky_ab = np.array( [float(i)/N*(2*np.pi/y_spacing)  for i in range(-Nm,Nm)] );
    KX_ab, KY_ab = np.meshgrid( kx_ab, ky_ab);
    spec_scale = 1.0/(N*N);

    print("Generating grid area plot...")

    fig = plt.figure( figsize=(20,20))
    plt.scatter( mesh_aligned[0,::50], mesh_aligned[1,::50], c=mesh_aligned[2,::50], vmin=zmin, vmax=zmax )
    plt.gca().invert_yaxis()
    plt.colorbar()
    plt.plot( np.array([xmin, xmax, xmax, xmin, xmin]), np.array([ymin,ymin,ymax,ymax,ymin]), '-k', linewidth=2 );
    plt.scatter( XX.flatten(), YY.flatten(), c="k", s=0.1, marker=".")
    plt.axis("equal")
    plt.title("WASS point cloud %s"%wdir )
    plt.grid("minor")
    figfile = path.join(outdir,"area_grid.png")
    fig.savefig(figfile,bbox_inches='tight')
    plt.close()
    print("Please open "+Fore.RED+figfile+Fore.RESET+" and check if everything is ok" )

    matfile = path.join(outdir,"config.mat")
    print("Saving grid setup to "+Fore.RED+matfile+Fore.RESET )
    sio.savemat( matfile, {
        "xmin":xmin,
        "xmax":xmax,
        "ymin":ymin,
        "ymax":ymax,
        "zmin":zmin,
        "zmax":zmax,
        "R":R,
        "T":T,
        "Rpl":Rpl,
        "Tpl":Tpl,
        "P0plane":P0plane,
        "CAM_BASELINE":baseline,
        "XX":XX,
        "YY":YY,
        "KX_ab":KX_ab,
        "KY_ab":KY_ab,
        "spec_scale":spec_scale,
        "x_spacing":x_spacing,
        "y_spacing":y_spacing
    } )



def grid( wass_frames, matfile, outdir ):
    gridsetup = sio.loadmat( matfile )
    #for wdir in wass_frames:
    #    print(wdir)
    wdir = wass_frames[0]

    meshname = path.join( wdir, "mesh_cam.xyzC")
    mesh = load_camera_mesh(meshname)
    mesh_aligned = align_on_sea_plane_RT( mesh, gridsetup["Rpl"], gridsetup["Tpl"]) * gridsetup["CAM_BASELINE"] 

    #print("Filtering mesh outliers...")
    #mesh_aligned = filter_mesh_outliers( mesh_aligned, debug=False )

    XX = gridsetup["XX"]
    YY = gridsetup["YY"]

    # fig = plt.figure( figsize=(20,20))
    # plt.scatter( mesh_aligned[0,::50], mesh_aligned[1,::50], c=mesh_aligned[2,::50], vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
    # plt.gca().invert_yaxis()
    # plt.colorbar()
    # plt.scatter( XX.flatten(), YY.flatten(), c="k", s=0.1, marker=".")
    # plt.axis("equal")
    # plt.title("WASS point cloud %s"%wdir )
    # plt.grid("minor")
    # figfile = path.join(outdir,"area_grid2.png")
    # fig.savefig(figfile,bbox_inches='tight')
    # plt.close()

    print("Interpolating...")
    interpolator = scipy.interpolate.LinearNDInterpolator( mesh_aligned[:2,:].T, mesh_aligned[2,:].T )
    Zi = interpolator( np.vstack( [XX.flatten(), YY.flatten() ]).T )
    Zi = np.reshape( Zi, XX.shape )
    print("Done!")

    fig = plt.figure( figsize=(20,20))
    plt.imshow(Zi)
    figfile = path.join(outdir,"area_interp.png")
    fig.savefig(figfile,bbox_inches='tight')
    plt.close()




def main():
    print("WASS surface gridder v.", WASSGRIDSURFACE_VERSION )
    print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n  Copyright (C) Filippo Bergamasco 2020 \n")

    parser = argparse.ArgumentParser()
    parser.add_argument("workdir", help="WASS output directory containing the reconstructed frames")
    parser.add_argument("outdir", help="Output directory")
    parser.add_argument("--action", type=str, help="What to do [setup | grid]" )
    parser.add_argument("--gridconfig", type=str, help="Grid configuration file for setup action")
    parser.add_argument("--gridsetup", type=str, help="Grid setup file for grid action")
    parser.add_argument("-b", "--baseline", default=1.0, type=float, help="Stereo camera distance" )
    parser.add_argument("-Iw", "--image_width", type=float, help="Camera frame width" )
    parser.add_argument("-Ih", "--image_height", type=float, help="Camera frame height" )
    args = parser.parse_args()


    print("Looking for WASS reconstructed stereo frames in ",args.workdir )
    wass_frames = glob.glob( path.join( args.workdir, "*_wd" ))
    wass_frames.sort()
    print("%d frames found."%len(wass_frames) )

    planefile = path.join( args.workdir, "planes.txt" )
    print("Looking planes definition file ",args.workdir )
    meanplane = None
    if path.exists( planefile ):
        print("Found! Loading...")
        all_planes = np.loadtxt( planefile )
        meanplane = np.mean( all_planes, axis=0)

    print("Baseline: "+Fore.RED+"%3.2f"%args.baseline + Fore.RESET)


    if not path.exists( args.outdir ):
        print("Invalid output dir: ", args.outdir )
        sys.exit(-1)

    if args.action == "setup":
        if args.gridconfig is None:
            print("--gridconfig option not specified, aborting.")
            sys.exit(-1)

        print("Loading grid configuration file: %s"%args.gridconfig )
        if not path.exists( args.gridconfig ):
            print("Not found.")
            sys.exit(-1)

        settings = configparser.ConfigParser()
        settings.read( args.gridconfig )
        
        setup( wass_frames[0], 
               meanplane, 
               baseline=args.baseline,
               outdir=args.outdir, 
               area_center=np.array([ settings.getfloat("Area","area_center_x"), settings.getfloat("Area","area_center_y")]),
               area_size=settings.getfloat("Area","area_size"),
               N = settings.getint("Area","N"),
               Iw=args.image_width,
               Ih=args.image_height )
        
    elif args.action == "grid":

        if args.gridsetup is None:
            print("Grid setup file not specified. See --gridsetup option")

        grid( wass_frames,
              matfile=args.gridsetup,
              outdir=args.outdir )
        
    else:
        print("Invalid actions specified.")
        sys.exit(-2)



if __name__ == "__main__":
    main()