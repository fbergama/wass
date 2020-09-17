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

from netcdfoutput import NetCDFOutput
from wass_utils import load_camera_mesh, align_on_sea_plane, align_on_sea_plane_RT, compute_sea_plane_RT, filter_mesh_outliers

WASSGRIDSURFACE_VERSION = "0.2"



def setup( wdir, meanplane, baseline, outdir, area_center, area_size, N, Iw=None, Ih=None, fps=0, timestring="" ):
    meshname = path.join(wdir,"mesh_cam.xyzC")
    print("Loading ", meshname )

    R = np.loadtxt(path.join(wdir,'Cam0_poseR.txt'))
    T = np.loadtxt(path.join(wdir,'Cam0_poseT.txt'))
    P0Cam =  np.vstack( (np.loadtxt( path.join(wdir,"P0cam.txt"))  ,[0, 0, 0, 1] ) )
    P1Cam =  np.vstack( (np.loadtxt( path.join(wdir,"P1cam.txt"))  ,[0, 0, 0, 1] ) )

    
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

    area_size_m = np.floor( area_size / 2)
    xmin = area_center[0]-area_size_m
    xmax = area_center[0]+area_size_m
    ymin = area_center[1]-area_size_m
    ymax = area_center[1]+area_size_m
    zmax = np.quantile( mesh_aligned[2,:],0.98 )*1.5 
    zmin = np.quantile( mesh_aligned[2,:],0.02 )*1.5

    # Let zmin/zmax be symmetric around 0
    if np.abs(zmax)>np.abs(zmin):
        zmin=-zmax
    else:
        zmax=-zmin

    print("zmin .. zmax = %3.2f ... %3.2f"%(zmin,zmax) )

    # Meshgrid
    XX,YY = np.meshgrid( np.linspace(xmin,xmax,N), np.linspace(ymin,ymax,N) )
    x_spacing = XX[0,1]-XX[0,0]
    y_spacing = YY[1,0]-YY[0,0]
    assert( abs(x_spacing - y_spacing) < 1E-5 )

    Nm = int( N/2 )

    kx_ab = np.array( [float(i)/N*(2.0*np.pi/x_spacing)  for i in range(-Nm,Nm)] )
    ky_ab = np.array( [float(i)/N*(2*np.pi/y_spacing)  for i in range(-Nm,Nm)] )
    KX_ab, KY_ab = np.meshgrid( kx_ab, ky_ab)
    spec_scale = 1.0/(N*N)

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
        "P0cam":P0Cam[0:3,:],
        "P1cam":P1Cam[0:3,:],
        "N":N,
        "R":R,
        "T":T,
        "Rpl":Rpl,
        "Tpl":Tpl,
        "P0plane":P0plane,
        "CAM_BASELINE":baseline,
        "scale":baseline,
        "XX":XX,
        "YY":YY,
        "KX_ab":KX_ab,
        "KY_ab":KY_ab,
        "spec_scale":spec_scale,
        "x_spacing":x_spacing,
        "y_spacing":y_spacing,
        "fps":fps,
        "timestring":timestring
    } )



def grid( wass_frames, matfile, outdir, subsample_percent = 100 ):
    step=150
    gridsetup = sio.loadmat( matfile )
    XX = gridsetup["XX"]
    YY = gridsetup["YY"]

    outdata = NetCDFOutput( filename=path.join(outdir,"gridded.nc" ) )
    baseline = gridsetup["CAM_BASELINE"].item(0)

    fps = gridsetup["fps"].item(0) 

    outdata.scale[:] = baseline
    outdata.add_meta_attribute("info", "Generated with WASS gridder v.%s"%WASSGRIDSURFACE_VERSION )
    outdata.add_meta_attribute("generator", "WASS" )
    outdata.add_meta_attribute("baseline", baseline )
    outdata.add_meta_attribute("fps", fps)
    outdata.add_meta_attribute("timestring", gridsetup["timestring"] )
    outdata.set_grids( XX*1000.0, YY*1000.0 )
    outdata.set_kxky( gridsetup["KX_ab"], gridsetup["KY_ab"] )

    outdata.set_instrinsics( np.zeros( (3,3), dtype=np.float32),
                             np.zeros( (3,3), dtype=np.float32),
                             np.zeros( (5,1), dtype=np.float32),
                             np.zeros( (5,1), dtype=np.float32),
                             gridsetup["P0plane"])

    Zmean = 0.0
    Zmin = np.Inf
    Zmax = -np.Inf
    N_frames = 1

    for wdir in tqdm(wass_frames):
        tqdm.write(wdir)
        dirname = path.split( wdir )[-1]
        FRAME_IDX = int(dirname[:-3])

        meshname = path.join( wdir, "mesh_cam.xyzC")
        mesh = load_camera_mesh(meshname)
        mesh_aligned = align_on_sea_plane_RT( mesh, gridsetup["Rpl"], gridsetup["Tpl"]) * gridsetup["CAM_BASELINE"] 
        mesh_aligned = mesh_aligned[:, np.random.permutation(mesh_aligned.shape[1]) ]

        # # 3D point grid quantization
        # scalefacx = (gridsetup["xmax"]-gridsetup["xmin"])
        # scalefacy = (gridsetup["ymax"]-gridsetup["ymin"])
        # pts_x = np.floor( (mesh_aligned[0,:]-gridsetup["xmin"])/scalefacx * (XX.shape[1]-1) + 0.5 ).astype(np.uint32).flatten()
        # pts_y = np.floor( (mesh_aligned[1,:]-gridsetup["ymin"])/scalefacy * (XX.shape[0]-1) + 0.5 ).astype(np.uint32).flatten()
        # good_pts = np.logical_and( np.logical_and( pts_x >= 0 , pts_x < XX.shape[1] ),
        #                            np.logical_and( pts_y >= 0 , pts_y < XX.shape[0] ) )

        # ZZ = np.ones( XX.shape, dtype=np.float32 )*np.nan
        # pts_x = pts_x[good_pts]
        # pts_y = pts_y[good_pts]
        # pts_z = mesh_aligned[2,good_pts]
        # ZZ[ pts_y, pts_x ] = pts_z

        #aux = ((ZZ-gridsetup["zmin"])/(gridsetup["zmax"]-gridsetup["zmin"])*255).astype(np.uint8) 
        #cv.imwrite( path.join(outdir,"area_interp2.png"), cv.resize(aux,(800,800), interpolation=cv.INTER_NEAREST ) )
        #sys.exit(0)

        # fig = plt.figure( figsize=(20,20))
        # plt.imshow( ZZ, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
        # figfile = path.join(outdir,"area_interp2.png")
        # fig.savefig(figfile,bbox_inches='tight')
        # plt.close()

        #tqdm.write("Filtering mesh outliers...")
        #mesh_aligned = filter_mesh_outliers( mesh_aligned, debug=False )

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

        area_mask =  np.logical_and( np.logical_and( mesh_aligned[0,:]>=gridsetup["xmin"] , mesh_aligned[0,:]<=gridsetup["xmax"] ),
                                     np.logical_and( mesh_aligned[1,:]>=gridsetup["ymin"] , mesh_aligned[1,:]<=gridsetup["ymax"] ) ) 


        pt_indices =  np.nonzero(area_mask)[1]
        tqdm.write("%d points in the area"%pt_indices.size )
        np.random.shuffle( pt_indices )
        pt_indices = pt_indices[ :int(pt_indices.size*subsample_percent/100.0)]
        tqdm.write("%d points after subsampling (%d%%)"%(pt_indices.size, subsample_percent) )

        tqdm.write("Interpolating... ", end="")
        interpolator = scipy.interpolate.LinearNDInterpolator( mesh_aligned[:2,pt_indices].T, mesh_aligned[2,pt_indices].T )
        Zi = interpolator( np.vstack( [XX.flatten(), YY.flatten() ]).T )
        Zi = np.reshape( Zi, XX.shape )
        tqdm.write("done")

        Zmean = Zmean + (np.nanmean(Zi)-Zmean)/N_frames
        Zmin = min( Zmin, np.nanmin(Zi) )
        Zmax = max( Zmax, np.nanmax(Zi) )

        I0 = cv.imread( path.join(wdir,"00000000_s.png"))
        outdata.add_meta_attribute("image_width", I0.shape[1] )
        outdata.add_meta_attribute("image_height", I0.shape[0] )
        ret, imgjpeg = cv.imencode(".jpg", I0 )
        outdata.push_Z( Zi*1000, (N_frames-1)/fps if fps>0 else 0, FRAME_IDX, imgjpeg )

        #aux = ((Zi-gridsetup["zmin"])/(gridsetup["zmax"]-gridsetup["zmin"])*255).astype(np.uint8) 
        #cv.imwrite( path.join(outdir,"area_interp.png"), cv.resize(aux,(800,800), interpolation=cv.INTER_NEAREST ) )

        # fig = plt.figure( figsize=(20,20))
        # plt.imshow(Zi, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
        # figfile = path.join(outdir,"area_interp_%08d.png"%FRAME_IDX)
        # fig.savefig(figfile,bbox_inches='tight')
        # plt.close()

        N_frames += 1


    outdata.add_meta_attribute("zmin", Zmin )
    outdata.add_meta_attribute("zmax", Zmax )
    outdata.add_meta_attribute("zmean", Zmean )

    print("Reconstructed sequence stats: ")
    print("    Zmin: ",Zmin)
    print("    Zmax: ",Zmax)
    print("   Zmean: ",Zmean)
    print("# frames: ",N_frames)

    outdata.close()





def main():
    print("WASS surface gridder v.", WASSGRIDSURFACE_VERSION )
    print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n  Copyright (C) Filippo Bergamasco 2020 \n")

    parser = argparse.ArgumentParser()
    parser.add_argument("workdir", help="WASS output directory containing the reconstructed frames")
    parser.add_argument("outdir", help="Output directory")
    parser.add_argument("--action", type=str, help="What to do [setup | grid | generategridconfig ]" )
    parser.add_argument("--gridconfig", type=str, help="Grid configuration file for setup action")
    parser.add_argument("--gridsetup", type=str, help="Grid setup file for grid action")
    parser.add_argument("-b", "--baseline", default=1.0, type=float, help="Stereo camera distance" )
    parser.add_argument("-f", "--fps", default=0, type=float, help="Sequence frames per second" )
    parser.add_argument("-t", "--timestring", default="", type=str, help="Sequence datetime string (ie. the RAW filename)" )
    parser.add_argument("-Iw", "--image_width", type=float, help="Camera frame width" )
    parser.add_argument("-Ih", "--image_height", type=float, help="Camera frame height" )
    parser.add_argument("--ss", "--subsample_percent", type=float, default=100, help="Point subsampling 0..100%" )
    args = parser.parse_args()


    if args.action == "generategridconfig":
        gridconfigfile = path.join(args.outdir,"gridconfig.txt")  
        print("Generating ",gridconfigfile)
        with open(gridconfigfile, "w" ) as f:
            f.write("[Area]\n")
            f.write("area_center_x=0.0\n")
            f.write("area_center_y=-35.0\n")
            f.write("area_size=50\n")
            f.write("N=256\n")

        print("All done, exiting.")
        return


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

        print("Baseline: "+Fore.RED+"%3.2f"%args.baseline + Fore.RESET)

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
               Ih=args.image_height,
               fps=args.fps,
               timestring=args.timestring )
        
    elif args.action == "grid":

        if args.gridsetup is None:
            print("Grid setup file not specified. See --gridsetup option")

        grid( wass_frames,
              matfile=args.gridsetup,
              outdir=args.outdir,
              subsample_percent=args.ss )

        print("Gridding completed.")
        
    else:
        print("Invalid actions specified.")
        sys.exit(-2)



if __name__ == "__main__":
    main()