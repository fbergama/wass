"""
wassgridsurface
Copyright (C) 2023 Filippo Bergamasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""


VERSION = "0.6.0"


import argparse
import configparser
import scipy.io as sio
import scipy.interpolate
import os
import sys
import glob
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
from os import path
from tqdm import tqdm
from tqdm.contrib.concurrent import thread_map


import colorama
colorama.init()
from colorama import Fore, Back, Style

from wassgridsurface.netcdfoutput import NetCDFOutput
from wassgridsurface.wass_utils import load_camera_mesh, align_on_sea_plane, align_on_sea_plane_RT, compute_sea_plane_RT, filter_mesh_outliers

#from TFVariationalRefinement import TFVariationalRefinement
from wassgridsurface.IDWInterpolator import IDWInterpolator
from wassgridsurface.DCTInterpolator import DCTInterpolator




def setup( wdir, meanplane, baseline, outdir, area_center, area_size_x, area_size_y, Nx, Ny, Iw=None, Ih=None, fps=0, timestring="" ):
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
                        [ 0,      0,       0, 1]], dtype=float )

    SCALEi = 1.0/baseline
    P0plane = toNorm @ P0Cam @ RTplane @ np.diag((SCALEi,SCALEi,-SCALEi, 1))

    area_size_m_x = np.floor( area_size_x / 2)
    area_size_m_y = np.floor( area_size_y / 2)
    xmin = area_center[0]-area_size_m_x
    xmax = area_center[0]+area_size_m_x
    ymin = area_center[1]-area_size_m_y
    ymax = area_center[1]+area_size_m_y
    zmax = np.quantile( mesh_aligned[2,:],0.98 )*1.5
    zmin = np.quantile( mesh_aligned[2,:],0.02 )*1.5

    # Let zmin/zmax be symmetric around 0
    if np.abs(zmax)>np.abs(zmin):
        zmin=-zmax
    else:
        zmax=-zmin

    print("zmin .. zmax = %3.2f ... %3.2f"%(zmin,zmax) )

    # Meshgrid
    XX,YY = np.meshgrid( np.linspace(xmin,xmax,Nx), np.linspace(ymin,ymax,Ny) )
    x_spacing = XX[0,1]-XX[0,0]
    y_spacing = YY[1,0]-YY[0,0]
    print(x_spacing)
    print(y_spacing)
    assert( abs(x_spacing - y_spacing) < 1E-2 )

    Nmx = Nx//2
    Nmy = Ny//2

    kx_ab = np.array( [float(i)/Nx*(2.0*np.pi/x_spacing)  for i in range(-Nmx,Nmx)] )
    ky_ab = np.array( [float(i)/Ny*(2*np.pi/y_spacing)  for i in range(-Nmy,Nmy)] )
    KX_ab, KY_ab = np.meshgrid( kx_ab, ky_ab)
    spec_scale = 1.0/(Nx*Ny)

    print("Generating grid area plot...")

    fig = plt.figure( figsize=(20,20))
    plt.scatter( mesh_aligned[0,::50], mesh_aligned[1,::50], c=mesh_aligned[2,::50], vmin=zmin, vmax=zmax )
    plt.gca().invert_yaxis()
    plt.colorbar()
    plt.plot( np.array([xmin, xmax, xmax, xmin, xmin]), np.array([ymin,ymin,ymax,ymax,ymin]), '-k', linewidth=2 )
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
        "Nx":Nx,
        "Ny":Ny,
        "N":np.amax([Nx,Ny]),
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



def grid( wass_frames, matfile, outdir, subsample_percent=100, mf=0, algorithm="DCT", user_mask_filename=None, alg_options=None, NUM_PARALLEL_PROCESSES=1 ):
    step=150
    gridsetup = sio.loadmat( matfile )
    XX = gridsetup["XX"]
    YY = gridsetup["YY"]

    Rpl = gridsetup["Rpl"]
    Tpl = gridsetup["Tpl"]
    Rp2c = Rpl.T
    Tp2c = -Rp2c@Tpl

    P0cam = gridsetup["P0cam"]
    P1cam = gridsetup["P1cam"]

    outdata = NetCDFOutput( filename=path.join(outdir,"gridded.nc" ), M=XX.shape[0], N=XX.shape[1] )
    baseline = gridsetup["CAM_BASELINE"].item(0)

    fps = gridsetup["fps"].item(0)

    outdata.scale[:] = baseline
    outdata.add_meta_attribute("info", "Generated with WASS gridder v.%s"%VERSION )
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


    wass_frames_with_indices = [ x for x in enumerate(wass_frames) ]
    N_frames = len( wass_frames_with_indices )


    user_mask = np.ones( XX.shape, dtype=np.float32 )

    if not user_mask_filename is None:
        print("Loading %s"%user_mask_filename )
        user_mask = cv.imread( user_mask_filename, cv.IMREAD_GRAYSCALE )
        user_mask = (user_mask>0).astype(np.float32)


    print("Interpolation algorithm: "+Fore.RED+algorithm+Fore.RESET )
    print("Using %d thread(s) for reconstruction..."%NUM_PARALLEL_PROCESSES )


    # Create a dedicated interpolator for each parallel process
    interpolators = [ IDWInterpolator( KSIZE=5, reps=1 ) if algorithm=="IDW" else DCTInterpolator( img_width=XX.shape[1], img_height=XX.shape[0], alg_options=alg_options ) for x in range(NUM_PARALLEL_PROCESSES) ]

    Zmeans = []
    Zmins = []
    Zmaxs = []

    def _grid_task( iteration_element ):

        idx, wdir = iteration_element
        interpolator = interpolators[ idx % NUM_PARALLEL_PROCESSES ]

        tqdm.write(wdir)
        dirname = path.split( wdir )[-1]
        FRAME_IDX = int(dirname[:-3])

        if idx<=NUM_PARALLEL_PROCESSES:
            time.sleep( idx ) # wait to optimize disk access

        meshname = path.join( wdir, "mesh_cam.xyzC")
        mesh = load_camera_mesh(meshname)
        mesh_aligned = align_on_sea_plane_RT( mesh, gridsetup["Rpl"], gridsetup["Tpl"]) * gridsetup["CAM_BASELINE"]
        mesh_aligned = mesh_aligned[:, np.random.permutation(mesh_aligned.shape[1]) ]

        # 3D point grid quantization
        scalefacx = (gridsetup["xmax"]-gridsetup["xmin"])
        scalefacy = (gridsetup["ymax"]-gridsetup["ymin"])
        pts_x = np.floor( (mesh_aligned[0,:]-gridsetup["xmin"])/scalefacx * (XX.shape[1]-1) + 0.5 ).astype(np.uint32).flatten()
        pts_y = np.floor( (mesh_aligned[1,:]-gridsetup["ymin"])/scalefacy * (XX.shape[0]-1) + 0.5 ).astype(np.uint32).flatten()
        good_pts = np.logical_and( np.logical_and( pts_x >= 0 , pts_x < XX.shape[1] ),
                                   np.logical_and( pts_y >= 0 , pts_y < XX.shape[0] ) )

        if algorithm=="IDW" or algorithm=="DCT":

            NREPS=10
            ZZ = np.ones( [XX.shape[0], XX.shape[1], NREPS], dtype=np.float32 )*np.nan

            pts_x = pts_x[good_pts]
            pts_y = pts_y[good_pts]
            pts_z = mesh_aligned[2,good_pts]

            indices = np.arange( pts_x.shape[0] )
            n_pts = int( pts_x.shape[0]*subsample_percent//100 )

            for ii in range(NREPS):
                np.random.shuffle( indices )
                curr_indices = np.copy( indices[:n_pts] )
                ZZ[ pts_y[indices[curr_indices]], pts_x[indices[curr_indices]], ii ] = pts_z[indices[curr_indices]]

            ZZ = np.nanmedian( ZZ, axis=-1 )

            if idx == 0:
                fig = plt.figure( figsize=(20,20))
                plt.imshow( ZZ, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
                figfile = path.join(outdir,"points.png" )
                fig.savefig(figfile,bbox_inches='tight')
                plt.close()

            Zi, mask = interpolator(ZZ, verbose=(NUM_PARALLEL_PROCESSES==1) )
            mask *= user_mask
            Zi[ mask==0 ] = np.nan

            if mf>0:
                Zi = Zi.astype(np.float32)
                Zi[ mask==0 ] = 0
                Zi = cv.medianBlur(Zi, ksize=mf)
                Zi[ mask==0 ] = np.nan

            if idx == 0:
                fig = plt.figure( figsize=(20,20))
                plt.imshow( Zi, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
                figfile = path.join(outdir,"gridded.png" )
                fig.savefig(figfile,bbox_inches='tight')
                plt.close()

                zmin, zmax = np.nanmin(Zi), np.nanmax(Zi)
                Zi_img = (Zi-zmin)/(zmax-zmin)*255
                Zi_img[ np.isnan( Zi_img ) ] = 0
                cv.imwrite( path.join(outdir,"grid_img.png"), Zi_img.astype(np.uint8) )
                del zmin, zmax, Zi_img


            # I0 = cv.imread( path.join(wdir,"undistorted/00000000.png"))
            # I1 = cv.imread( path.join(wdir,"undistorted/00000001.png"))

            # tfvr = TFVariationalRefinement( I0[...,0], I1[...,0], Rp2c=Rp2c, Tp2c=Tp2c, P0cam=P0cam, P1cam=P1cam, XX=XX, YY=YY, baseline=baseline, mask=mask )

            # # print("-ZZ")
            # # print( tfvr.compute_loss(-ZZ/baseline) )
            # # I0_samp, I1_samp, _, _ = tfvr.sample_images( -ZZ/baseline )

            # # cv.imwrite( path.join(outdir, "00_I0.png"), I0_samp.numpy() )
            # # cv.imwrite( path.join(outdir, "00_I1.png"), I1_samp.numpy() )


            # I0_samp, I1_samp, _, _ = tfvr.sample_images( -ZZ/baseline )
            # cv.imwrite( path.join(outdir, "01_I0.png"), I0_samp.numpy() )
            # cv.imwrite( path.join(outdir, "01_I1.png"), I1_samp.numpy() )

            # fig = plt.figure( figsize=(20,20))
            # plt.imshow( np.abs( (I0_samp-I1_samp).numpy()), vmin=0, vmax=80  )
            # plt.colorbar()
            # figfile = path.join(outdir,"Zinit_error.png")
            # fig.savefig(figfile,bbox_inches='tight')
            # plt.close()

            # fig = plt.figure( figsize=(20,20))
            # plt.imshow( ZZ, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
            # figfile = path.join(outdir,"Zinit.png" )
            # fig.savefig(figfile,bbox_inches='tight')
            # plt.close()

            # Z_opt = tfvr.optimize( -ZZ/baseline )
            # I0_samp, I1_samp, _, _ = tfvr.sample_images( Z_opt )
            # cv.imwrite( path.join(outdir, "02_opt_I0.png"), I0_samp.numpy() )
            # cv.imwrite( path.join(outdir, "02_opt_I1.png"), I1_samp.numpy() )
            # Z_opt = -Z_opt*baseline
            # fig = plt.figure( figsize=(20,20))
            # plt.imshow( np.abs( (I0_samp-I1_samp).numpy()), vmin=0, vmax=80 )
            # plt.colorbar()
            # figfile = path.join(outdir,"Zopt_error.png" )
            # fig.savefig(figfile,bbox_inches='tight')
            # plt.close()


            # # Z_dx, Z_dy = tfvr.compute_Z_gradient( -ZZ/baseline )
            # # fig = plt.figure( figsize=(20,20))
            # # plt.imshow( Z_dy.numpy() )
            # # plt.colorbar()
            # # plt.title("Z_dy")
            # # fig.savefig( path.join(outdir,"Z_dy"), bbox_inches='tight')


            # fig = plt.figure( figsize=(20,20))
            # plt.imshow( Z_opt, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
            # figfile = path.join(outdir,"Zopt.png" )
            # fig.savefig(figfile,bbox_inches='tight')
            # plt.close()
            # return

        elif algorithm=="LinearND":

            #np.savez( path.join(outdir,"pts_%06d"%FRAME_IDX), pts_x=pts_x, pts_y=pts_y, pts_z=pts_z, ZZ=ZZ )
            #gridlimits = np.array( [gridsetup["xmin"],gridsetup["xmax"],gridsetup["ymin"],gridsetup["ymax"]], dtype=np.float32 )
            #np.savez( path.join(outdir,"pts_raw_%06d"%FRAME_IDX), mesh_aligned=mesh_aligned, gridlimits=gridlimits )

            #aux = ((ZZ-gridsetup["zmin"])/(gridsetup["zmax"]-gridsetup["zmin"])*255).astype(np.uint8)
            #cv.imwrite( path.join(outdir,"area_interp2.png"), cv.resize(aux,(800,800), interpolation=cv.INTER_NEAREST ) )
            #sys.exit(0)

            # fig = plt.figure( figsize=(20,20))
            # plt.imshow( ZZ, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
            # figfile = path.join(outdir,"area_interp2.png")
            # fig.savefig(figfile,bbox_inches='tight')
            # plt.close()

            # #tqdm.write("Filtering mesh outliers...")
            # #mesh_aligned = filter_mesh_outliers( mesh_aligned, debug=False )

            # # fig = plt.figure( figsize=(20,20))
            # # plt.scatter( mesh_aligned[0,::50], mesh_aligned[1,::50], c=mesh_aligned[2,::50], vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
            # # plt.gca().invert_yaxis()
            # # plt.colorbar()
            # # plt.scatter( XX.flatten(), YY.flatten(), c="k", s=0.1, marker=".")
            # # plt.axis("equal")
            # # plt.title("WASS point cloud %s"%wdir )
            # # plt.grid("minor")
            # # figfile = path.join(outdir,"area_grid2.png")
            # # fig.savefig(figfile,bbox_inches='tight')
            # # plt.close()

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
        else:
            print("Invalid interpolation algorithm, aborting")
            return


        # Zi now contains the interpolated surface

        Zmeans.append( np.nanmean(Zi) )
        Zmins.append( np.nanmin(Zi) )
        Zmaxs.append( np.nanmax(Zi) )


        I0 = cv.imread( path.join(wdir,"00000000_s.png"))
        outdata.add_meta_attribute("image_width", I0.shape[1] )
        outdata.add_meta_attribute("image_height", I0.shape[0] )
        ret, imgjpeg = cv.imencode(".jpg", I0 )

        mask_filename = path.join( wdir, "undistorted", "mask0.png" ) 
        imagemask = None
        if path.exists( mask_filename ): 
            with open( mask_filename, "rb" ) as f:
                imagemask = np.fromfile(f, np.uint8 );

        outdata.push_Z( Zi*1000, float(idx)/float(fps) if fps>0 else 0.0, FRAME_IDX, imgjpeg, imagemask, idx=idx )


        #aux = ((Zi-gridsetup["zmin"])/(gridsetup["zmax"]-gridsetup["zmin"])*255).astype(np.uint8)
        #cv.imwrite( path.join(outdir,"area_interp.png"), cv.resize(aux,(800,800), interpolation=cv.INTER_NEAREST ) )

        # fig = plt.figure( figsize=(20,20))
        # plt.imshow(Zi, vmin=gridsetup["zmin"], vmax=gridsetup["zmax"] )
        # figfile = path.join(outdir,"area_interp_%08d.png"%FRAME_IDX)
        # fig.savefig(figfile,bbox_inches='tight')
        # plt.close()

    #------
    r = thread_map(_grid_task, wass_frames_with_indices, max_workers=NUM_PARALLEL_PROCESSES )



    Zmin = np.amin( np.array(Zmins)) 
    Zmax = np.amax( np.array(Zmaxs)) 
    Zmean = np.mean( np.array(Zmeans)) 
    outdata.add_meta_attribute("zmin", Zmin )
    outdata.add_meta_attribute("zmax", Zmax )
    outdata.add_meta_attribute("zmean", Zmean )

    print("Reconstructed sequence stats: ")
    print("    Zmin: ",Zmin)
    print("    Zmax: ",Zmax)
    print("   Zmean: ",Zmean)
    print("# frames: ",N_frames)

    outdata.close()



def wassgridsurface_main():
    print(" WASS surface gridder v.", VERSION )
    print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\nCopyright (C) Filippo Bergamasco 2022 \n")

    howtostring = """
        How to use:
        Assuming that "./output" is the WASS output dir containing the frames you want to grid

        1) Create the gridding output directory
            mkdir gridding

        2) Generate grid config file:
            wassgridsurface --action generategridconfig . gridding

        3) Edit the grid config file ./gridding/gridconfig.txt with your favourite editor

        4) Setup the reconstruction
            wassgridsurface --action setup ./output ./gridding --gridconfig ./gridding/gridconfig.txt --baseline [CAMERA_BASELINE]

        5) Open the image ./gridding/area_grid.png to check the extension of the reconstructed area.
           To make changes, go back to step 3.

        6) Run the gridding
            wassgridsurface --action grid --gridsetup ./gridding/config.mat ./output ./gridding

        Resulting NetCDF file can be found in ./gridding/gridded.nc
    """

    parser = argparse.ArgumentParser( epilog=howtostring, formatter_class=argparse.RawDescriptionHelpFormatter )
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
    parser.add_argument("--ss", "--subsample_percent", type=float, default=100, help="Point subsampling 0..100%%" )
    parser.add_argument("--mf", "--medianfilter", type=int, default=0, help="Median filter window size (0,3,5,7)" )
    parser.add_argument("-n", "--num_frames", type=int, default=-1, help="Number of frames to process. -1 to process all frames." )
    parser.add_argument("-p", "--parallel", type=int, default=1, help="Number of parallel tasks to execute" )
    parser.add_argument("--ia", "--interpolation_algorithm", type=str, default="DCT", help='Interpolation algorithm to use. Alternatives are: "DCT", "IDW", "LinearND" ' )
    parser.add_argument("--mask", type=str, default=None, help='User supplied grid mask filename. Must be a grayscale (bw) image with the same size of the grid' )
    parser.add_argument("--dct_nfreqs", type=int, default=None, help="DCT interpolator number of frequencies" )
    parser.add_argument("--dct_regalpha", type=float, default=None, help="DCT interpolator regularizer alpha" )
    parser.add_argument("--dct_maxtol", type=float, default=None, help="DCT interpolator max function tolerance change" )
    parser.add_argument("--dct_lr", type=float, default=None, help="DCT interpolator learning rate" )
    parser.add_argument("--dct_maxiters", type=int, default=None, help="DCT interpolator max number of iterations" )
    args = parser.parse_args()

    if args.action == "generategridconfig":
        gridconfigfile = path.join(args.outdir,"gridconfig.txt")
        print("Generating ",gridconfigfile)
        with open(gridconfigfile, "w" ) as f:
            f.write("[Area]\n")
            f.write("area_center_x=0.0\n")
            f.write("area_center_y=-35.0\n")
            f.write("area_size=50\n")
            f.write("N=1024\n")

        print("All done, exiting.")
        return


    print("Looking for WASS reconstructed stereo frames in ",args.workdir )
    wass_frames = glob.glob( path.join( args.workdir, "*_wd" ))
    wass_frames.sort()
    print("%d frames found."%len(wass_frames) )
    if args.num_frames>-1:
        wass_frames = wass_frames[:(args.num_frames)]
        print("%d frames to process."%len(wass_frames))

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

        area_size_x=0
        area_size_y=0
        Nx=0
        Ny=0

        try:
            area_size_x = settings.getfloat("Area","area_size_x")
            area_size_y = settings.getfloat("Area","area_size_y")

        except configparser.NoOptionError:
            area_size_x = area_size_y = settings.getfloat("Area","area_size")


        try:
            Nx = settings.getint("Area","Nx")
            Ny = settings.getint("Area","Ny")

        except configparser.NoOptionError:
            Nx = Ny = settings.getint("Area","N")

        setup( wass_frames[0],
               meanplane,
               baseline=args.baseline,
               outdir=args.outdir,
               area_center=np.array([ settings.getfloat("Area","area_center_x"), settings.getfloat("Area","area_center_y")]),
               area_size_x=area_size_x,
               area_size_y=area_size_y,
               Nx = Nx,
               Ny = Ny,
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
              mf=args.mf,
              subsample_percent=args.ss,
              algorithm=args.ia,
              user_mask_filename=args.mask,
              alg_options = {
                  "Nfreqs": args.dct_nfreqs,
                  "REGULARIZER_ALPHA": args.dct_regalpha,
                  "MAX_ITERS": args.dct_maxiters,
                  "TOLERANCE_CHANGE": args.dct_maxtol,
                  "LEARNING_RATE": args.dct_lr,
                  },
              NUM_PARALLEL_PROCESSES=args.parallel )

        print("Gridding completed.")

    else:
        print("Invalid actions specified.")
        sys.exit(-2)


