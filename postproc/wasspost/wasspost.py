import argparse
from argparse import RawDescriptionHelpFormatter
from netCDF4 import Dataset
import numpy as np
import cv2 as cv
import os
import sys
from tqdm.auto import tqdm, trange
import matplotlib.pyplot as plt

import scipy
import scipy.signal

from .geometry import compute_slope_and_normals, compute_occlusion_mask
from .spectra import compute_spectrum
from .plotting import plot_spectrum

VERSION="0.4.2"



def get_grid( dataset ):
    """Returns grid data from WASS' NetCDF dataset
    """

    XX = np.array( dataset["X_grid"] )/1000.0
    YY = np.array( dataset["Y_grid"] )/1000.0
    dx = XX[0,1]-XX[0,0]
    dy = YY[1,0]-YY[0,0]
    assert np.allclose( dx, dy), "grid cells must be square"
    return XX,YY,dx




def load_data_from_workspace( wassdir: str, workspace_index: int, cam: int, extension=".png", imread_flags=cv.IMREAD_GRAYSCALE ): 
    Ifilename = os.path.join( wassdir, "%06d_wd"%workspace_index, "undistorted", "%08d%s"%(cam,extension) ) 
    tqdm.write("Loading %s"%Ifilename )
    I = cv.imread( Ifilename, imread_flags )
    return I




def action_info( ncfile ):
    """Prints info about a WASS' NetCDF file
    """
    print(f"Opening {ncfile}")
    S = ""
    with Dataset( ncfile, "r") as ds:
        S = ""
        S += "\nVariables:\n\n"
        for v in ds.variables:
            S += "%s - %s (%s)\n"%(v, ds.variables[v].shape,ds.variables[v].dtype)
        #S += "\nAttributes:\n\n"
        #for a in ds.ncattrs():
        #    S += " - %s = %s\n"%(a,ds.getncattr(a))
        S += "\nGroups: \n"
        for g in ds.groups:
            S += " %s\n"%ds[g].path
            S += "\n  Variables:\n"
            for v in ds[g].variables:
                S += "    - %s: \n%s\n"%(v,np.array(ds[g].variables[v]))
            S += "\n  ncattrs:\n"
            for a in ds[g].ncattrs():
                S += "    %s = %s\n"%(a,ds[g].getncattr(a))

        print(S)



def action_filter( ncfile:str, cutoff:float, type:str = "lowpass", askconfirm:bool = True ):
    """ Applies an 8th order Butterworth lowpass or highpass filter (time-wise)
        time delta between frames must be known

        cutoff: frequency in Hz.
    """

    with Dataset( ncfile, "r+") as ds:

        if ds["time"].shape[0]<=10:
            print("Dataset too short. I need more than 10 frames for lowpass filtering")
            sys.exit(-1)

        dt = ds["time"][1].item(0) - ds["time"][0].item(0)
        if dt==0:
            print("Invalid time delta. Please fix sequence FPS first")
            sys.exit(-1)

        ZZ = ds["Z"]
        FPS = 1.0/dt
        order = 8
        sos = scipy.signal.butter(order, cutoff, btype=type, output='sos', fs=FPS )

        if askconfirm:
            print("\nWARNING: Sea surface elevation data will be modified by this operation!")
            print("         It is strongly recommended that you backup the NetCDF file first.\n")
            user_input = input("Do you want to continue? (y/n): ")
            if user_input.lower() != "y":
                sys.exit(0)

        print("\nApplying 8th order Butterworth %s filter with cutoff=%3.3f Hz"%(type,cutoff))

        for ii in tqdm( range(ZZ.shape[1]) ):
            in_timeserie = np.array( ZZ[:,ii,:])
            out_timeserie = scipy.signal.sosfiltfilt(sos, in_timeserie, axis=0)

            if type=='highpass':
                # Also remove the mean
                out_timeserie = out_timeserie - np.mean( out_timeserie, axis=0, keepdims=True )

            ZZ[:,ii,:]=out_timeserie


        print("All done.")



def action_spectrum( ncfile:str, outputdir:str ):
    """Computes and plots frequency spectrum
    """

    with Dataset( ncfile, "r") as ds:

        if ds["time"].shape[0]<=512:
            print("Dataset too short. I need more than 512 frames to compute a reliable spectrum")
            sys.exit(-1)

        ZZ = ds["Z"]
        dt = ds["time"][1].item(0) - ds["time"][0].item(0)

        if dt==0:
            print("Invalid time delta. Please fix sequence FPS first")
            sys.exit(-1)

        print("Computing frequency spectrum...")
        f, S, _ = compute_spectrum(ZZ,dt)
        plot_spectrum(f, S, os.path.join(outputdir,"spectrum.png"))





def action_setfps( ncfile:str, FPS:int ):
    """Overwrites the sequence FPS metadata and recomputes
       all timestamps. Note: data is not resampled, this function
       is meant to manually set the timestamps if they were not
       provied during the gridding phase.
    """

    with Dataset( ncfile, "r+") as ds:
        dt = 1.0/float(FPS)
        N = ds["time"].shape[0]
        new_times = np.arange(N).astype(float)*dt
        ds["time"][:] = new_times
        ds["/meta"].setncattr("fps", FPS)
        print("New FPS set to ", FPS)




def action_visibilitymap( ncfile:str, cam:str, outputdir:str, numframes:int ):
    """Computes visibility map
    """
    print(f"Setting Cam{cam} as reference")

    with Dataset( ncfile, "r") as ds:

        Cam2Grid = np.array( ds["/meta"].variables[f"Cam{cam}toGrid"] )
        cam_origin = np.expand_dims(Cam2Grid[:,-1], axis=-1)

        ZZ = ds["Z"]
        N = ZZ.shape[0]

        XX,YY,dx = get_grid( ds )

        XXl = np.expand_dims( XX.flatten(), axis=1 )
        YYl = np.expand_dims( YY.flatten(), axis=1 )

        for idx in trange(N if numframes == 0 else numframes):
            ZZ_data = np.array( ZZ[idx,:,:] )/1000.0
            ZZl = np.expand_dims( ZZ_data.flatten(), axis=1 )
            p3d = np.concatenate( [XXl,YYl,ZZl,ZZl*0+1], axis=-1 ).T

            ray_z_g = p3d - cam_origin
            ray_z_g = ray_z_g[:3, :]
            ray_z_g = ray_z_g / np.linalg.norm( ray_z_g, axis=0 )  # Rays_z in grid reference system

            _, Nfield = compute_slope_and_normals( XX, YY, ZZ_data )
            incident_angles = np.rad2deg( np.acos( np.linalg.vecdot( np.reshape(Nfield, (-1,3)), (-ray_z_g).T  ) ) )
            incident_angles = np.reshape( incident_angles, XX.shape )

            #plt.figure( figsize=(10,10) )
            #plt.imshow( incident_angles, cmap='jet', vmin=0, vmax=90 )
            #plt.colorbar()
            #plt.title("Incident angles (deg)")
            #plt.tight_layout()
            #plt.savefig( os.path.join( outputdir,"%05d_incident_angles.png"%idx ) )
            #plt.close()

            # Compute occlusion mask
            ray_z_g_g = np.transpose( np.reshape(-ray_z_g, (3,XX.shape[0],XX.shape[1])), (1,2,0))
            occlusion_mask = compute_occlusion_mask( ZZ_data/dx, ray_z_g_g, invert_y_axis=False )
            del ray_z_g_g

            #normal_mask = np.zeros_like( occlusion_mask )
            #cv.imwrite( os.path.join(outputdir,"%08d_normal_mask_cam%01d.png"%(idx,cam)), normal_mask*255 )

            # consider occluded also points with incident angles > 88 deg
            occlusion_mask[ incident_angles>=88 ] = 1

            n_occluded = np.sum(occlusion_mask)
            tqdm.write("Image %d (Cam %d) has %d (%3.2f%%) occluded points"%(idx,cam,n_occluded,n_occluded/occlusion_mask.size*100.0))

            cv.imwrite( os.path.join(outputdir,"%08d_occlusion_mask_cam%01d.png"%(idx,cam)), occlusion_mask*255 )





def action_polarimetric_setup( ncfile:str, cam:int, wassdir:str, outputdir:str, numframes:int ):
    """Computes DOLP/AOLP/normals etc. for further polarimetric processing
    """

    ENABLE_PLOTS=False

    print(f"Setting Cam{cam} as reference")
    if not wassdir is None:
        print(f"Images will be loaded from: {wassdir}")

    if wassdir is None:
        print("wass output dir must be specified, aborting")
        return

    I = load_data_from_workspace( wassdir, 0, cam )
    Iw, Ih = I.shape[1], I.shape[0]
    print("Image size: %dx%d"%(Iw,Ih))

    with Dataset( ncfile, "r") as ds:
        Pplane = np.array( ds["/meta"].variables[f"P{cam}plane"] )
        Cam2Grid = np.array( ds["/meta"].variables[f"Cam{cam}toGrid"] )
        K = np.array( ds["/meta"].variables[f"intr{cam}"] )
        cam_origin = np.expand_dims(Cam2Grid[:,-1], axis=-1)

        ZZ = ds["Z"]
        N = ZZ.shape[0]
        XX,YY,dx = get_grid( ds )

        Iw, Ih = I.shape[1], I.shape[0]
        toNorm = np.array( [[ 2.0/Iw, 0     , -1, 0],
                            [ 0     , 2.0/Ih, -1, 0],
                            [ 0,      0,       1, 0],
                            [ 0,      0,       0, 1]], dtype=float )
        toNormI = np.linalg.inv(toNorm)
        Pcam = toNormI @ Pplane 

        Savg = np.zeros( (XX.shape[0],XX.shape[1],3), dtype=float )
        ValidData = np.zeros( (XX.shape[0],XX.shape[1]), dtype=float )
        Navg = np.zeros( (XX.shape[0],XX.shape[1],3), dtype=float )
        Zavg = np.zeros( (XX.shape[0],XX.shape[1]), dtype=float )

        for idx in trange(N if numframes == 0 else numframes):

            ZZ_data = np.array( ZZ[idx,:,:] )/1000.0
            Zavg += ZZ_data

            XXl = np.expand_dims( XX.flatten(), axis=1 )
            YYl = np.expand_dims( YY.flatten(), axis=1 )
            ZZl = np.expand_dims( ZZ_data.flatten(), axis=1 )
            p3d = np.concatenate( [XXl,YYl,ZZl,ZZl*0], axis=-1 ).T
            p3d[3,:]=1
            p2d = Pcam @ p3d
            p2d = p2d[:3,:] / p2d[2,:]
            mapx = np.reshape( p2d[0,:], ZZ_data.shape ).astype( np.float32 )
            mapy = np.reshape( p2d[1,:], ZZ_data.shape ).astype( np.float32 )

            p2dN = np.copy( np.linalg.inv(K) @ p2d )
            ray_z = p2dN / np.linalg.norm( p2dN, axis=0 )  # Rays_z in cam reference system

            ray_z_g = p3d - cam_origin
            ray_z_g = ray_z_g[:3, :]
            ray_z_g = ray_z_g / np.linalg.norm( ray_z_g, axis=0 )  # Rays_z in grid reference system


            # Compute slope and surface normals
            slope, Nfield = compute_slope_and_normals( XX, YY, ZZ_data )
            Navg += Nfield

            if ENABLE_PLOTS:
                plt.figure( figsize=(20,10) )
                plt.subplot(1,2,1)
                plt.imshow( slope[:,:,0], cmap='jet' )
                plt.colorbar()
                plt.title("Slope X")
                plt.subplot(1,2,2)
                plt.imshow( slope[:,:,1], cmap='jet' )
                plt.colorbar()
                plt.title("Slope Y")
                plt.tight_layout()
                plt.savefig( os.path.join( outputdir,"%05d_slope.png"%idx ) )
                plt.close()


            # Compute incident angles (N dot -ray_z_g)
            incident_angles = np.rad2deg( np.acos( np.linalg.vecdot( np.reshape(Nfield, (-1,3)), (-ray_z_g).T  ) ) )
            incident_angles = np.reshape( incident_angles, XX.shape )

            if ENABLE_PLOTS:
                plt.figure( figsize=(10,10) )
                plt.imshow( incident_angles, cmap='hsv' )
                plt.colorbar()
                plt.title("Incident angles (deg)")
                plt.tight_layout()
                plt.savefig( os.path.join( outputdir,"%05d_incident_angles.png"%idx ) )
                plt.close()


            # Compute occlusion mask
            ray_z_g_g = np.transpose( np.reshape(-ray_z_g, (3,XX.shape[0],XX.shape[1])), (1,2,0))
            occlusion_mask = compute_occlusion_mask(ZZ_data/dx, ray_z_g_g )
            occlusion_mask[ incident_angles>=85 ] = 1
            del ray_z_g_g
            cv.imwrite( os.path.join(outputdir,"%08d_occlusion_mask.png"%idx), occlusion_mask*255 )


            # Stokes vector's sampling
            S0 = load_data_from_workspace(wassdir,idx,cam,"_S0.tiff", cv.IMREAD_ANYDEPTH )
            assert S0.shape[0] > 0
            S0grid = cv.remap( S0, mapx, mapy, cv.INTER_LINEAR )
            S0grid[occlusion_mask==1]=np.nan
            del S0
            cv.imwrite( os.path.join(outputdir,"%08d_S0.jpg"%idx), np.clip(S0grid*128.0,0.0,255.0).astype(np.uint8) )

            S1 = load_data_from_workspace(wassdir,idx,cam,"_S1.tiff", cv.IMREAD_ANYDEPTH )
            assert S1.shape[0] > 0
            S1grid = cv.remap( S1, mapx, mapy, cv.INTER_LINEAR )
            S1grid[occlusion_mask==1]=np.nan
            del S1

            S2 = load_data_from_workspace(wassdir,idx,cam,"_S2.tiff", cv.IMREAD_ANYDEPTH )
            assert S2.shape[0] > 0
            S2grid = cv.remap( S2, mapx, mapy, cv.INTER_LINEAR )
            S2grid[occlusion_mask==1]=np.nan
            del S2

            dolp = np.sqrt( np.square(S1grid)+np.square(S2grid) ) / S0grid
            dolp = np.clip(dolp*255.0,0.0,255.0).astype(np.uint8) 
            cv.imwrite( os.path.join(outputdir,"%08d_dolp.jpg"%idx), cv.applyColorMap(dolp, cv.COLORMAP_JET) )


            Sgrid = np.concatenate( [np.expand_dims(k,axis=-1) for k in (S0grid, S1grid, S2grid)], axis=-1 )

            Savg += np.nan_to_num(Sgrid)
            ValidData += (1.0-occlusion_mask.astype(float))

            np.savez( os.path.join(outputdir,"%08d_pdata"%idx), S=Sgrid, N_grid=Nfield, rays_cam=ray_z, Cam2Grid=Cam2Grid )


        # we iterated through all the surfaces

        Savg /= np.expand_dims(ValidData, axis=-1)
        Zavg /= float(N)
        Nnorm = np.linalg.norm( Navg, axis=-1 )
        Navg[:,:,0] /= Nnorm
        Navg[:,:,1] /= Nnorm
        Navg[:,:,2] /= Nnorm

        plt.figure( figsize=(10,10) )
        plt.imshow( Zavg, cmap='jet' )
        plt.colorbar()
        plt.title("Average elevation")
        plt.tight_layout()
        plt.savefig( os.path.join( outputdir,"avg_Z.png" ) )
        plt.close()

        plt.figure( figsize=(30,10) )

        for ii in range(3):
            plt.subplot(1,3,ii+1)
            plt.imshow( Savg[:,:,ii], cmap='jet' if ii>0 else 'gray' )
            plt.colorbar()
            plt.title("Average S%d"%ii)

        plt.tight_layout()

        plt.savefig( os.path.join( outputdir,"avg_S.png" ) )
        plt.close()

        np.savez( os.path.join(outputdir,"pdata_avg"), Savg=Savg, Navg_grid=Navg, Zavg=Zavg, Cam2Grid=Cam2Grid )







def action_texture( ncfile, cam, wassdir, outputdir, upscalefactor, N ):
    """Computes sea surface texture with respect to the elevation grid
    """
    print(f"Setting Cam{cam} as reference")
    if not wassdir is None:
        print(f"Images will be loaded from: {wassdir}")

    with Dataset( ncfile, "r") as ds:
        Pplane = np.array( ds["/meta"].variables[f"P{cam}plane"] )
        XX,YY,_ = get_grid( ds )

        for _ in range(upscalefactor-1):
            XX = cv.pyrUp( XX )
        for _ in range(upscalefactor-1):
            YY = cv.pyrUp( YY )
        ZZ = ds["Z"]


        if N<=0:
            N = ZZ.shape[0]

        # ----------------------

        for idx in trange(N):
            I = None

            if not wassdir is None:
                #Ifilename = os.path.join( wassdir, "%06d_wd"%idx, "undistorted", "%08d.png"%cam ) 
                #tqdm.write("Loading %s"%Ifilename )
                #I = cv.imread( Ifilename, cv.IMREAD_GRAYSCALE )
                I = load_data_from_workspace( wassdir, idx, cam )
            else:
                I = cv.imdecode( ds[f"cam{cam}images"][idx], cv.IMREAD_GRAYSCALE )

            Iw, Ih = I.shape[1], I.shape[0]
            toNorm = np.array( [[ 2.0/Iw, 0     , -1, 0],
                                [ 0     , 2.0/Ih, -1, 0],
                                [ 0,      0,       1, 0],
                                [ 0,      0,       0, 1]], dtype=float )
            toNormI = np.linalg.inv(toNorm)
            Pcam = toNormI @ Pplane 

            ZZ_data = np.array( ZZ[idx,:,:] )/1000.0

            for _ in range(upscalefactor-1):
                ZZ_data = cv.pyrUp( ZZ_data )

            XXl = np.expand_dims( XX.flatten(), axis=1 )
            YYl = np.expand_dims( YY.flatten(), axis=1 )
            ZZl = np.expand_dims( ZZ_data.flatten(), axis=1 )
            p3d = np.concatenate( [XXl,YYl,ZZl,ZZl*0], axis=-1 ).T
            p3d[3,:]=1
            p2d = Pcam @ p3d
            p2d = p2d[:2,:] / p2d[2,:]

            mapx = np.reshape( p2d[0,:], ZZ_data.shape ).astype( np.float32 )
            mapy = np.reshape( p2d[1,:], ZZ_data.shape ).astype( np.float32 )

            texture = cv.remap( I, mapx, mapy, cv.INTER_LANCZOS4 )
            cv.imwrite( os.path.join(outputdir,"%08d_tx_cam%01d.png"%(idx,cam)), texture )

            #plt.figure( figsize=(20,10) )
            #plt.imshow( I, cmap="gray" )
            #plt.scatter( p2d[0,:], p2d[1,:], c=ZZl, s=0.5, vmin=-2.0, vmax=2.0 )
            #plt.colorbar()
            #plt.tight_layout()
            #plt.savefig( os.path.join( outputdir,"%05d.png"%idx ) )
            #plt.close()

            #pxi = np.round( p2d[0,:] ).astype(int)
            #pyi = np.round( p2d[1,:] ).astype(int)
            #inside_mask = np.logical_and( np.logical_and( pxi >= 0 , pxi < Iw ),
            #np.logical_and( pyi >= 0 , pyi < Ih ) )
            #
            #texture = np.zeros( ZZ_data.shape, dtype=np.uint8 ).flatten()
            #texture[ inside_mask]  = I[ pyi[inside_mask], pxi[inside_mask] ]
            #texture = np.reshape( texture, ZZ_data.shape ) 
            #cv.imwrite( os.path.join(outputdir,"%05d.png"%idx), texture )
        
    pass



def get_action_description():
    return """
    Post-processing operations:
    ----------------
        info: prints some info about the specified nc file
        visibilitymap: compute visibility map for each grid point 
        texture: generate surface grid texture
        psetup: setup polarimetric data for further processing
        spectrum: plots frequency spectrum
        setfps: overwrites sequence FPS and recomputes times accordingly
        lowpass: applies a forward-backward 8th order Butterworth lowpass filter (time-wise)
        highpass: applies a forward-backward 8th order Butterworth highpass filter (time-wise)
                 (use the --cutoff argument to set the cut-off frequency in Hz)


    Note 
    ----------------
        This program is still under active development. You can
        expect severe changes between different versions. Use it
        wisely.
        """


def wasspost_main():
    parser = argparse.ArgumentParser(
                        prog='wasspost',
                        description='WASS NetCDF post processing tool v.'+VERSION,
                        epilog=get_action_description(),
                        formatter_class=RawDescriptionHelpFormatter )
    parser.add_argument('action', choices=['info','visibilitymap','texture', 'psetup', 'spectrum', 'setfps', 'lowpass', 'highpass'], help='post-processing operation to perform (see below)')
    parser.add_argument('ncfile', help='The NetCDF file to post-process, produced by WASS or WASSfast')
    parser.add_argument('--cam', choices=[0,1], type=int, help="Camera to use", default=0 )
    parser.add_argument('--wass_output_dir', type=str, help="WASS output directory. If specified, some data (like undistorted images) are loaded from there", default=None )
    parser.add_argument('--output_dir', "-o", type=str, help="Output directory", default="." )
    parser.add_argument('--texture_upscale', type=int, help="Upscale factor", default="1" )
    parser.add_argument('--fps', type=int, help="Sequence FPS", default="-1" )
    parser.add_argument('--cutoff', type=float, help="filter cutoff in Hz", default="1.0" )
    parser.add_argument('--num_frames', "-n", type=int, help="Number for frames to process (0 to select all the available frames)", default="0" )
    parser.add_argument('--assume-yes', action=argparse.BooleanOptionalAction, help="Assume yes if a question is asked" )
    args = parser.parse_args()


    # Global checks
    if not os.path.exists( args.output_dir ):
        print(f"Output dir {args.output_dir} does not exists, aborting")
        return 

    # Action selection
    if args.action=="info":
        action_info( args.ncfile )
    elif args.action=="texture":
        action_texture( args.ncfile, args.cam, args.wass_output_dir, args.output_dir, args.texture_upscale, args.num_frames )
    elif args.action=="psetup":
        action_polarimetric_setup( args.ncfile, args.cam, args.wass_output_dir, args.output_dir, args.num_frames )
    elif args.action=="visibilitymap":
        action_visibilitymap( args.ncfile, args.cam, args.output_dir, args.num_frames )
    elif args.action=="spectrum":
        action_spectrum( args.ncfile, args.output_dir )
    elif args.action=="lowpass":
        action_filter( args.ncfile, args.cutoff, type="lowpass", askconfirm=args.assume_yes is None )
    elif args.action=="highpass":
        action_filter( args.ncfile, args.cutoff, type="highpass", askconfirm=args.assume_yes is None )
    elif args.action=="setfps":
        if args.fps<=0:
            print("Please set the desired FPS with the --fps argument")
            sys.exit(-1)

        action_setfps( args.ncfile, args.fps )


