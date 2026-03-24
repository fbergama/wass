import argparse
from argparse import RawDescriptionHelpFormatter
from netCDF4 import Dataset
import numpy as np
import cv2 as cv
import os
import sys
from tqdm.auto import tqdm, trange
import matplotlib.pyplot as plt

import warnings
from dask.diagnostics import ProgressBar
import xarray as xr
from scipy.ndimage import uniform_filter1d

import scipy
import scipy.signal

from .geometry import compute_slope_and_normals, compute_occlusion_mask
from .spectra import compute_spectrum
from .plotting import plot_spectrum


VERSION="0.6.0"



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
    #tqdm.write("Loading %s"%Ifilename )
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



def action_filter( ncfile:str, cutoff:float, type:str = "lowpass", askconfirm:bool = True, filter_variable:str="Z", overwrite:bool=False ):
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

        filter_output = filter_variable if overwrite else (filter_variable+"_filtered")

        print("Filter input->output: %s -> %s"%(filter_variable, filter_output) )
        ZZ = ds[filter_variable]

        if not filter_output in ds.variables:
            print("Creating output variable %s"%filter_output )
            _ = ds.createVariable(filter_output, datatype=ZZ.datatype, dimensions=ZZ.dimensions )


        ZZout = ds[filter_output]

        FPS = 1.0/dt
        order = 8
        sos = scipy.signal.butter(order, cutoff, btype=type, output='sos', fs=FPS )

        if askconfirm and overwrite:
            print("\nWARNING: data in the nc variable %s will be modified by this operation!"%filter_variable )
            print("           It is strongly recommended that you backup the NetCDF file first.\n")
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

            ZZout[:,ii,:] = out_timeserie


        print("All done.")



def action_spectrum( ncfile:str, outputdir:str ):
    """Computes and plots frequency spectrum
    """

    with Dataset( ncfile, "r") as ds:

        if ds["time"].shape[0]<=256:
            print("Dataset too short. I need more than 256 frames to compute a reliable spectrum")
            sys.exit(-1)

        ZZ = ds["Z"]
        dt = ds["time"][1].item(0) - ds["time"][0].item(0)

        if dt==0:
            print("Invalid time delta. Please fix sequence FPS first")
            sys.exit(-1)

        print("Computing frequency spectrum...")
        f, S, _ = compute_spectrum(ZZ, dt, scale=1/1000 ) # wass nc files are in mm.
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





def action_polarimetric_setup( ncfile:str, cam:int, wassdir:str, outputdir:str, numframes:int, into_nc:bool ):
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

    with Dataset( ncfile, "r" ) as ds:

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







def action_radiance( ncfile, cam, wassdir, outputdir, upscalefactor, N, into_nc:bool ):
    """Computes sea surface radiance with respect to the elevation grid
    """
    print(f"Setting Cam{cam} as reference")
    if not wassdir is None:
        print(f"Images will be loaded from: {wassdir}")

    with Dataset( ncfile, "r+" if into_nc else "r" ) as ds:

        radiance_dataset = None

        Pplane = np.array( ds["/meta"].variables[f"P{cam}plane"] )
        XX,YY,_ = get_grid( ds )

        for _ in range(upscalefactor-1):
            XX = cv.pyrUp( XX )
        for _ in range(upscalefactor-1):
            YY = cv.pyrUp( YY )
        ZZ = ds["Z"]


        if N<=0:
            N = ZZ.shape[0]


        if into_nc:
            # check if radiance dataset exists
            radiance_variable_name = "/radiance_cam%d"%cam
            try:
                radiance_dataset = ds[radiance_variable_name]
                print("Data will be inserted in variable %s inside nc file"%radiance_variable_name )
            except IndexError:
                radiance_dataset = ds.createVariable(radiance_variable_name, datatype="f4", dimensions=("count","X","Y") )
                print("%s variable created in NCfile"%radiance_variable_name )
        # ----------------------


        for idx in trange(N):
            I = None

            if not wassdir is None:
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

            radiance = cv.remap( I, mapx, mapy, cv.INTER_LANCZOS4 )

            if into_nc:
                radiance_dataset[ idx, ...] = radiance/256.0
            else:
                cv.imwrite( os.path.join(outputdir,"%08d_tx_cam%01d.png"%(idx,cam)), radiance )

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
            #radiance = np.zeros( ZZ_data.shape, dtype=np.uint8 ).flatten()
            #radiance[ inside_mask]  = I[ pyi[inside_mask], pxi[inside_mask] ]
            #radiance = np.reshape( radiance, ZZ_data.shape ) 
            #cv.imwrite( os.path.join(outputdir,"%05d.png"%idx), radiance )
        
    pass



def action_bgimage( ncfile, cam, filtersize ):

    radiance_variable_name = "radiance_cam%d"%cam
    output_variable_name = "radiance_bgimage_cam%d"%cam

    temp_file = os.path.join( os.path.dirname(ncfile), '___temp.nc' )
    chunk_size = {'count': filtersize, 'X': 128, 'Y': 128}

    print(f"Filtering {ncfile}/{radiance_variable_name} to {temp_file}/{output_variable_name}")
    print(f"Box-filter size: {filtersize}x1x1")


    with warnings.catch_warnings():
        # Suppress UserWarnings about unoptimal chunk_size. I know that is unoptimal, but
        # X:128 and Y:128 is needed because with a large M the chunk won't fit in RAM.
        #
        warnings.simplefilter("ignore", category=UserWarning)   

        with xr.open_dataset(ncfile, chunks=chunk_size, decode_timedelta=False ) as ds:
            da_var = ds[radiance_variable_name]
            
            # Apply the out-of-core filter using Dask's map_overlap
            filtered_data = da_var.data.map_overlap(
                lambda x: uniform_filter1d(x, size=filtersize, axis=0, mode='reflect'),
                depth={0: filtersize // 2, 1: 0, 2: 0},
                boundary='reflect',
                dtype=da_var.dtype
            )
            
            # Wrap the Dask array back into a new xarray Dataset
            ds_new = xr.Dataset({
                output_variable_name: xr.DataArray(filtered_data, coords=da_var.coords, dims=da_var.dims)
            })
            
            with ProgressBar():
                ds_new.to_netcdf( temp_file )

    print(f"Writing data into {ncfile}/{output_variable_name}")

    # Append the new variable back into the original file
    with xr.open_dataset(temp_file) as ds_temp:

        # Strip the hidden encoding metadata that causes the "Invalid argument" error
        ds_temp[output_variable_name].encoding.clear()

        # Write to ncfile
        ds_temp.to_netcdf(ncfile, mode='a')

    print(f"Cleaning up...")


    if os.path.exists(temp_file):
        os.remove(temp_file)
        print(f"Deleted temporary file: {temp_file}")

    print("Process finished!")




def action_radiance_threshold( ncfile, cam, threshold_val=0.35, use_vats=False ):

    radiance_variable ="radiance_cam%d"%cam 
    radiance_bg_variable = "radiance_bgimage_cam%d"%cam
    output_variable = "radiance_thresholded_cam%d"%cam


    with Dataset(ncfile, "r+") as ds:

        if not radiance_bg_variable in ds.variables:
            print(f"/{radiance_bg_variable} not found in ncfile. Run bgimage action first.")
            sys.exit(-1)

        if not radiance_variable in ds.variables:
            print(f"/{radiance_variable} not found in ncfile. Run radiance action first.")
            sys.exit(-1)
        
        radiance_bg = ds[radiance_bg_variable]
        radiance = ds[radiance_variable]

        N = radiance.shape[0]

        print(f"Thresholding {radiance_variable} -> {output_variable}")
        if use_vats:
            print("Using VATS for threshold")
        else:
            print(f"Threshold val: {threshold_val}")

        if not output_variable in ds.variables:
            print("Creating output variable %s"%output_variable )
            _ = ds.createVariable(output_variable, datatype='u8', dimensions=radiance.dimensions )

        for ii in trange(N):
            I = radiance[ii,:,:]
            Ibg = radiance_bg[ii,:,:]

            Isub = I - (Ibg - np.amin(Ibg))

            if use_vats:
                h, bin_edges = np.histogram( Isub, bins=30, density=True )
                xx = np.arange( h.shape[0] )

                pts = np.concatenate( (xx[np.newaxis,...], h[np.newaxis,...], np.ones( (1,h.shape[0] ) ) ))

                peak_idx = np.argmax(h)
                
                p1 = pts[:, peak_idx]
                p2 = pts[:, -1]
                l = np.cross( p1, p2 )
                distances = np.abs( l @ pts )

                threshold_idx = np.argmax( distances[peak_idx:] )+peak_idx
                threshold_val = bin_edges[ threshold_idx+1 ]


            thresholded = (Isub>threshold_val).astype(np.uint8)
            ds[ output_variable ][ii,:,:] = thresholded



def get_action_description():
    return """
    Post-processing operations:
    ----------------
        info: prints some info about the specified nc file
        visibilitymap: compute visibility map for each grid point 
        radiance: generate surface grid radiance texture
        radiance-threshold: creates a binary mask based on the radiance intensity 
        bgimage: computes the time averaged background image for each radiance image in the sequence 
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
    parser.add_argument('action', choices=['info','visibilitymap','radiance', 'bgimage', 'radiance-threshold', 'psetup', 'spectrum', 'setfps', 'lowpass', 'highpass'], help='post-processing operation to perform (see below)')
    parser.add_argument('ncfile', help='The NetCDF file to post-process, produced by WASS or WASSfast')
    parser.add_argument('--cam', choices=[0,1], type=int, help="Camera to use", default=0 )
    parser.add_argument('--wass-output-dir', type=str, help="WASS output directory. If specified, some data (like undistorted images) are loaded from there", default=None )
    parser.add_argument('--output-dir', "-o", type=str, help="Output directory", default="." )
    parser.add_argument('--radiance-upscale', type=int, help="Upscale factor", default="1" )
    parser.add_argument('--bgimage-filter-size', type=int, help="Filter size for time-average bg image generation", default="2000" )
    parser.add_argument('--fps', type=int, help="Sequence FPS", default="-1" )
    parser.add_argument('--cutoff', type=float, help="filter cutoff in Hz", default="1.0" )
    parser.add_argument('--filter-variable', type=str, help="nc variable to filter", default="Z" )
    parser.add_argument('--num-frames', "-n", type=int, help="Number for frames to process (0 to select all the available frames)", default="0" )
    parser.add_argument('--assume-yes', action=argparse.BooleanOptionalAction, help="Assume yes if a question is asked" )
    parser.add_argument('--into-nc', action=argparse.BooleanOptionalAction, help="Insert data into NC file instead of producing images" )
    parser.add_argument('--overwrite', action=argparse.BooleanOptionalAction, help="Overwrite data when filtering" )
    args = parser.parse_args()


    # Global checks
    if not os.path.exists( args.output_dir ):
        print(f"Output dir {args.output_dir} does not exists, aborting")
        return 

    # Action selection
    if args.action=="info":
        action_info( args.ncfile )
    elif args.action=="radiance":
        action_radiance( args.ncfile, args.cam, args.wass_output_dir, args.output_dir, args.radiance_upscale, args.num_frames, into_nc=args.into_nc )
    elif args.action=="radiance-threshold":
        action_radiance_threshold( args.ncfile, args.cam )
    elif args.action=="bgimage":
        action_bgimage( args.ncfile, args.cam, args.bgimage_filter_size )
    elif args.action=="radiance-threshold":
        action_bgimage( args.ncfile, args.cam )
    elif args.action=="psetup":
        action_polarimetric_setup( args.ncfile, args.cam, args.wass_output_dir, args.output_dir, args.num_frames )
    elif args.action=="visibilitymap":
        action_visibilitymap( args.ncfile, args.cam, args.output_dir, args.num_frames )
    elif args.action=="spectrum":
        action_spectrum( args.ncfile, args.output_dir )
    elif args.action=="lowpass":
        action_filter( args.ncfile, args.cutoff, type="lowpass", askconfirm=args.assume_yes is None, filter_variable=args.filter_variable, overwrite=args.overwrite  )
    elif args.action=="highpass":
        action_filter( args.ncfile, args.cutoff, type="highpass", askconfirm=args.assume_yes is None, filter_variable=args.filter_variable, overwrite=args.overwrite  )
    elif args.action=="setfps":
        if args.fps<=0:
            print("Please set the desired FPS with the --fps argument")
            sys.exit(-1)

        action_setfps( args.ncfile, args.fps )


