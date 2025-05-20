import argparse
from argparse import RawDescriptionHelpFormatter
from netCDF4 import Dataset
import numpy as np
import cv2 as cv
import os
from tqdm.auto import tqdm, trange
import matplotlib.pyplot as plt

VERSION="0.3.3"



def load_data_from_workspace( wassdir: str, workspace_index: int, cam: int, extension=".png", imread_flags=cv.IMREAD_GRAYSCALE ): 
    Ifilename = os.path.join( wassdir, "%06d_wd"%workspace_index, "undistorted", "%08d%s"%(cam,extension) ) 
    tqdm.write("Loading %s"%Ifilename )
    I = cv.imread( Ifilename, imread_flags )
    return I



def compute_slope_and_normals( XX: np.ndarray, YY: np.ndarray, ZZ: np.ndarray ) -> tuple:
    dx = XX[0,1]-XX[0,0]
    dy = YY[1,0]-YY[0,0]

    assert dx>0.0
    assert dy>0.0
    
    slope_y, slope_x = np.gradient( ZZ, dy, dx )

    slope = np.dstack( (slope_x[:,:,None], slope_y[:,:,None] ) )
    normals = np.dstack( (slope_x[:,:,None], slope_y[:,:,None], -np.ones( (ZZ.shape[0],ZZ.shape[1],1)) ) ) 
    normals = -normals / np.linalg.norm( normals, axis=-1, keepdims=True )
    return slope, normals



def compute_occlusion_mask( ZZ: np.ndarray, ray_d: np.ndarray ) -> np.ndarray:
    """Computes occlusion mask of a surface elevation field

    Parameters
        ----------
        ZZ : np.ndarray with shape (W,H)
            Surface elevation scalar field 
            
        ray_d: np.ndarray with shape (W,H,3)
            Ray direction to cast for each grid point.
            (i,j,2) must be positive (ie. rays must go
            upward)

    Returns
        ----------
        a uint8 np.ndarray with shape (W,H) where each
        point (i,j) is either:
        0: if the ray starting at (i,j) with direction 
           ray_d(i,j,:) does not intersect the surface ZZ
        1: otherwise (ie. point i,j is occluded by the 
           surface ZZ)
    """
    assert ray_d.shape == (ZZ.shape[0], ZZ.shape[1], 3 )
    assert np.amin(ray_d[:,:,-1]>0) # rays must go upward

    maxz = np.amax(ZZ)
    ray_d_norm = np.reshape( ray_d / np.expand_dims( np.amax(ray_d[:,:,:2], axis=-1), axis=-1 ), (-1,3) )

    seed_x, seed_y = np.meshgrid( np.arange( ZZ.shape[1], dtype=np.int32), np.arange(ZZ.shape[0], dtype=np.int32) )
    seeds = np.vstack( (seed_x.flatten(), seed_y.flatten()) ).T

    occlusionmask = np.zeros( ZZ.shape, dtype=np.uint8 )

    seeds_curr = np.copy(seeds)
    seeds_z = np.expand_dims( ZZ[ seeds_curr[:,1], seeds_curr[:,0] ], axis=-1 )
    seeds_curr = np.hstack( (seeds_curr,seeds_z) )

    while seeds_curr.shape[0] > 0:
        # move seeds_curr
        seeds_curr = seeds_curr + ray_d_norm
        rounded_seeds_II = np.round( seeds_curr[:,1] ).astype(np.uint32)
        rounded_seeds_JJ = np.round( seeds_curr[:,0] ).astype(np.uint32)

        # check bounds
        good_seeds = np.logical_and( rounded_seeds_II>=0, rounded_seeds_II<ZZ.shape[0])
        good_seeds = np.logical_and( good_seeds, rounded_seeds_JJ>=0 )
        good_seeds = np.logical_and( good_seeds, rounded_seeds_JJ<ZZ.shape[1] )
        good_seeds = np.logical_and( good_seeds, seeds_curr[:,2]<=maxz )  # if a seed is above maxz can be safely removed
    
        # keep seeds that are within bounds
        seeds_curr = seeds_curr[ good_seeds, : ]
        seeds = seeds[ good_seeds, :]
        ray_d_norm = ray_d_norm[ good_seeds, :]
        rounded_seeds_II = rounded_seeds_II[ good_seeds ]
        rounded_seeds_JJ = rounded_seeds_JJ[ good_seeds ]

        # Sample elevation at seeds location
        z_val_at_seeds = ZZ[ rounded_seeds_II, rounded_seeds_JJ ]
    
        # check which seeds are below elevation
        occluded_seeds = z_val_at_seeds >= seeds_curr[:,2]
    
        # set mask accordingly
        occlusionmask[ seeds[occluded_seeds,1], seeds[occluded_seeds,0] ] = 1
    
        # keep seeds that are not occluded
        good_seeds = np.logical_not( occluded_seeds )
        seeds_curr = seeds_curr[ good_seeds, : ]
        seeds = seeds[ good_seeds, :]
        ray_d_norm = ray_d_norm[ good_seeds, :]

        
    return occlusionmask



def action_info( ncfile ):
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



def action_visibilitymap( ncfile:str, cam:str, outputdir:str, numframes:int ):
    print(f"Setting Cam{cam} as reference")

    with Dataset( ncfile, "r") as ds:

        Cam2Grid = np.array( ds["/meta"].variables[f"Cam{cam}toGrid"] )
        cam_origin = np.expand_dims(Cam2Grid[:,-1], axis=-1)

        XX = np.array( ds["X_grid"] )/1000.0
        YY = np.array( ds["Y_grid"] )/1000.0
        ZZ = ds["Z"]
        N = ZZ.shape[0]


        XXl = np.expand_dims( XX.flatten(), axis=1 )
        YYl = np.expand_dims( YY.flatten(), axis=1 )

        for idx in trange(N if numframes == 0 else numframes):
            ZZ_data = np.array( ZZ[idx,:,:] )/1000.0
            ZZl = np.expand_dims( ZZ_data.flatten(), axis=1 )
            p3d = np.concatenate( [XXl,YYl,ZZl,ZZl*0], axis=-1 ).T
            p3d[3,:]=1

            ray_z_g = p3d - cam_origin
            ray_z_g = ray_z_g[:3, :]
            ray_z_g = ray_z_g / np.linalg.norm( ray_z_g, axis=0 )  # Rays_z in grid reference system

            # Compute occlusion mask
            ray_z_g_g = np.transpose( np.reshape(-ray_z_g, (3,XX.shape[0],XX.shape[1])), (1,2,0))
            occlusion_mask = compute_occlusion_mask(ZZ_data, ray_z_g_g )
            del ray_z_g_g
            tqdm.write(f"Image {idx} (Cam {cam}) has {np.sum(occlusion_mask)/occlusion_mask.size*100.0}% occluded points")

            cv.imwrite( os.path.join(outputdir,"%08d_occlusion_mask_cam%01d.png"%(idx,cam)), occlusion_mask )




def action_polarimetric_setup( ncfile:str, cam:int, wassdir:str, outputdir:str ):

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

        XX = np.array( ds["X_grid"] )/1000.0
        YY = np.array( ds["Y_grid"] )/1000.0
        ZZ = ds["Z"]
        N = ZZ.shape[0]

        Iw, Ih = I.shape[1], I.shape[0]
        toNorm = np.array( [[ 2.0/Iw, 0     , -1, 0],
                            [ 0     , 2.0/Ih, -1, 0],
                            [ 0,      0,       1, 0],
                            [ 0,      0,       0, 1]], dtype=float )
        toNormI = np.linalg.inv(toNorm)
        Pcam = toNormI @ Pplane 

        Savg = np.zeros( (XX.shape[0],XX.shape[1],3), dtype=float )
        Navg = np.zeros( (XX.shape[0],XX.shape[1],3), dtype=float )
        Zavg = np.zeros( (XX.shape[0],XX.shape[1]), dtype=float )

        for idx in trange(N):

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
            occlusion_mask = compute_occlusion_mask(ZZ_data, ray_z_g_g )
            del ray_z_g_g
            cv.imwrite( os.path.join(outputdir,"%08d_occlusion_mask.png"%idx), occlusion_mask )


            # Stokes vector's sampling
            S0 = load_data_from_workspace(wassdir,idx,cam,"_S0.tiff", cv.IMREAD_ANYDEPTH )
            assert S0.shape[0] > 0
            S0grid = cv.remap( S0, mapx, mapy, cv.INTER_LINEAR )
            del S0
            cv.imwrite( os.path.join(outputdir,"%08d_S0.jpg"%idx), np.clip(S0grid*128.0,0.0,255.0).astype(np.uint8) )

            S1 = load_data_from_workspace(wassdir,idx,cam,"_S1.tiff", cv.IMREAD_ANYDEPTH )
            assert S1.shape[0] > 0
            S1grid = cv.remap( S1, mapx, mapy, cv.INTER_LINEAR )
            del S1

            S2 = load_data_from_workspace(wassdir,idx,cam,"_S2.tiff", cv.IMREAD_ANYDEPTH )
            assert S2.shape[0] > 0
            S2grid = cv.remap( S2, mapx, mapy, cv.INTER_LINEAR )
            del S2

            dolp = np.sqrt( np.square(S1grid)+np.square(S2grid) ) / S0grid
            dolp = np.clip(dolp*255.0,0.0,255.0).astype(np.uint8) 
            cv.imwrite( os.path.join(outputdir,"%08d_dolp.jpg"%idx), cv.applyColorMap(dolp, cv.COLORMAP_JET) )


            Sgrid = np.concatenate( [np.expand_dims(k,axis=-1) for k in (S0grid, S1grid, S2grid)], axis=-1 )
            Savg += Sgrid

            np.savez( os.path.join(outputdir,"%08d_pdata"%idx), S=Sgrid, N_grid=Nfield, rays_cam=ray_z, Cam2Grid=Cam2Grid )


        # we iterated through all the surfaces

        Savg /= float(N)
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
    print(f"Setting Cam{cam} as reference")
    if not wassdir is None:
        print(f"Images will be loaded from: {wassdir}")

    with Dataset( ncfile, "r") as ds:
        Pplane = np.array( ds["/meta"].variables[f"P{cam}plane"] )
        XX = np.array( ds["X_grid"] )/1000.0
        YY = np.array( ds["Y_grid"] )/1000.0

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
    parser.add_argument('action', choices=['info','visibilitymap','texture', 'psetup'], help='post-processing operation to perform (see below)')
    parser.add_argument('ncfile', help='The NetCDF file to post-process, produced by WASS or WASSfast')
    parser.add_argument('--cam', choices=[0,1], type=int, help="Camera to use", default=0 )
    parser.add_argument('--wass_output_dir', type=str, help="WASS output directory. If specified, some data (like undistorted images) are loaded from there", default=None )
    parser.add_argument('--output_dir', "-o", type=str, help="Output directory", default="." )
    parser.add_argument('--texture_upscale', type=int, help="Upscale factor", default="1" )
    parser.add_argument('--num_frames', "-n", type=int, help="Number for frames to process (0 to select all the available frames)", default="0" )
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
        action_polarimetric_setup( args.ncfile, args.cam, args.wass_output_dir, args.output_dir )
    elif args.action=="visibilitymap":
        action_visibilitymap( args.ncfile, args.cam, args.output_dir, args.num_frames )


