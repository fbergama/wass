import argparse
from argparse import RawDescriptionHelpFormatter
from netCDF4 import Dataset
import numpy as np
import cv2 as cv
import os
from tqdm.auto import tqdm, trange
#import matplotlib.pyplot as plt

VERSION="0.1.3"



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



def action_visibilitymap( ncfile, cam ):
    print(f"Setting Cam{cam} as reference")

    with Dataset( ncfile, "r") as ds:
        K = np.array( ds["/meta"].variables[f"intr{cam}"] )

        XX = np.array( ds["X_grid"] )/1000.0
        YY = np.array( ds["Y_grid"] )/1000.0
        ZZ = ds["Z"]
    pass



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

        all_textures = []
        for idx in trange(N):
            I = None

            if not wassdir is None:
                Ifilename = os.path.join( wassdir, "%06d_wd"%idx, "undistorted", "%08d.png"%cam ) 
                tqdm.write("Loading %s"%Ifilename )
                #I = cv.imread( Ifilename, cv.IMREAD_GRAYSCALE )
                I = cv.imread( Ifilename, cv.IMREAD_COLOR )
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
            all_textures.append( texture )
            cv.imwrite( os.path.join(outputdir,"tx_%08d.png"%idx), texture )

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
        
        tmean = np.mean( np.array( all_textures), axis=0 )
        cv.imwrite( os.path.join(outputdir,"mean.png"), tmean )
    pass



def get_action_description():
    return """
    Post-processing operations:
    ----------------
        info: prints some info about the specified nc file
        visibilitymap: compute visibility map for each grid point 
        texture: generate surface grid texture
        """


def wasspost_main():
    parser = argparse.ArgumentParser(
                        prog='wasspost',
                        description='WASS NetCDF post processing tool',
                        epilog=get_action_description(),
                        formatter_class=RawDescriptionHelpFormatter )
    parser.add_argument('action', choices=['info','visibilitymap','texture'], help='post-processing operation to perform (see below)')
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
