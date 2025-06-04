import numpy as np



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



def compute_occlusion_mask( ZZ: np.ndarray, ray_d: np.ndarray, invert_y_axis = False ) -> np.ndarray:
    """Computes occlusion mask of a surface elevation field

    Parameters
        ----------
        ZZ : np.ndarray with shape (W,H)
            Surface elevation scalar field 
            
        ray_d: np.ndarray with shape (W,H,3)
            Ray direction to cast for each grid point.
            (i,j,2) must be positive (ie. rays must go
            upward)

        invert_y_axis: invert ray_d y-axis before computation

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
    assert np.amin(ray_d[:,:,-1]>0), "rays must go upward"

    maxz = np.amax(ZZ)

    ray_d_norm = np.reshape( ray_d / np.expand_dims( np.amax( np.abs(ray_d[:,:,:2]), axis=-1), axis=-1 ), (-1,3) )

    if invert_y_axis:
        ray_d_norm[:,1] *= -1


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
