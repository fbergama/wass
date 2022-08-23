"""
wassgridsurface
Copyright (C) 2022 Filippo Bergamasco

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

import numpy as np
import struct

def load_camera_mesh( meshfile ):
    with open(meshfile, "rb") as mf:
        npts = struct.unpack( "I", mf.read( 4 ) )[0]
        limits = np.array( struct.unpack( "dddddd", mf.read( 6*8 ) ) )
        Rinv = np.reshape( np.array(struct.unpack("ddddddddd", mf.read(9*8) )), (3,3) )
        Tinv = np.reshape( np.array(struct.unpack("ddd", mf.read(3*8) )), (3,1) )

        data = np.reshape( np.array( bytearray(mf.read( npts*3*2 )), dtype=np.uint8 ).view(dtype=np.uint16), (3,npts), order="F" )

        mesh_cam = data.astype( np.float32 )
        mesh_cam = mesh_cam / np.expand_dims( limits[0:3], axis=1) + np.expand_dims( limits[3:6], axis=1 );
        mesh_cam = Rinv@mesh_cam + Tinv;

        return mesh_cam


def compute_sea_plane_RT( plane ):
    assert len(plane)==4, "Plane must be a 4-element vector"
    a=plane[0]
    b=plane[1]
    c=plane[2]
    d=plane[3];
    q = (1-c)/(a*a + b*b)
    R=np.array([[1-a*a*q, -a*b*q, -a], [-a*b*q, 1-b*b*q, -b], [a, b, c] ] )
    T=np.expand_dims( np.array([0,0,d]), axis=1)

    return R, T





def align_on_sea_plane_RT( mesh, R, T ):
    # Rotate, translate
    mesh_aligned = R@mesh + T;

    # Invert z axis
    mesh_aligned[2,:]*=-1.0;

    return mesh_aligned


def align_on_sea_plane( mesh, plane ):
    assert mesh.shape[0]==3, "Mesh must be a 3xN numpy array"

    R,T = compute_sea_plane_RT( plane )
    return align_on_sea_plane_RT(mesh, R,T)



def filter_mesh_outliers( mesh_aligned, ransac_inlier_threshold=0.2, debug=True ):
    def filter_plane_ransac( pts, inlier_threshold = 0.2 ):
        """ Return the plane outliers
        """
        assert ptsR.shape[0] == 3, "Points must be a 3xN array"

        Npts = ptsR.shape[1]

        #print("Running RANSAC on %d points"%Npts)

        best_inliers = np.zeros(Npts)
        best_n_inliers = 0
        best_N = None
        best_P = None

        if Npts>3:
            for iit in range(100):
                p0 = ptsR[:, np.random.randint(0,Npts-1) ]
                p1 = ptsR[:, np.random.randint(0,Npts-1) ]
                p2 = ptsR[:, np.random.randint(0,Npts-1) ]


                N = np.cross( p1-p0, p2-p0 )
                Nnorm = np.linalg.norm(N)
                if Nnorm < 1E-5:
                    continue

                N = N / Nnorm
                P = np.expand_dims( np.mean( np.vstack( [p0,p1,p2]), axis=0 ), axis=1 )

                distances = ptsR - P
                distances = np.abs( distances[0,:]*N[0] + distances[1,:]*N[1] + distances[2,:]*N[2] )

                inliers = distances < inlier_threshold
                curr_n_inliers = np.sum(inliers)


                if curr_n_inliers > best_n_inliers:
                    best_n_inliers = curr_n_inliers
                    best_inliers = inliers
                    best_N = N
                    best_P = P


        #fig = plt.figure( figsize=(10,10) )
        #plt.scatter( ptsR[0,:], ptsR[1,:], c=best_inliers)
        #plt.grid("minor")
        #plt.colorbar()
        return (1-best_inliers).astype(np.bool)



    from scipy.spatial import KDTree
    tree = KDTree( mesh_aligned[0:2,:].T )

    np.random.seed( None )
    votes = np.zeros( mesh_aligned.shape[1] )
    processed = np.zeros( mesh_aligned.shape[1] )

    limits_x = [ np.amin( mesh_aligned[0,:] ), np.amax( mesh_aligned[0,:] ) ]
    limits_y = [ np.amin( mesh_aligned[1,:] ), np.amax( mesh_aligned[1,:] ) ]

    xx,yy = np.meshgrid( np.linspace(limits_x[0],limits_x[1],15), np.linspace(limits_y[0],limits_y[1],15) )
    scan_pts = np.vstack( [xx.flatten(), yy.flatten() ] )

    for ii in range( scan_pts.shape[1]):
        randpt = scan_pts[:,ii]

        pts_idx = np.array( tree.query_ball_point( (randpt[0],randpt[1]),0.5 ) )

        if len(pts_idx)>0:
            processed[ pts_idx ] += 1
            ptsR = mesh_aligned[:,pts_idx]

            outliers = filter_plane_ransac(ptsR, inlier_threshold=ransac_inlier_threshold )
            outliers_idx = pts_idx[outliers]
            votes[outliers_idx] += 1


    mesh_aligned_filtered = mesh_aligned[:, votes<1]

    if debug:
        import matplotlib.pyplot as plt
        plt.figure( figsize=(10,10) )
        plt.scatter( mesh_aligned[0,:], mesh_aligned[1,:], c=votes)
        plt.colorbar()

        plt.figure( figsize=(10,10) )
        plt.scatter( mesh_aligned[0,:], mesh_aligned[1,:], c=processed)
        plt.colorbar()

    return mesh_aligned_filtered

