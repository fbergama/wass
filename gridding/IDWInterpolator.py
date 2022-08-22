import scipy.signal
import numpy as np
import cv2 as cv


class IDWInterpolator:

    def __init__( self, KSIZE=5, exp=2.4, reps=3 ):
        assert KSIZE%2 == 1
        Kd = np.array( [ (k-KSIZE//2) for k in range(KSIZE) ], dtype=np.float64 )

        Kx = np.tile( Kd, (KSIZE,1) )
        Ky = Kx.T

        K = 1.0 / np.power( np.sqrt( Kx**2 + Ky**2 ), exp)
        K[KSIZE//2, KSIZE//2]=0
        self.K = K
        self.reps = reps


    def __call__( self, I):

        orig_pts_mask = 1-np.isnan(I).astype(np.uint8)
        final_mask = cv.morphologyEx( orig_pts_mask, cv.MORPH_CLOSE, np.ones( self.K.shape, dtype=np.uint8), iterations=reps )

        mask = orig_pts_mask.astype(np.float32)
        I[ np.isnan(I) ] = 0
        I = np.copy(I)
        Iinit = np.copy(I)

        for ii in range(self.reps):
            #print("IDW iteration ",ii )
            I2 = scipy.signal.convolve2d( I, self.K, mode="same" )
            mask2 = scipy.signal.convolve2d( mask, self.K, mode="same" )
            I2 = I2 / (mask2+1E-9)
            mask = np.sign(mask2)
            I = orig_pts_mask*Iinit + (1-orig_pts_mask)*I2

        I[ final_mask==0 ] = np.nan

        return I, final_mask

