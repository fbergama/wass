import tensorflow as tf
import tensorflow_addons as tfa
import numpy as np
import scipy.signal
import cv2 as cv



def derivative_of_Gaussian_win( N, sigma=1.0 ):
    assert N%2==1
    x = np.expand_dims( np.array( scipy.signal.windows.gaussian(N, sigma ) ), axis=0 ).astype(np.double)
    x = x.T @ x
    dx = np.gradient(x,axis=1)
    dy = np.gradient(x,axis=0)
    return dx, dy


class TFVariationalRefinement:
    def __init__( self, I0, I1, Rp2c, Tp2c, P0cam, P1cam, XX, YY, baseline, mask  ):

        self.XXshape = XX.shape

        self.Xpts = tf.constant( np.expand_dims( XX.flatten(), axis=0 )/baseline, dtype=tf.float32 )
        self.Ypts = tf.constant( np.expand_dims( YY.flatten(), axis=0 )/baseline, dtype=tf.float32 )

        self.Rp2c = tf.constant( Rp2c, dtype=tf.float32 )
        self.Tp2c = tf.constant( Tp2c, dtype=tf.float32 )

        self.P0cam = tf.constant( P0cam, dtype=tf.float32 )
        self.P1cam = tf.constant( P1cam, dtype=tf.float32 )

        self.I0 = tf.constant(I0[np.newaxis,:,:,np.newaxis], dtype=tf.float32 )
        self.I1 = tf.constant(I1[np.newaxis,:,:,np.newaxis], dtype=tf.float32 )

        self.mask = tf.constant( mask, dtype=tf.float32 )
        self.mask_reduced = tf.constant( cv.erode( mask, np.ones((11,11))).astype(np.float32) )

        sobel_x, sobel_y = derivative_of_Gaussian_win( 7, sigma=0.8 )
        sobel_kernels = np.concatenate( [sobel_x[...,np.newaxis,np.newaxis], sobel_y[...,np.newaxis,np.newaxis] ], axis=-1 )
        self.sobel_kernels = tf.constant( sobel_kernels, dtype=tf.float32 )
        

    def sample_images( self, Z ):
        p3d = tf.concat( [self.Xpts, self.Ypts, tf.reshape(Z,self.Xpts.shape) ], axis=0 )
        p3d_cam = self.Rp2c @ p3d + self.Tp2c

        surfpts_cam0 = self.P0cam @ tf.concat( [p3d_cam, tf.ones((1,p3d_cam.shape[1]))], axis=0)
        surfpts_cam0 /= surfpts_cam0[2,:]

        surfpts_cam1 = self.P1cam @ tf.concat( [p3d_cam, tf.ones((1,p3d_cam.shape[1]))], axis=0)
        surfpts_cam1 /= surfpts_cam1[2,:]

        cam0_p2 = tf.transpose( surfpts_cam0[:2,...] )
        cam1_p2 = tf.transpose( surfpts_cam1[:2,...] )

        I0_samp = tf.reshape( tfa.image.interpolate_bilinear( self.I0, tf.expand_dims( cam0_p2, axis=0 ), indexing="xy" ),
                            self.XXshape ) * self.mask

        I1_samp = tf.reshape( tfa.image.interpolate_bilinear( self.I1, tf.expand_dims( cam1_p2, axis=0 ), indexing="xy" ),
                            self.XXshape ) * self.mask
        
        return I0_samp, I1_samp, cam0_p2, cam1_p2


    def compute_Z_gradient( self, Z ):

        Z_grad = tf.nn.conv2d( tf.expand_dims( tf.expand_dims(Z,axis=0), axis=-1), self.sobel_kernels, strides=1, padding="SAME" )
        Z_dx = Z_grad[0,:,:,0]#*self.mask_reduced
        Z_dy = Z_grad[0,:,:,1]#*self.mask_reduced
        return Z_dx, Z_dy


    def compute_loss( self, Z ):
        I0_samp, I1_samp,_,_ = self.sample_images( Z )

        I0_mean = tf.reduce_mean(I0_samp)
        I0_std = tf.math.reduce_std(I0_samp)
        I1_mean = tf.reduce_mean(I1_samp)
        I1_std = tf.math.reduce_std(I1_samp)

        #I0_samp_norm = (I0_samp-I0_mean) / I0_std
        #I1_samp_norm = (I1_samp-I1_mean) / I1_std
        I0_samp_norm = (I0_samp)/255.0
        I1_samp_norm = (I1_samp)/255.0
        
        Z_dx, Z_dy = self.compute_Z_gradient( Z )
        
        data_loss = tf.reduce_mean( tf.square( I0_samp_norm-I1_samp_norm ) )
        smoothness_loss = tf.reduce_mean( tf.square(Z_dx) + tf.square(Z_dy) )
        
        #print("data: ", data_loss )
        #print("smoothness: ",smoothness_loss)
        #alpha=100.0

        return data_loss, smoothness_loss


    def optimize( self, Zinit, max_iters=400, alpha=10 ):
        print("Zinit shape", Zinit.shape )

        Zfullshape = Zinit.shape
        Zinit = cv.resize(Zinit, (Zinit.shape[0]//2, Zinit.shape[1]//2), interpolation=cv.INTER_LINEAR)
        #Z = tf.Variable( Zinit, dtype=tf.float32 )
        #Z = tf.Variable( np.zeros( (Zinit.shape[0]//8, Zinit.shape[1]//8), dtype=np.float32) )
        Z = tf.Variable( Zinit, dtype=tf.float32 )


        opt = tf.keras.optimizers.Adam(learning_rate=1E-3, epsilon=1E-7 )

        myself = self
        def energy():
            Zresized = tf.image.resize( Z[tf.newaxis,:,:,tf.newaxis], Zfullshape )

            dloss, sloss = myself.compute_loss( Zresized[0,...,0] )
            #print("Data loss: ",dloss.numpy())
            return dloss + alpha*sloss

        prev_loss = energy().numpy()
        print("Optimizing: Initial loss: %3.5f"%prev_loss )
        print("===============================================")
        print(" It     Loss     DLoss")
        for ii in range(max_iters):
            step_count = opt.minimize(energy, [Z]).numpy()
            if ii%10==0:
                current_loss = energy().numpy()
                delta_loss =  np.abs( prev_loss-current_loss ) 
                print("%05d  %3.5f   %3.5f"%(ii,current_loss,delta_loss))
                #if delta_loss<1E-6:
                #    break
                prev_loss = current_loss
        
        Zresized = tf.image.resize(Z[tf.newaxis,:,:,tf.newaxis], Zfullshape )
        return (Zresized[0,...,0]*self.mask).numpy()



