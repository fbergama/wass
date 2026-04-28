
import numpy as np
import scipy
import scipy.signal
from tqdm import tqdm
from scipy.fft import fft2, ifft2, fftshift, ifftshift, fftfreq


def compute_spectrum( data, dt, nperseg=512, rangespan=5, scale=1.0 ):
    """Computes frequency spectrum in a central region of the 3D+time data cube
    Parameters:
        data: NxWxH  tensor (can be a NetCDF dataset)
        dt: time delta

    Optional:
        nperseg: length of each segment in scipy.signal.csd
        rangespan: range around the central point where to sample the timeserie
        scale: scale to apply to data before computing the spectrum

    Returns:
        f: frequencies
        S: power spectra
        timeserie: elevation tiemserie at grid cente
    """

    nsamples = data.shape[0]
    gridsize = data.shape[1:3]
    halfgridsize_i = gridsize[0]//2
    halfgridsize_j = gridsize[1]//2
    valid_samples_i = range( halfgridsize_i-rangespan, halfgridsize_i+rangespan+1 )
    valid_samples_j = range( halfgridsize_j-rangespan, halfgridsize_j+rangespan+1 )

    print(f"averaging frequency spectrum on grid points (row,cols) [{halfgridsize_i-rangespan} ... {halfgridsize_i+rangespan}], [{halfgridsize_j-rangespan} ... {halfgridsize_j+rangespan}]")
    print(f"FFT size: {nperseg}")

    timeserie = scale * data[:,halfgridsize_i,halfgridsize_j]
    timeserie = timeserie - np.mean(timeserie)
    f, S = scipy.signal.csd(timeserie, timeserie, 1.0/dt, nperseg=nperseg )

    for ii in tqdm(valid_samples_i):
        for jj in valid_samples_j:
            timeserie_neigh = scale*data[:,ii,jj]
            timeserie_neigh = timeserie_neigh - np.mean(timeserie_neigh)
            _, S_neig = scipy.signal.csd(timeserie_neigh, timeserie_neigh, 1.0/dt, nperseg=nperseg )
            S += S_neig
    
    S = S / float( np.size(valid_samples_i)*np.size(valid_samples_j) + 1)

    return f, S, timeserie



def compute_3D_spectrum( data, du:float, dt:float, segments:int=8, datascale:float=1.0 ):

    dx=du
    dy=du
    
    # Extract a central part of the Zcube
    #N = data.shape[1]//2
    N = data.shape[1]*2//3
    segments = 10 
    
    sequence_length = data.shape[0]
    Nt = int(sequence_length / segments)
    if Nt%2 > 0:
        Nt+=1
    seg_shift = int(Nt/2)
    
    Zcube_mr = data.shape[1] // 2
    Zcube_mc = data.shape[2] // 2
    r_start, r_end = Zcube_mr-N//2-20, Zcube_mr+N//2-20+1
    c_start, c_end = Zcube_mc-N//2, Zcube_mc+N//2+1 
    
    Nx = r_end - r_start
    Ny = c_end - c_start
    print(f"         rows: {r_start} ... {r_end}")
    print(f"         cols: {c_start} ... {c_end}")
    print("         du,dt: ",dx,dt)
    print("      Nx,Ny,Nt: ",Nx,Ny,Nt)
    
    kx_max=(2.0*np.pi/dx)/2.0
    ky_max=(2.0*np.pi/dy)/2.0
    f_max= (1.0/dt)/2.0
    dkx=2.0*np.pi/(dx*np.floor(Nx/2.0)*2.0)
    dky=2.0*np.pi/(dy*np.floor(Ny/2.0)*2.0)
    df =1.0/(dt*np.floor(Nt/2.0)*2.0)
    
    
    assert( Nx%2 != 0)
    assert( Ny%2 != 0)
    assert( Nt%2 == 0)
    
    kx=np.arange(-kx_max,kx_max+dkx,dkx)
    ky=np.arange(-ky_max,ky_max+dky,dky)
    
    if Nt%2==0:
        f=np.arange(-f_max, f_max, df)
    else:
        f=np.arange(-f_max, f_max+df, df)

    
    KX, KY = np.meshgrid( kx, ky )
    dkx=kx[3]-kx[2]
    dky=ky[3]-ky[2]
    KXY=np.sqrt(KX**2+KY**2)

    
    print(f" Kx  min..max: {np.amin(kx)} ... {np.amax(kx)} ")
    print(f" Ky  min..max: {np.amin(kx)} ... {np.amax(kx)} ")
    print(f"  f  min..max: {np.amin( np.abs(f))} ... {np.amax(f)} (Hz)")
    
    hanningx = scipy.signal.windows.hann(KX.shape[0])
    hanningy = scipy.signal.windows.hann(KX.shape[1])
    hanningt = scipy.signal.windows.hann(Nt)
    
    Win3Dhann = np.tile( np.expand_dims( hanningx, axis=-1) * hanningy, (Nt,1,1) ) *  np.tile( np.expand_dims( np.expand_dims( hanningt, axis=-1 ), axis=-1 ), (1, KX.shape[0], KX.shape[1]) )
    assert( KX.shape == Win3Dhann.shape[1:] )
    
    #  window correction factors
    wc2x = 1.0/np.mean(hanningx**2)
    wc2y = 1.0/np.mean(hanningy**2)
    wc2t = 1.0/np.mean(hanningt**2)
    wc2xy  = wc2x *wc2y
    wc2xyt = wc2xy*wc2t
    
    print("   dt, dkx, dky, df: ",dt, dkx, dky, df)
    
    # Fix for rounding errors
    r_end = r_start + Win3Dhann.shape[1]
    c_end = c_start + Win3Dhann.shape[2]
    
    S_welch = np.zeros_like( Win3Dhann )
    n_samples = 0
    print("   Computing 3D FFT via Welch's method... ", end="")


    #chunks = [ (segm, rshift, cshift) for segm in range(segments*2) for rshift in range(-20,40,20) for cshift in range(-20,40,20)]
    
    for ii in tqdm(range(segments*2)):
        rshift = 0
        cshift = 0
    #for chunk in tqdm(chunks):
        #ii,rshift,cshift = chunk
        #print("Welch sample %d/%d"%(ii+1,segments*2))
        Zcube_small = np.array( data[(ii*seg_shift):(ii*seg_shift+Nt), r_start+rshift:r_end+rshift, c_start+cshift:c_end+cshift ] ) * datascale

        # Remove nans
        Zcube_small = np.where(np.isnan(Zcube_small), np.nanmean(Zcube_small, axis=0), Zcube_small)
        
        if Zcube_small.shape[0] != Nt:
            break
            
        Zcube_w = (Zcube_small - np.mean(Zcube_small) ) * Win3Dhann
        
        S = np.fft.fftshift( np.fft.fftn( Zcube_w, norm="ortho" ) )
        S /= (S.shape[0]*S.shape[1]*S.shape[2])
        S = np.abs(S)**2 / (dkx*dky*df)
        #-----------------------------
        #%%%%% corrects for window
        #----------------------------
        #%% FABIEN
        S *= wc2xyt
        
        # Store
        S_welch += S    
        n_samples += 1
        
    S_welch /= n_samples    

    print(" Done!")
    return S_welch, KX, KY, f





def filter_2d_butterworth(surface, du, cutoff_fs, order ):
    """
    Applies a radially symmetric 2D Butterworth lowpass filter to a surface.
    
    Parameters:
    - surface: 2D numpy array (W x H) representing the wave elevation or normal field.
    - du: Spatial resolution in meters/pixel.
    - cutoff_fs: Spatial cutoff frequency in cycles/meter.
    - order: Filter order (4th order spatial mimics 8th order temporal).
    
    Returns:
    - filtered_surface: 2D numpy array of the filtered surface (real-valued).
    """
    W, H = surface.shape
    
    # 1. Transform the surface to the 2D frequency domain
    F_surface = fft2(surface)
    F_surface_shifted = fftshift(F_surface) # Move zero-frequency (DC) to the center
    
    # 2. Create the 1D frequency axes (cycles/meter)
    fx = fftshift(fftfreq(W, d=du))
    fy = fftshift(fftfreq(H, d=du))
    
    # 3. Create a 2D radial frequency grid
    FX, FY = np.meshgrid(fy, fx) # Note: meshgrid uses (y, x) ordering by default
    R = np.sqrt(FX**2 + FY**2)
    
    # 4. Construct the 2D Butterworth transfer function
    # Formula: H(R) = 1 / sqrt(1 + (R / f_c)^(2n))
    # To avoid division by zero at the DC component, we add a tiny epsilon if cutoff_fs is 0
    butterworth_filter = 1.0 / np.sqrt(1.0 + (R / cutoff_fs)**(2 * order))
    
    # 5. Apply the filter element-wise in the frequency domain
    F_filtered_shifted = F_surface_shifted * butterworth_filter
    
    # 6. Transform back to the spatial domain
    F_filtered = ifftshift(F_filtered_shifted)
    filtered_surface = np.real(ifft2(F_filtered))
    
    return filtered_surface
