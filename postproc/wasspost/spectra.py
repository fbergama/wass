
import numpy as np
import scipy
import scipy.signal
from tqdm import tqdm


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
