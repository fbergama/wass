import matplotlib.pyplot as plt
import numpy as np

def plot_spectrum( f, S, filename ):
    plt.rcParams['text.usetex'] = True
    plt.rcParams.update({'font.size': 25})

    maxfreq = f[ np.argmax( S ) ]
    maxenergy = np.amax(S)

    fig, ax = plt.subplots( figsize=(16,9) )
    ax.loglog( f, S, linewidth=3, c="k" )

    ax.grid(which='minor')
    ax.set_ylabel("$S (m^2 s)$")
    ax.set_xlabel("$f_a (Hz)$")
    ax.set_ylim( (maxenergy*1E-4, maxenergy*3))



    ax.set_xticks([0.1, maxfreq, 0.5, 0.9], labels=["0.1", "%1.2f"%maxfreq, "0.5", "0.9"])
    plt.axvline(x = maxfreq, color='b', linestyle='dashed')
    #plt.axvline(x = 0.80, color='b', linestyle='dashed')
    #plt.axvline(x = 0.85, color='b', linestyle='dashed')
    #plt.axvline(x = 0.90, color='b', linestyle='dashed')
    #ax.legend()
    ax.set_title("Frequency spectrum (peak @ %2.3f Hz)"%maxfreq)
    fig.tight_layout()
    print("Saving ",filename)
    fig.savefig(filename)
