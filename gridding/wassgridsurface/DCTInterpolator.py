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
from scipy.fftpack import dct, idct
import torch
from tqdm import tqdm


class DCTInterpolator:

    def __init__( self, img_width, img_height, Nfreqs=150 ):
        C = dct(np.eye(img_height), type=3, norm='ortho')
        self.Dc = torch.tensor( C, requires_grad=True, dtype=torch.float )
        self.Nfreqs = Nfreqs
        self.height = img_height
        self.width = img_width


    def __call__( self, I ):
        orig_pts_mask = 1-np.isnan(I).astype(np.uint8)
        mask = orig_pts_mask.astype(np.float32)
        I[ np.isnan(I) ] = 0
        I = np.copy(I)

        Dc = self.Dc
        Iorig = torch.tensor(I, dtype=torch.float)
        Imask = torch.tensor(mask, dtype=torch.float)
        x = torch.rand( (self.Nfreqs,self.Nfreqs), requires_grad=True )
        xprev = torch.clone(x)

        optimizer = torch.optim.Rprop( [x], lr=5.0 )
        #optimizer = torch.optim.LBFGS( [x], lr=0.5, line_search_fn="strong_wolfe" )
        MAX_ITERS = 500
        TOLERANCE_CHANGE=1E-4
        REGULARIZER_ALPHA=9E-7

        for ii in range(MAX_ITERS):

            def closure():
                optimizer.zero_grad()
                full_signal = torch.nn.functional.pad( x, (0,self.height-self.Nfreqs,0,self.width-self.Nfreqs), "constant", 0 )
                Irec = Dc.T @ full_signal @ Dc
                loss = torch.sum( torch.square(Irec - Iorig)*Imask )/torch.sum(Imask) + REGULARIZER_ALPHA*torch.linalg.vector_norm( x, ord=1 )
                loss.backward()
                return loss

            optimizer.step( closure )

            if ii%50 == 0:
                loss = closure()
                fdelta = torch.max( torch.abs(x-xprev)).item()
                tqdm.write("Loss: %f, f max change: %f, reg: %f"%(loss.item(),fdelta,torch.linalg.vector_norm( x, ord=1 ).item() ) )
                if fdelta < TOLERANCE_CHANGE:
                    print("Reached min tolerance change, exiting")
                    break

            xprev = torch.clone(x)


        full_signal = torch.nn.functional.pad( x, (0,self.height-self.Nfreqs,0,self.width-self.Nfreqs), "constant", 0 )
        Irec = Dc.T @ full_signal @ Dc
        Irec = torch.clone(Irec).detach().numpy()
        return  Irec, np.ones_like( mask )
