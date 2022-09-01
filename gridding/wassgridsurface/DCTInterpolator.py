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

    def __init__( self, img_width, img_height, alg_options ):

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using {self.device} device")

        C = dct(np.eye(img_height), type=3, norm='ortho')
        self.Dc = torch.tensor( C, requires_grad=True, dtype=torch.float, device=self.device )
        self.height = img_height
        self.width = img_width
        self.Nfreqs = DCTInterpolator._get_setting_helper(alg_options,"Nfreqs")
        self.MAX_ITERS = DCTInterpolator._get_setting_helper(alg_options,"MAX_ITERS")
        self.TOLERANCE_CHANGE = DCTInterpolator._get_setting_helper(alg_options,"TOLERANCE_CHANGE")
        self.REGULARIZER_ALPHA = DCTInterpolator._get_setting_helper(alg_options,"REGULARIZER_ALPHA")
        self.LEARNING_RATE = DCTInterpolator._get_setting_helper(alg_options,"LEARNING_RATE")

        print("")
        print("DCT Interpolator settings:")
        print("           Nfreqs: %d"%self.Nfreqs)
        print("REGULARIZER_ALPHA: %f"%self.REGULARIZER_ALPHA)
        print("    LEARNING_RATE: %f"%self.LEARNING_RATE)
        print(" TOLERANCE_CHANGE: %f"%self.TOLERANCE_CHANGE)
        print("        MAX_ITERS: %d"%self.MAX_ITERS)
        print("")


    def _get_setting_helper( options_dict, setting_name ):
        default_settings = {
                "Nfreqs": 150,
                "MAX_ITERS": 500,
                "TOLERANCE_CHANGE": 1E-4,
                "REGULARIZER_ALPHA": 8E-7,
                "LEARNING_RATE": 5.0
                }

        if not options_dict is None:
            if setting_name in options_dict:
                return options_dict[setting_name] if not (options_dict[setting_name] is None) else default_settings[setting_name]

        return default_settings[setting_name]


    def __call__( self, I ):
        orig_pts_mask = 1-np.isnan(I).astype(np.uint8)
        mask = orig_pts_mask.astype(np.float32)
        I[ np.isnan(I) ] = 0
        I = np.copy(I)

        Dc = self.Dc
        Iorig = torch.tensor(I, dtype=torch.float, device=self.device )
        Imask = torch.tensor(mask, dtype=torch.float, device=self.device )
        x = torch.rand( (self.Nfreqs,self.Nfreqs), requires_grad=True, device=self.device )
        xprev = torch.clone(x)

        optimizer = torch.optim.Rprop( [x], lr=self.LEARNING_RATE )
        #optimizer = torch.optim.LBFGS( [x], lr=0.5, line_search_fn="strong_wolfe" )

        REGULARIZER_ALPHA = self.REGULARIZER_ALPHA
        TOLERANCE_CHANGE = self.TOLERANCE_CHANGE

        for ii in range(self.MAX_ITERS+1):

            def closure( output_all_loss_components=False ):
                optimizer.zero_grad()
                full_signal = torch.nn.functional.pad( x, (0,self.height-self.Nfreqs,0,self.width-self.Nfreqs), "constant", 0 )
                Irec = Dc.T @ full_signal @ Dc
                data_loss = torch.sum( torch.square(Irec - Iorig)*Imask )/torch.sum(Imask)
                regularizer_loss = torch.linalg.vector_norm( x, ord=1 )
                loss = data_loss + REGULARIZER_ALPHA*regularizer_loss
                loss.backward()
                if output_all_loss_components:
                    return loss, data_loss, regularizer_loss

                return loss

            optimizer.step( closure )

            if ii%50 == 0:
                loss, data_loss, regularizer_loss = closure( output_all_loss_components=True )
                fdelta = torch.max( torch.abs(x-xprev)).item()
                tqdm.write("Iteration %05d - Loss: %6.5f =  data: %6.5f  reg: %6.5f   F max delta: %3.5f"%(ii, loss.item(), data_loss.item(), regularizer_loss.item(), fdelta ) )
                if fdelta < TOLERANCE_CHANGE:
                    print("Reached min tolerance change, exiting")
                    break

            xprev = torch.clone(x)


        full_signal = torch.nn.functional.pad( x, (0,self.height-self.Nfreqs,0,self.width-self.Nfreqs), "constant", 0 )
        Irec = Dc.T @ full_signal @ Dc
        Irec = torch.clone(Irec).cpu().detach().numpy()
        return  Irec, np.ones_like( mask )
