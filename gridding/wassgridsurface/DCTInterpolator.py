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
import math
import numpy as np
from scipy.fftpack import dct, idct
import torch
from tqdm import tqdm

try:
    import tomllib                 # Python >= 3.11
except ModuleNotFoundError:
    try:
        import tomli as tomllib    # optional backport, used only if present
    except ModuleNotFoundError:
        tomllib = None


def _minimal_toml_loads(text):
    """Very small TOML reader covering the subset written by DCTInterpolator
    (dotted section headers, scalar key = value, # comments). Only used when
    neither tomllib nor tomli is available."""
    root = {}
    current = root
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            current = root
            for part in line[1:-1].split("."):
                current = current.setdefault(part.strip(), {})
            continue
        key, sep, value = line.partition("=")
        if not sep:
            raise ValueError("Malformed TOML line: %s" % raw)
        current[key.strip()] = _minimal_toml_value(value.strip())
    return root


def _minimal_toml_value(s):
    if s in ("true", "false"):
        return s == "true"
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "\"'":
        return s[1:-1]
    try:
        return int(s)
    except ValueError:
        return float(s)


class DCTInterpolator:

    DEFAULT_SETTINGS = {
            "Nfreqs": 150,
            "MAX_ITERS": 500,
            "TOLERANCE_CHANGE": 1E-4,
            "REGULARIZER_ALPHA": 8E-7,
            "LEARNING_RATE": 5.0
            }

    SETTING_TYPES = {
            "Nfreqs": int,
            "MAX_ITERS": int,
            "TOLERANCE_CHANGE": float,
            "REGULARIZER_ALPHA": float,
            "LEARNING_RATE": float
            }

    def __init__( self, img_width, img_height, alg_options, seed=None ):
        """seed: optional int. If None (default) the optimizer starts from a
        randomly initialized coefficient matrix, so successive runs differ. If
        set, every __call__ re-seeds its own generator with it, making the
        reconstruction reproducible regardless of frame order or thread
        scheduling (the gridder runs frames in parallel)."""

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using {self.device} device")
        self.seed = None if seed is None else int(seed)

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
        print("             seed: %s"%("random" if self.seed is None else str(self.seed)))
        print("")

    @staticmethod
    def _get_setting_helper( options_dict, setting_name ):
        default_settings = DCTInterpolator.DEFAULT_SETTINGS

        if not options_dict is None:
            if setting_name in options_dict:
                return options_dict[setting_name] if not (options_dict[setting_name] is None) else default_settings[setting_name]

        return default_settings[setting_name]

    # ------------------------------------------------------------------
    #  Serialization
    # ------------------------------------------------------------------

    def get_options( self ):
        """Return the algorithm options currently in use as a plain dict."""
        return { name: getattr(self, name) for name in DCTInterpolator.SETTING_TYPES }

    @staticmethod
    def _toml_scalar( v ):
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (int, np.integer)):
            return str(int(v))
        if isinstance(v, (float, np.floating)):
            v = float(v)
            if math.isnan(v):
                return "nan"
            if math.isinf(v):
                return "inf" if v > 0 else "-inf"
            r = repr(v)
            if not any( c in r for c in ".eE" ):
                r += ".0"     # keep it a TOML float, not an int
            return r
        raise TypeError("Cannot serialize %r to TOML" % type(v))

    def to_toml( self ):
        """Serialize this interpolator's configuration to a TOML string."""
        S = DCTInterpolator._toml_scalar
        lines = [ "[DCTInterpolator]",
                  "img_width = %s"  % S(self.width),
                  "img_height = %s" % S(self.height) ]
        if self.seed is not None:
            lines.append( "seed = %s" % S(self.seed) )
        lines += [ "",
                   "[DCTInterpolator.alg_options]" ]
        for name in DCTInterpolator.SETTING_TYPES:
            lines.append( "%s = %s" % (name, S(getattr(self, name))) )
        return "\n".join(lines) + "\n"

    def save( self, filename ):
        """Write this interpolator's configuration to a TOML file."""
        with open( filename, "w", encoding="utf-8" ) as f:
            f.write( self.to_toml() )

    @classmethod
    def from_dict( cls, data, seed=None ):
        """seed overrides any 'seed' recorded in the configuration."""
        section = data.get("DCTInterpolator", data)
        if "img_width" not in section or "img_height" not in section:
            raise ValueError("Missing img_width / img_height in DCTInterpolator section")

        raw_opts = section.get("alg_options", {})
        unknown = set(raw_opts) - set(cls.SETTING_TYPES)
        if unknown:
            raise ValueError("Unknown DCTInterpolator settings: %s" % ", ".join(sorted(unknown)))

        opts = { k: cls.SETTING_TYPES[k](v) for k, v in raw_opts.items() }
        if seed is None:
            seed = section.get("seed")
        return cls( int(section["img_width"]), int(section["img_height"]), opts, seed=seed )

    @classmethod
    def from_toml( cls, text, seed=None ):
        """Build a DCTInterpolator from a TOML string."""
        if tomllib is not None:
            return cls.from_dict( tomllib.loads(text), seed=seed )
        return cls.from_dict( _minimal_toml_loads(text), seed=seed )

    @classmethod
    def load( cls, filename, seed=None ):
        """Build a DCTInterpolator from a TOML file."""
        with open( filename, "rb" ) as f:
            raw = f.read()
        if tomllib is not None:
            return cls.from_dict( tomllib.loads( raw.decode("utf-8") ), seed=seed )
        return cls.from_dict( _minimal_toml_loads( raw.decode("utf-8") ), seed=seed )

    # ------------------------------------------------------------------

    def __call__( self, I, verbose=True ):
        orig_pts_mask = 1-np.isnan(I).astype(np.uint8)
        mask = orig_pts_mask.astype(np.float32)
        I[ np.isnan(I) ] = 0
        I = np.copy(I)

        Dc = self.Dc
        Iorig = torch.tensor(I, dtype=torch.float, device=self.device )
        Imask = torch.tensor(mask, dtype=torch.float, device=self.device )
        if self.seed is None:
            x = torch.rand( (self.Nfreqs,self.Nfreqs), requires_grad=True, device=self.device )
        else:
            # own generator, re-seeded per call: same init for every frame, and
            # independent of the global RNG and of parallel frame scheduling
            g = torch.Generator( device=self.device )
            g.manual_seed( self.seed )
            x = torch.rand( (self.Nfreqs,self.Nfreqs), generator=g, device=self.device )
            x.requires_grad_( True )
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
                if verbose:
                    tqdm.write("Iteration %05d - Loss: %6.5f =  data: %6.5f  reg: %6.5f   F max delta: %3.5f"%(ii, loss.item(), data_loss.item(), regularizer_loss.item(), fdelta ) )
                if fdelta < TOLERANCE_CHANGE:
                    if verbose:
                        tqdm.write("Reached min tolerance change, exiting")
                    break

            xprev = torch.clone(x)


        full_signal = torch.nn.functional.pad( x, (0,self.height-self.Nfreqs,0,self.width-self.Nfreqs), "constant", 0 )
        Irec = Dc.T @ full_signal @ Dc
        Irec = torch.clone(Irec).cpu().detach().numpy()
        return  Irec, np.ones_like( mask )
