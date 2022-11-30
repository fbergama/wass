# WASS

WASS (Waves Acquisition Stereo System) is an optimized stereo processing pipeline for sea waves 3D reconstruction.


It was developed by [Filippo Bergamasco](http://www.dsi.unive.it/~bergamasco/) as a joint-collaboration between [Università Ca'Foscari di Venezia](http://www.unive.it) and [CNR ISMAR](http://www.ismar.cnr.it). It is a result of more than 6 years of active research with [Alvise Benetazzo](http://www.ismar.cnr.it/people/benetazzo-alvise) (CNR-ISMAR) in the field of accurate 3D surface reconstruction of sea waves.


## Official project page

[https://sites.google.com/unive.it/wass](https://sites.google.com/unive.it/wass)


## Installation instructions 

Since WASS 1.6 there is no need to compile OpenCV from scratch. Just install the
latest version with your favourite package manager.

On Linux Ubuntu-like:

```
$ sudo apt install libopencv-dev
``` 

On OSX:

```
$ brew install opencv
```

### Steps:


1. Download WASS and checkout the latest WASS branch:

```
$ git clone https://github.com/fbergama/wass.git
$ cd wass
$ git checkout v_1.7
$ git submodule update --init
```

2. Prepare for build

```
$ mkdir build
$ cd build
```

3. Build and install

```
$ cmake ../src
$ make install
```

4. Suggested: add the `../dist/bin` directory to your path!


5. Test if it works

```
wass_stereo  v. 1.7_heads/v_1.7-0-g4177b1f
----------------------------------------------
 [Release] Linux-5.15.0-46-generic - GNU, OpenCV 4.5.4

Usage:
wass_stereo [--genconfig] <config_file> <workdir> [--measure] [--rectify-only]

Not enough arguments, aborting.
```


## How to run it

I suggest you to use the new *wasscli command line interface* to automate the WASS pipeline execution:

[https://pypi.org/project/wasscli/](https://pypi.org/project/wasscli/)



## Post processing

To grid the reconstructed point clouds consider the new wassgridsurface tool:

[https://pypi.org/project/wassgridsurface/](https://pypi.org/project/wassgridsurface/)



## License

```
Copyright (C) 2016-2022 Filippo Bergamasco 

WASS is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

WASS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```



