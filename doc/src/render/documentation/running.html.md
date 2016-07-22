---
title: "Getting started"
layout: "default"
isPage: true
menuOrder: 3
---

# Getting Started 

Assuming that your WASS installation was successful this tutorial will let you run the pipeline to reconstruct some sample real-world sample data. This should help you acquire familiarity with some of the concepts will be expanded in the following parts of the documentation. 

## WASS architecture

WASS is composed by a set of program executables that perform, in sequence, specific steps of the reconstruction process (that's why is called a pipeline).
An external process, called ```WASSjs```, controls the execution of such pipeline to provide a simple graphical (web-based) user interface to the user and exploit the best possible parallelism offered by modern multi-core machines.

At the moment, WASS is composed by the following programs (you can see them under ```<WASS_ROOT>/dist/bin/```):

|  Name                    |   Task                                            |
|--------------------------|---------------------------------------------------|
| ```wass_prepare```       |  Takes a couple of stereo images as input and place them into a proper "working directory". Images are also undistorted according to the provided intrinsic calibration data   |
| ```wass_match```         |  Matches corresponding features of a stereo image pair. After the matching, the Essential matrix is recovered and the extrinsic parameters between the two cameras are factorized (up to translation scale). Matched features are filtered according to epipolar consistency.                  |
| ```wass_autocalibrate``` |  Matches between multiple stereo frames are all collected and Sparse Bundle Adjustment (SBA) is used to simultaneously optimize the extrinsic camera parameters and the 3D points triangulated from all the matches. Final extrinsics are saved to all the workspaces.  |
| ```wass_stereo```        |  Performs dense stereo 3D reconstruction on a given stereo frame. After the reconstruction, multiple filters are applied to the point cloud to remove outliers. Finally, a plane is robustly fitted to the point cloud. |


```wass_match``` and ```wass_autocalibrate``` are only used if auto-calibration is needed (ie. the extrinsic parameters are unknown). With the exception of ```wass_autocalibrate``` that operates on all the working directories simultaneously, each program loads its data from a specific working directory and saves its results into that.

The "working directory" is a basic unit of work that holds two corresponding stereo frames together with the computed reconstruction data (point cloud, plane parameters, etc). All the working directories of a stereo sequence follows the following name convention:

```000000_wd/``` for the first frame, ```000001_wd/``` for the second, and so on. No assumption are made on the temporal difference between two frames that are all considered independent and hence processed in parallel.
 

## How to run it

If you skipped the testing step, please download the data from
[http://www.dsi.unive.it/~bergamasco/tmp/WASS_TEST.zip](http://www.dsi.unive.it/~bergamasco/tmp/WASS_TEST.zip)
and unzip the downloaded package into the ```<WASS_ROOT>/test``` directory. That package also contains the sample stereo sequence we are going to process.

In ```<WASS_ROOT>/test/W07/``` you can see how an hypotetical image sequence should be prepared for the reconstruction. The ```config/``` subdirectory contains the calibration data (at least the intrinsics) and some configuration files. The ```input/``` subdirectory contains the image data, divided between the two cameras, whose name convention follows the rule:

```
<sequence_number (6 digits)>_<timestamp (13 digits)>_<camera number (2 digits)>.tif
```

In our case, the ```input/``` subdirectory looks as follows:

```
├── cam0
│   ├── 000000_0000000000000_01.tif
│   ├── 000001_0000000083342_01.tif
│   ├── 000002_0000000166652_01.tif
│   ├── 000003_0000000249994_01.tif
│   ├── 000004_0000000333336_01.tif
│   └── 000005_0000000416678_01.tif
│   └──  ...
└── cam1
    ├── 000000_0000000000000_02.tif
    ├── 000001_0000000083341_02.tif
    ├── 000002_0000000166650_02.tif
    ├── 000003_0000000249991_02.tif
    ├── 000004_0000000333333_02.tif
    └── 000005_0000000416674_02.tif
    └──  ...

```

After the installation, WASS is pre-configured to reconstruct the sequence inside ```<WASS_ROOT>/test/W07/```. To launch WASSjs (the pipeline monitor), execute the following:

### On Linux / Mac OSX

Open two terminals. On the first one type:

```
$ cd <WASS_ROOT>/WASSjs/ext/
$ ./launch_redis.sh
```

on the second one enter the commands:

```
$ cd <WASS_ROOT>/WASSjs
$ node Wass.js
```

WASSjs is now running. Keep both the terminals open during its whole execution.


### On Windows

Open ```<WASS_ROOT>/WASSjs``` and double click on ```launch_wassjs.bat```. Two command prompts will appear to show debug informations during WASS execution.


<br/>

### Step 1 - Access WASSjs

Open your browser at the address [http://localhost:8080](http://localhost:8080) to access the wass monitor:


### Step 2 - Prepare the working directories

From the top menu, click on ```Prepare```. You should see the working progress in the


### Step 3 - Auto-calibrate the stereo cameras


### Step 4 - Run the 3D reconstruction



