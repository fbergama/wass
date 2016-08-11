---
title: "Testing"
layout: "default"
isPage: true
menuOrder: 2
---

# Testing the WASS pipeline

After the WASS installation is highly recommended to test the whole pipeline to verify its correct operation. Tests are performed by some Matlab
scripts against a set of synthetically generated stereo frames. Please be aware that the processing speed during the tests can be lower than the normal mode of operation described in the [Getting Started](getting_started.html) section. 


## Downloading the test data

Download the test data from
[http://www.dsi.unive.it/wass/WASS_TEST.zip](http://www.dsi.unive.it/wass/WASS_TEST.zip)
and unzip the downloaded package into the ```<WASS_ROOT>/test``` directory.

Your ```<WASS_ROOT>/test``` directory should look similar to this:

```
├── plot_sequence.m
├── test_pipeline.m
├── verify_matcher.m
├── verify_meshes.m
├── WASS_TEST
│   ├── gendata
│   │   ├── intrinsics_00000000.xml
│   │   ├── intrinsics_00000001.xml
│   │   ├── matchdata_full.txt
│   │   └── plane.txt
│   ├── synth
│   │   ├── 3D
│   │   │   ├── 000000_3d.ply
│   │   │   ├── 000001_3d.ply
│   │   │   ├── 000002_3d.ply
│   │   │   ├── 000003_3d.ply
│   │   │   ├── 000004_3d.ply
│   │   │   ├── 000005_3d.ply
│   │   │   ├── 000006_3d.ply
│   │   │   ├── 000007_3d.ply
│   │   │   ├── 000008_3d.ply
│   │   │   ├── 000009_3d.ply
│   │   │   ├── 000010_3d.ply
│   │   │   ├── 000011_3d.ply
│   │   │   ├── 000012_3d.ply
│   │   │   ├── 000013_3d.ply
│   │   │   ├── 000014_3d.ply
│   │   │   ├── 000015_3d.ply
│   │   │   ├── 000016_3d.ply
│   │   │   ├── 000017_3d.ply
│   │   │   ├── 000018_3d.ply
│   │   │   └── 000019_3d.ply
│   │   ├── config
│   │   │   ├── distortion_00.xml
│   │   │   ├── distortion_01.xml
│   │   │   ├── ext_R.xml
│   │   │   ├── ext_T.xml
│   │   │   ├── intrinsics_00.xml
│   │   │   ├── intrinsics_01.xml
│   │   │   ├── matcher_config.txt
│   │   │   └── stereo_config.txt
│   │   └── input
│   │       ├── cam0
│   │       │   ├── 000000_0000000000000_1.tif
│   │       │   ├── 000001_0000000000000_1.tif
│   │       │   ├── 000002_0000000000000_1.tif
│   │       │   ├── 000003_0000000000000_1.tif
│   │       │   ├── 000004_0000000000000_1.tif
│   │       │   ├── 000005_0000000000000_1.tif
│   │       │   ├── 000006_0000000000000_1.tif
│   │       │   ├── 000007_0000000000000_1.tif
│   │       │   ├── 000008_0000000000000_1.tif
│   │       │   └── 000009_0000000000000_1.tif
│   │       └── cam1
│   │           ├── 000000_0000000000000_2.tif
│   │           ├── 000001_0000000000000_2.tif
│   │           ├── 000002_0000000000000_2.tif
│   │           ├── 000003_0000000000000_2.tif
│   │           ├── 000004_0000000000000_2.tif
│   │           ├── 000005_0000000000000_2.tif
│   │           ├── 000006_0000000000000_2.tif
│   │           ├── 000007_0000000000000_2.tif
│   │           ├── 000008_0000000000000_2.tif
│   │           └── 000009_0000000000000_2.tif
│   └── W07
│       ├── config
│       │   ├── distortion_00.xml
│       │   ├── distortion_01.xml
│       │   ├── ext_R.xml
│       │   ├── ext_T.xml
│       │   ├── intrinsics_00.xml
│       │   ├── intrinsics_01.xml
│       │   ├── matcher_config.txt
│       │   └── stereo_config.txt
│       └── input
│           ├── cam0
│           │   ├── 000000_0000000000000_01.tif
│           │   ├── 000001_0000000083342_01.tif
│           │   ├── 000002_0000000166652_01.tif
│           │   ├── 000003_0000000249994_01.tif
│           │   ├── 000004_0000000333336_01.tif
│           │   └── 000005_0000000416678_01.tif
│           └── cam1
│               ├── 000000_0000000000000_02.tif
│               ├── 000001_0000000083341_02.tif
│               ├── 000002_0000000166650_02.tif
│               ├── 000003_0000000249991_02.tif
│               ├── 000004_0000000333333_02.tif
│               └── 000005_0000000416674_02.tif
└── WASS_TEST.zip
```


## Running the tests

Open Matlab and change the working directory to ```<WASS_ROOT>/test```. Then, run the matlab script ```test_pipeline.m```. After a while, depending
on your hardware, you should get the following output:

```
***************************************************
**  Verifying 3D point clouds                 *****
***************************************************
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000000_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000001_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000002_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000003_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000004_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000005_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000006_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000007_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000008_wd/........... test passed
Loading compressed data...
Sampling gt elevations...
/home/fibe/WASS/test/output/000009_wd/........... test passed
***************************************************
 ALL TESTS OK!
>> 

```

indicating that all the tests were successful.

