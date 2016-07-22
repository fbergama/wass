---
title: "Installing"
layout: "default"
isPage: true
menuOrder: 1
---

# Installation instructions

WASS was designed to be portable to numerous platforms. It was developed
and actively tested on Linux but can be used on any recent versions of Mac OSX and Microsoft
Windows.

Use the following links to get detailed installation instructions for your system:

- [Linux](#linux)
- [Mac OSX](#mac-osx)
- [Windows](#windows)

## Pre-requisites

WASS relies to the following 3rd party components:

- [OpenCV](http://opencv.org) Open-Source Computer Vision Library 
- [Boost](http://www.boost.org) C++ Libraries
- [SBA](http://users.ics.forth.gr/~lourakis/sba/) Sparse Bundle Adjustment C/C++ Library
- [redis](http://redis.io) In-memory data structure store
- [node.js](https://nodejs.org) Javascript runtime

Additionally, [Matlab](www.mathworks.com) is required for some post-processing steps like surface estimation and waves spectrum recovery.

## Note for multi-core/cpu platforms

Each wass component (ie. ```wass_stereo```, ```wass_match```, etc.) runs as a
single-threaded process performing a specific task on a working directory. To
exploit the intrinsic parallelism of the stereo reconstruction task, the WASSjs
controller manages multiple parallel instances of each component. 

As a consequence, WASS can scale well in distributed memory machines (like
virtualized environments) with the only requirement to have a consistent shared
view of a common filesystem.


# Linux


# Mac OSX

*** NOTE: the following procedure was tested on Mac OS X 10.11.5 (El Capitan)
but should be valid on any relatively modern version of Mac OSX.***

The easiest way to install all the required libraries is via [Homebrew](http://brew.sh).
If brew is not already installed, follow the instructions on:

[http://brew.sh](http://brew.sh) to install the latest version.


Once brew is installed, open a terminal and enter the following commands

```
$ brew update
$ brew install cmake
$ brew install boost
$ brew install homebrew/science/opencv3
```

to install [CMake](https://cmake.org), [Boost](http://www.boost.org) and 
[OpenCV](http://opencv.org) in ```/usr/local/Cellar/```.

The next step is to install [node.js](https://nodejs.org). Download the 
"Mac OS X Installer (.pkg)" from the [nodejs download page](http://nodejs.org/en/download)
and run the installation package. You can verify that node.js is properly installed with the
commands:

```
$ node --version
$ npm --version
```

that should respond with the currently installed version of both ```node``` and ```npm```.

At this point, download the latest wass source code from the offical repository
(if you haven't done it yet) in a local directory of your choice. For example,
to download wass on your home directory enter the commands:

```
$ cd ~
$ git clone https://##
$ cd wass
$ git submodule update --init
```

To build the wass pipeline programs, on the same terminal, enter the commands:

```
$ mkdir build
$ cd build
$ cmake ../src/ -DOpenCV_DIR="/usr/local/opt/opencv3/share/OpenCV"
$ make
$ make install
```

you should verify that everything was build properly by typing:

```
$ cd ../dist/bin
$ ./wass_prepare
```

that should output something like:

```
wass_prepare  v. 1.0_heads/master-0-g5a7e63d
---------------------------------------
 Darwin-15.5.0 - AppleClang

wass_prepare arguments:
  --workdir arg         Workdir name
  --calibdir arg        Calibration data directory
  --c0 arg              Cam0 image file
  --c1 arg              Cam1 image file
```

To build WASSjs, enter the commands (assuming the current directory being ```<WASS_ROOT>/dist/bin/``` ): 

```
$ cd ../../WASSjs
$ npm install
$ cd ext
$ tar xvf redis-2.8.19.tar.gz
$ cd redis-2.8.19
$ make
```

wass and WASSjs should be now installed and configured. You can verify if
everything is working properly by [testing the pipeline](testing.html).


# Windows

WASS can be compiled on Windows with the latest Microsoft Visual Studio 2015
but requires a prior building of OpenCV, Boost C++ Libraries and
[CLAPACK](http://www.netlib.org/clapack/).

For this reason, ***it's higly recommended to install the Binary Version***
from the [download section](/download.html) and unzip the downloaded package on
a directory of your choice. We will refer to that directory as
```<BOOST_ROOT>``` throughout this documentation.

After that, download the latest ***nodejs*** for Windows from
[http://nodejs.org/en/download](http://nodejs.org/en/download), choose the
64bit Windows installer for the "current" (non LTS) version, and proceed with
its installation.

You can verify that node is installed properly by opening a command prompt 
and typing:

```
node --version
```

WASS should be installed and configured. You can proceed by [testing the pipeline](testing.html).

