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
- [Docker](#docker)


***Important Note:***

Since from version 4.x the OpenCV library dropped the support of the old C API, part of the WASS
pipeline can no longer be compiled without "downgrading" to 3.x branch. To be sure of having a tested and working
WASS system, please consider to install it as a Docker container as described [here](#docker).

Advantages of using the containerized version:

- No need to manually compile and install all the required dependencies for your system (everything is done by the Dockerfile script)
- You'll have a tested and repeatable ecosystem across different platforms
- Easily setup in/out/config directories via Docker volumes (bash scripts provided)

If you already have Docker up an running, you can quickly skip to the [Docker installation instructions](#docker).



## Pre-requisites

WASS relies to the following 3rd party components:

- [OpenCV](http://opencv.org) Open-Source Computer Vision Library (Version 3.x required) 
- [Boost](http://www.boost.org) C++ Libraries
- [Lapack](http://www.netlib.org/lapack/) Linear Algebra PACKage
- [SBA](http://users.ics.forth.gr/~lourakis/sba/) Sparse Bundle Adjustment C/C++ Library
- [redis](http://redis.io) In-memory data structure store
- [node.js](https://nodejs.org) Javascript runtime

Additionally, [Matlab](http://www.mathworks.com) is required for some post-processing steps like surface estimation and waves spectrum recovery.


# Linux

*** NOTE: the following procedure was tested on Linux Ubuntu 16.04.01 LTS (Xenial Xerus)
but should be valid on any relatively modern Linux distribution providing the appropriate
third party packages.***

Start by installing [Boost](http://www.boost.org), [Lapack](http://www.netlib.org/lapack/) and 
all the needed build tools from the official packages provided by Ubuntu. Open a terminal, cd
into a directory of your choice end enter the following command:

```
sudo apt-get install git build-essential cmake curl liblapack-dev libblas-dev libboost-all-dev 
```

[OpenCV](http://opencv.org) has to be compiled by source since version 3.1 is
not provided yet via the official Ubuntu repositories. On the same terminal,
enter the following commands:

```
sudo apt-get install ffmpeg libavcodec-dev libavformat-dev
git clone https://github.com/opencv/opencv.git --depth 1 -b 3.4.0 --single-branch
cd opencv
mkdir build
cd build 
cmake ../ -DCMAKE_INSTALL_PREFIX="../dist/" -DCMAKE_BUILD_TYPE="Release"
make
make install
cd ../..
```

The commands will, in order: install some libraries needed by OpenCV, clone the OpenCV official
repository, create a "build" and "dist" directory, configure/build/install the software
into ```opencv/dist``` subdirectory.

At this point, download the latest WASS source code from the official repository and 
create a ```build/``` subdirectory: 

```
git clone https://github.com/fbergama/wass
cd wass
git submodule update --init
mkdir build
```

To build WASS, on the same terminal, enter the following:

```
cd build
cmake ../src/ -DOpenCV_DIR="../../opencv/build"
make
make install
cd ..
```

If no error occurred, your terminal should be located in ```WASS_ROOT``` and
all the wass pipeline executables should be located in ```dist/bin```
subfolder.

The last step is to install WASSjs. First, install ```node.js``` from packages
following the instructions on
[https://nodejs.org/en/download/package-manager/](https://nodejs.org/en/download/package-manager/)
(if using Ubuntu, proceed to the "Debian and Ubuntu based Linux
distributions").

Then, enter the following commands:

```
cd WASSjs
sudo npm install
cd ext
tar xvfz redis-2.8.19.tar.gz
cd redis-2.8.19
make
cd ../../..
```

WASS should be installed and configured. You can proceed with [testing the pipeline](testing.html).

# Mac OSX

We suggest to install [Docker Desktop](https://www.docker.com/products/docker-desktop) (free registration required)
and proceed to the [Docker installation instructions](#docker).

*** NOTE: the following procedure was tested on Mac OS X 10.11.5 (El Capitan)
but should be valid on any relatively modern version of Mac OSX matching the required libraries***

The easiest way to install all the required libraries is via
[Homebrew](http://brew.sh).  If brew is not already installed, follow the
instructions on [http://brew.sh](http://brew.sh) to install the latest version.


Once brew is installed, open a terminal and enter the following commands

```
brew update
brew install cmake
brew install boost
brew install webp
brew install opencv@3
```

to install [CMake](https://cmake.org), [Boost](http://www.boost.org) and 
[OpenCV](http://opencv.org) in ```/usr/local/Cellar/```.

The next step is to install [node.js](https://nodejs.org). Download the 
"Mac OS X Installer (.pkg)" from the [nodejs download page](http://nodejs.org/en/download)
and run the installation package. You can verify that node.js is properly installed with the
commands:

```
node --version
npm --version
```

that should respond with the currently installed version of both ```node``` and ```npm```.

At this point, download the latest wass source code from the offical repository
(if you haven't done it yet) in a local directory of your choice. For example,
to download wass on your home directory enter the commands:

```
cd ~
git clone https://github.com/fbergama/wass
cd wass
git submodule update --init
```

To build the wass pipeline programs, on the same terminal, enter the commands:

```
mkdir build
cd build
cmake ../src/ -DOpenCV_DIR="/usr/local/opt/opencv@3/<version>/share/OpenCV"
make
make install
```

you should verify that everything was build properly by typing:

```
cd ../dist/bin
./wass_prepare
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
cd ../../WASSjs
npm install
cd ext
tar xvf redis-2.8.19.tar.gz
cd redis-2.8.19
make
cd ../..
```

wass and WASSjs should be now installed and configured. You can verify if
everything is working properly by [testing the pipeline](testing.html).


# Windows

WASS can be built from sources with the latest Microsoft Visual Studio 2015
but requires a prior building of OpenCV, Boost C++ Libraries and
[CLAPACK](http://www.netlib.org/clapack/).

For this reason, ***it's higly recommended to install the Binary Version***
from the [download section](/wass/download.html) and unzip the downloaded package on
a directory of your choice. We will refer to that directory as
```<WASS_ROOT>``` throughout this documentation.

After that, download the latest ***nodejs*** for Windows from
[http://nodejs.org/en/download](http://nodejs.org/en/download), choose the
64bit Windows installer for the "current" (non LTS) version, and proceed with
its installation.

You can verify that node is installed properly by opening a command prompt 
and typing:

```
node --version
```

Finally, install the [Visual C++ 2015 Redistributable
x64](files/vc_redist.x64.exe).

WASS should be now installed and configured. You can proceed with [testing the
pipeline](testing.html).


# Docker

A complete WASS system can be installed in a Docker container. A set of bash scripts
simplify the process of creating and running the appropriate container.

Supposing that Docker is already installed and configured for your system, you can 
simply follow this steps:

## Download the latest version of WASS

```
cd ~
git clone https://github.com/fbergama/wass
cd wass
```

## Build the docker image

```
sudo ./Docker/wass_docker_build.sh
```

This usually requires root permissions but may vary depending on your Docker installation

## Run wass

Once the image is created, supposing that all the WASS configuration files are
located in a folder named ```<config>```, the input data in a folder named ```<in>``` and you want
the output data to be placed in ```<out>```, just run WASS with:

```
$ ./Docker/wass_docker_run.sh <config> <in> <out>
```

And open your browser to http://localhost:8080. Additionally, you can spawn a shell inside the wass
Docker container with the command:

```
$ ./Docker/wass_docker_shell.sh
```

and use the pipeline manually.

## settings.json and worksession.json config files

Files inside the container are not preserved if modified. If you need to make changes
to ```settings.json``` or ```worksession.json``` just copy those files in the ```<config>``` directory.
Upon bootstrapping, they will be copied to the correct location inside the container.



