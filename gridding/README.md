# wassgridsurface
---

wassgridsurface is a tool to interpolate into a uniform grid the scattered
point clouds produced by [WASS](https://sites.google.com/unive.it/wass).


## Usage


Assuming that "./output" is the WASS output dir containing the frames you want to grid:


### 1) Create the gridding output directory

```
mkdir gridding
```


### 2) Generate grid config file:

```
wassgridsurface --action generategridconfig . gridding
```


### 3) Edit the grid config file 

Open ```./gridding/gridconfig.txt``` with your favourite editor and set the required values


### 4) Setup the reconstruction

```
wassgridsurface --action setup ./output ./gridding --gridconfig ./gridding/gridconfig.txt --baseline [CAMERA_BASELINE]
```


### 5) Check

Open the image ```./gridding/area_grid.png``` to check the extension of the reconstructed area. Return to step 3) to make changes.


### 6) Run the gridding process

```
wassgridsurface --action grid --gridsetup ./gridding/config.mat ./output ./gridding
```

The resulting NetCDF file can be found in ./gridding/gridded.nc


To get a list of all command-line options:

```
wassgridsurface -h
```
