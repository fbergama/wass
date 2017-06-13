---
title: "Dense stereo configuration"
layout: "default"
isPage: true
menuOrder: 5
---

# Operation and configuration of wass_stereo

A proper configuration of the ```wass_stereo``` executable is a critical aspect
to obtain a dense and accurate reconstruction from a stereo sequence. 

The default configuration parameters have been tuned considering a stereo rig composed
by 5 megapixel cameras placed side-by-side with parallel optical axis, placed at about 10m
above the sea level and facing downward at about 20° with respect to the horizon. A different
configuration is possible but will probably require an adjustment of the configuration parameters.

In this page we describes step-by-step how to tune such parameters to get
the best possible results.

## Step 1 - Create a new configuration file

Start by creating a new configuration file by running the ```wass_stereo``` executable with
the ```--genconfig``` option:

```
cd <WASS_ROOT>/dist/bin/
./wass_stereo --genconfig

wass_stereo  v. 1.0_heads/master-0-g5a7e63d
---------------------------------------
 Darwin-15.5.0 - AppleClang

(wass_stereo) info: Generating stereo_config.txt ...
(wass_stereo) info: Done!
```

A new file called ```stereo_config.txt``` will be generated in the same directory of the ```wass_stereo```
executable containing some ```key=value``` pairs that can be freely edited. Lines starting with a ```#```
are not considered by ```wass_stereo``` (the default value is used). Copy the newly generated ```stereo_config.txt```
into the configuration directory of your dataset, together with the calibration files.


## Step 2 - Edit the basic stereo settings and check the result

The most important settings to be edited for a new dataset are the following:

|  key     |   Value    |
|----------|------------|
| ```MIN_DISPARITY``` <br /> ```MAX_DISPARITY```  |  Minimum and maximum disparity (in pixels) allowed while searching for stereo correspondences. Minimum disparity should be greater than 0 and maximum should be less than the image width. A greater range of disparities lower the reconstruction speed |
| ```WINSIZE``` |  Cross-correlation window size (in px) to compute matching cost |
| ```DENSE_SCALE``` | Scale to be applied to input images before dense stereo matching. Use 1 for full resolution stereo matching (recommended) |
| ```PLANE_MAX_DISTANCE``` |  Filter all the points exceeding PLANE_MAX_DISTANCE from the average fitted plane. Set this value greater than the expected wave elevation |
| ```DISPARITY_OFFSET``` |  Shift the right image to the left (if the number is positive) or to the right (if the number is negative) with respect to the left image before the disparity map estimation |

The ```DISPARITY_OFFSET``` option is particularly important to obtain a dense reconstruction of the scene as it allows to adjust the minimum and maximum reconstruction depth. To tune this parameter, start by setting it to ```0```. Then, launch a new reconstruction and open the file ```stereo_input.jpg``` that will be generated in any of the reconstructed workspaces (for example ```00000000_wd/```):

<img src="/wass/img/stereo_input0.jpg" style="width: 80%" />

The image shows the right image on top of the left image. Ideally, one would tune ```DISPARITY_OFFSET``` so that the farthest point to be reconstructed has zero disparity on the two images (ie. it projects on the same column) whereas the nearest point has a column offset less than ```MAX_DISPARITY```.

<img src="/wass/img/min_max_disparity.jpg" style="width: 80%" />

If ```DISPARITY_OFFSET``` parameter is set to a positive value ```n```, the right stereo image is shifted to the left with respect to the left image by ```n``` pixels. Similarly, if the parameter is negative the right stereo image is moved to the right. Here is an example of the shift obtained by setting ```DISPARITY_OFFSET=-100``` and ```DISPARITY_OFFSET=100``` respectively:

<div>
<img src="/wass/img/stereo_input-100.jpg" style="width: 45%" />
<img src="/wass/img/stereo_input100.jpg" style="width: 46.5%" />
</div>

Once ```DISPARITY_OFFSET``` has been set to a satisfying value, ```MAX_DISPARITY``` can be tuned as well (with the constrains of being multiple of 16) to improve the reconstruction time.


## Step 3 - Edit the optional settings to improve the reconstruction

Here is a list of additional parameters to consider that may improve the reconstruction depending on the dataset


|  key     |   Value    |
|----------|------------|
| ```DISP_DILATE_STEPS```  | Number of dilation steps to be performed after disparity map computation. A high value may help close the holes in the reconstructed disparity map |
| ```DISP_EROSION_STEPS```  | Number of erosion steps to be performed after disparity map dilation. A high value may help removing the erroneous reconstructed points at the edges of the observed area |
| ```PLANE_WEIGHT_PROPORTIONAL_TO_DISTANCE```  | Compute the mean sea-plane by averaging each point weighted by its distance from the cameras |
| ```MEDIAN_FILTER_WSIZE```  | Window size of median filter applied to the disparity map. ```0``` to disable |
| ```ZGAP_PERCENTILE```  | Percentile used while z-gap connected components filtering |
| ```SAVE_COMPRESSED```  | Save the generated point cloud in the compressed format (about half the size of non-compressed format with a minimal loss of precision) |

## Automatic Left-Right detection

Unless the ```DISABLE_AUTO_LEFT_RIGHT``` parameter is set to ```1```, ```wass_stereo``` will automatically try to detect the spatial configuration of the two cameras. If something goes wrong, it is possibly to swap the default left-right positioning by setting ```DISABLE_AUTO_LEFT_RIGHT=1``` and ```SWAP_LEFT_RIGHT``` to ```1``` or ```0``` according to the desired configuration.

***Note that vertical stereo is not supported yet.***



