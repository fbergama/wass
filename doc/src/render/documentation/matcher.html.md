---
title: "Matcher configuration"
layout: "default"
isPage: true
menuOrder: 6
---

# wass_match configuration

To adapt the feature matching procedure to the characteristics of the processed
stereo data, the ```wass_match``` executable can be configured with a dedicated
file whose name and location is defined in
```<WASS_ROOT>/WASSjs/settings.json``` (See the [pipeline configuration
page](/wass/documentation/configure.html) for more info).

By convention, the ```wass_match``` configuration file is named ```matcher_config.txt```
and is placed in the "configuration folder" containing the calibration data of the 
currently processed stereo dataset. To generate a new configuration file with the 
default values, run the ```wass_match``` executable with the option ```--genconfig```:

```
cd <WASS_ROOT>/dist/bin
./wass_match --genconfig

wass_match  v. 1.0_heads/master-0-g5a7e63d
---------------------------------------
 Darwin-15.5.0 - AppleClang

(wass_match) info: Generating matcher_config.txt ...
(wass_match) info: Done!
```

The generated file should be similar to the following:

```
# OpenSURF Hessian threshold
# 
#FEATURE_HESSIAN_THRESHOLD=0.0001

# OpenSURF init samples
# 
#FEATURE_INIT_SAMPLES=1

# Minimum distance allowed between two features (in px)
# 
#FEATURE_MIN_DISTANCE=10

# OpenSURF number of layers
# 
#FEATURE_N_LAYERS=4

# OpenSURF number of octaves
# 
#FEATURE_N_OCTAVES=4

# Matcher payoff lambda
# 
#MATCHER_LAMBDA=1e-05

# Max matches epipolar distance
# 
#MATCHER_MAX_EPI_DISTANCE=0.5

# Matcher maximum number of rounds to perform
# 
#MATCHER_MAX_ROUNDS=20

# Matcher minimum required group size
# 
#MATCHER_MIN_GROUP_SIZE=5

# Matcher population threshold
# 
#MATCHER_POPULATION_THRESHOLD=0.7

# Maxmum number of image features to extract
# 
#NUM_FEATURES_PER_IMAGE=2000
```

Each configuration option is composed by a ```key=value``` pair that can be
freely edited by the user. Lines beginning with a ```#``` are considered as
comments and hence are discarded by ```wass_match``` (ie. the default value is
assumed).

Here is a brief explanation of each configuration option:

|     key    |   Value    |
|------------|------------------|
| ```FEATURE_HESSIAN_THRESHOLD``` |  Hessian threshold used by OpenSURF feature extractor |
| ```FEATURE_INIT_SAMPLES``` |  "init samples" parameter used by OpenSURF feature extractor |
| ```FEATURE_MIN_DISTANCE``` | Minimum required distance (in pixel) between two features | 
| ```FEATURE_N_LAYERS``` | OpenSURF number of layers |
| ```FEATURE_N_OCTAVES``` | OpenSURF number of octaves |
| ```MATCHER_LAMBDA``` | Game-theoretic matcher payoff lambda parameter |
| ```MATCHER_MAX_EPI_DISTANCE``` | Game-theoretic matcher maximum epipolar distance allowed between two matched features |
| ```MATCHER_MAX_ROUNDS``` | Maximum number of games to be played |
| ```MATCHER_MIN_GROUP_SIZE``` | Minimum cardinality of a winning strategy to be considered valid | 
| ```MATCHER_POPULATION_THRESHOLD``` | Winning strategies payoff threshold |
| ```NUM_FEATURES_PER_IMAGE``` | Maximum number of features to extract for each image |

