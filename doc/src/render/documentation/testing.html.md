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
[http://www.dsi.unive.it/~bergamasco/tmp/WASS_TEST.zip](http://www.dsi.unive.it/~bergamasco/tmp/WASS_TEST.zip)
and unzip the downloaded package into the ```<WASS_ROOT>/test``` directory.

Your ```<WASS_ROOT>/test``` directory should look similar to this:

```
```


## Running the tests

Open Matlab and change the working directory to ```<WASS_ROOT>/test```. Then, run the matlab script ```pipeline_test.m```. After a while, depending
on your hardware, you should get the following output:

```

```

indicating that all the tests were successful.

