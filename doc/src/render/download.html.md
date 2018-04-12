---
title: "Download"
layout: "default"
isPage: true
menuOrder: 1
---


<img src="/wass/img/linux.png" class="platformlogo" />
<img src="/wass/img/osx.png" class="platformlogo" />

The suggested way to obtain the latest version of WASS for ***Linux*** or
***Mac OSX*** is by cloning the project GitHub repository and compile it from
sources.

Open a terminal in a directory of your choice and enter the following commands:

```
git clone https://github.com/fbergama/wass
cd wass
git submodule update --init
```

Read the [Installation instructions](documentation/install.html) for a step-by-step tutorial
on how to build the source code depending on your system.


### Windows binaries

<img src="/wass/img/win.png" class="platformlogo" />

For ***Microsoft Windows (64bit)***, you can also download a pre-built binary distribution (Tested on Windows 7 and Windows 10, Compiled with MSVC 2015):

Latest version (recommended):

- [wass_1.4_win32_x64.zip](/wass/files/wass_1.4_win32_x64.zip)


Previous versions:

- [wass_1.1_win32_x64.zip](/wass/files/wass_1.1_win32_x64.zip)


**NOTE:** Pre-built WASS distribution requires [Visual C++ 2015 Redistributable x64](files/vc_redist.x64.exe)



