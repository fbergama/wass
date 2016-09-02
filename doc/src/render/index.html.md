---
title: "Overview"
layout: "default"
isPage: true
---

<div class="starter-template" >
<h1 class="maintitle">Welcome to the WASS project</h1>
<p class="subtitle" >WASS (Waves Acquisition Stereo System) is an optimized stereo processing pipeline for sea waves 3D reconstruction.</p>
<video width="800" height="422" autoplay loop >
  <source src="/wass/videos/3D_mesh_2.mp4" >
  Your browser does not support the video tag.
</video>

</div>

<p class="subtitle2">
WASS was developed by [Filippo Bergamasco](http://www.dsi.unive.it/~bergamasco/) as a joint-collaboration between [Università Ca'Foscari di Venezia](http://www.unive.it) and [CNR ISMAR](http://www.ismar.cnr.it). It is a result of more than 3 years of active research with [Alvise Benetazzo](http://www.ismar.cnr.it/people/benetazzo-alvise) (CNR-ISMAR) in the field of vision-based 3D surface reconstruction of sea waves.
</p>

<div class="logodiv" >
<img src="/wass/img/unive-logo.jpg" class="logoimg" />&nbsp;
<img src="/wass/img/cnr-logo.jpg" class="logoimg" />&nbsp;
<img src="/wass/img/ritmare-logo.png" class="logoimg" />
</div>

# Features

- High-quality 3D point-cloud recovery from stereo frames
- Fully-automatic robust extrinsic calibration of the stereo rig (up to scale)
- Optimized C++ code for fast processing (~30sec per frame for 3 MPixel images on consumer i7 CPU)
- Advanced point-cloud filtering for outlier removal
- Easily controllable with your favourite browser
- Event-driven parallel task management via [node.js](http://www.nodejs.org)
- Open source code based on mature open-source components
- Multi-platform support (Linux, OSX, Windows)


# Contributing

You are more than welcome to contribute either by improving the code or by testing it with your stereo data. If you are using WASS for your research, please [contact the authors](mailto:alvise.benetazzo@ve.ismar.cnr.it) for informations on how to cite this work.

To report bugs or propose new features please use the [GitHub Issue Tracker](https://github.com/fbergama/wass/issues).
 

# License

```
Copyright (C) 2016 Filippo Bergamasco 

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

