# WASS

WASS (Waves Acquisition Stereo System) is an optimized stereo processing pipeline for sea waves 3D reconstruction.


It was developed by [Filippo Bergamasco](http://www.dsi.unive.it/~bergamasco/) as a joint-collaboration between [Università Ca'Foscari di Venezia](http://www.unive.it) and [CNR ISMAR](http://www.ismar.cnr.it). It is a result of more than 3 years of active research with [Alvise Benetazzo](http://www.ismar.cnr.it/people/benetazzo-alvise) (CNR-ISMAR) in the field of accurate 3D surface reconstruction of sea waves.


## Official project page

[http://www.dais.unive.it/wass](http://www.dais.unive.it/wass)


## Try it via Docker

1. Install [Docker](https://www.docker.com/products/docker-desktop) and [Docker compose](https://docs.docker.com/compose/).
2. Download the file [docker-compose.xml](https://raw.githubusercontent.com/fbergama/wass/Docker/docker-compose.yml)
3. Download the [test data](http://www.dais.unive.it/wass/WASS_TEST_docker.zip) and unzip the package in the same directory of `docker-compose.xml`
4. Create a directory named `out`. Your WASS working directory should contain the following: `docker-compose-yml`, `out/`, `WASS_TEST/`

5. In **Linux/OSX**:
    - open a terminal
    - cd into the directory containing `docker-compose-yml`
    - run the command: 
```
export UID=$(id -u) && export GID=$(id -g) && docker-compose up
```

5. In **Windows**: 
    - open the PowerShell prompt
    - cd into the directory containing `docker-compose-yml`
    - run the command:
```
docker-compose up
```

6. Open your browser to [http://localhost:8080](http://localhost:8080) and click in sequence: `prepare`,  `match`, `auto-calibrate` and `dense stereo`. The processed data will be placed in the `out` directory. If you want to process the whole dataset without using the web interface you can run the following:

```
docker run -i -t --network wass-net bergamasco/wass:runall
```

To spawn a shell inside the container:

```
docker exec -i -t  -u $(id -u):$(id -g) wass /bin/bash
```



### Run it with your data

Edit the lines 13, 19 and 25 of `docker-compose-yml` (the ones with `source: "<dir>"`) to set the location of configuration, input and output directory respectively. Refer to the pipeline documentation:
- [http://www.dais.unive.it/wass/documentation/stereo.html](http://www.dais.unive.it/wass/documentation/stereo.html)
- [http://www.dais.unive.it/wass/documentation/matcher.html](http://www.dais.unive.it/wass/documentation/matcher.html)

to properly setup the configuration files.

## License

```
Copyright (C) 2016-2020 Filippo Bergamasco 

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



