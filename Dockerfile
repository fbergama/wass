FROM ubuntu:16.04
LABEL maintainer "filippo.bergamasco@unive.it"

RUN apt-get -yqq update && apt-get -yqq install curl build-essential cmake git
RUN apt-get -yqq update && apt-get -yqq install liblapack-dev libblas-dev libboost-all-dev
RUN apt-get -yqq update && apt-get -yqq install ffmpeg libavcodec-dev libavformat-dev libswscale-dev libavresample-dev

# Install gosu
RUN gpg --keyserver ha.pool.sks-keyservers.net --recv-keys B42F6819007F00F88E364FD4036A9C25BF357DD4
RUN curl -o /usr/local/bin/gosu -SL "https://github.com/tianon/gosu/releases/download/1.4/gosu-$(dpkg --print-architecture)" \
    && curl -o /usr/local/bin/gosu.asc -SL "https://github.com/tianon/gosu/releases/download/1.4/gosu-$(dpkg --print-architecture).asc" \
    && gpg --verify /usr/local/bin/gosu.asc \
    && rm /usr/local/bin/gosu.asc \
    && chmod +x /usr/local/bin/gosu


# Install node
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get -yqq update && apt-get install -y nodejs

# Build OpenCV version 3.4
#
WORKDIR /LIBS/
RUN git clone -b 3.4.0 --single-branch https://github.com/opencv/opencv.git --depth 1
RUN git clone -b 3.4.0 --single-branch https://github.com/opencv/opencv_contrib.git --depth 1
WORKDIR /LIBS/opencv
RUN mkdir build && mkdir dist && cd build && \
    cmake ../ -DCMAKE_INSTALL_PREFIX="../dist/" -DCMAKE_BUILD_TYPE="Release" -DOPENCV_EXTRA_MODULES_PATH=/LIBS/opencv_contrib/modules -DBUILD_opencv_xfeatures2d=OFF   -DBUILD_opencv_xobjdetect=OFF -DBUILD_opencv_ximgproc=ON   -DBUILD_opencv_xphoto=OFF -DBUILD_opencv_superres=OFF  -DBUILD_opencv_surface_matching=OFF  -DBUILD_opencv_structured_light=OFF  -DBUILD_opencv_stitching=OFF  -DBUILD_opencv_saliency=OFF  -DBUILD_opencv_phase_unwrapping=OFF  -DBUILD_opencv_bioinspired=OFF  -DBUILD_opencv_aruco=OFF  -DBUILD_opencv_dnn=OFF  -DBUILD_opencv_datasets=OFF  -DBUILD_opencv_python_bindings_generator=OFF  -DBUILD_opencv_fuzzy=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF && \
    make -j 8


# Add the WASS source tree
WORKDIR /wass
ADD . /wass

# Build the backend
RUN git submodule init && git submodule update
WORKDIR /wass/build
RUN rm -Rf *
RUN cmake ../src/ -DCMAKE_BUILD_TYPE="Release" -DOpenCV_DIR=/LIBS/opencv/build/ && make && make install


# Run npm to install all WASSjs dependencies
WORKDIR /wass/WASSjs
RUN npm install
# Install REDIS
WORKDIR /wass/WASSjs/ext
RUN tar xvfz redis-2.8.19.tar.gz && cd redis-2.8.19 && make

ADD ./Docker/worksession.json /wass/WASSjs/worksession.json
ADD ./Docker/settings.json /wass/WASSjs/settings.json

WORKDIR /DATA_IN
VOLUME ["/DATA_IN"]

WORKDIR /DATA_OUT
VOLUME ["/DATA_OUT"]

WORKDIR /DATA_CONF
VOLUME ["/DATA_CONF"]

WORKDIR /wass/WASSjs
EXPOSE 8080
CMD ./run_as_daemon.sh

