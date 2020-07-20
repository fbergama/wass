FROM ubuntu:18.04 AS builder
LABEL maintainer "filippo.bergamasco@unive.it"

RUN groupadd -r -g 999 wass && useradd -r -g wass -u 999 wass

RUN apt-get -y -qq update && apt-get -y -qq install zip gzip tar curl build-essential cmake git liblapack-dev libblas-dev libboost-all-dev ffmpeg libavcodec-dev libavformat-dev libswscale-dev libavresample-dev



# grab gosu for easy step-down from root ---------------------------------
#
# https://github.com/tianon/gosu/releases
ENV GOSU_VERSION 1.11
RUN set -eux; \
# save list of currently installed packages for later so we can clean up
    savedAptMark="$(apt-mark showmanual)"; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        dirmngr \
        gnupg \
        wget \
    ; \
    rm -rf /var/lib/apt/lists/*; \
    \
    dpkgArch="$(dpkg --print-architecture | awk -F- '{ print $NF }')"; \
    wget -O /usr/local/bin/gosu "https://github.com/tianon/gosu/releases/download/$GOSU_VERSION/gosu-$dpkgArch"; \
    wget -O /usr/local/bin/gosu.asc "https://github.com/tianon/gosu/releases/download/$GOSU_VERSION/gosu-$dpkgArch.asc"; \
    \
# verify the signature
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver hkps://keys.openpgp.org --recv-keys B42F6819007F00F88E364FD4036A9C25BF357DD4; \
    gpg --batch --verify /usr/local/bin/gosu.asc /usr/local/bin/gosu; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME" /usr/local/bin/gosu.asc; \
    \
# clean up fetch dependencies
    apt-mark auto '.*' > /dev/null; \
    [ -z "$savedAptMark" ] || apt-mark manual $savedAptMark > /dev/null; \
    apt-get purge -y --auto-remove -o APT::AutoRemove::RecommendsImportant=false; \
    \
    chmod +x /usr/local/bin/gosu; \
# verify that the binary works
    gosu --version; \
    gosu nobody true


# End of gosu stuff ------------------------------------



# Install node
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get -yqq update && apt-get install -y nodejs npm


# Build OpenCV version 3.4
#
WORKDIR /LIBS
RUN chown wass:wass /LIBS
USER wass
RUN git clone -b 3.4.0 --single-branch https://github.com/opencv/opencv.git --depth 1
RUN git clone -b 3.4.0 --single-branch https://github.com/opencv/opencv_contrib.git --depth 1
WORKDIR /LIBS/opencv
RUN mkdir build && mkdir dist && cd build && \
    cmake ../ -DCMAKE_INSTALL_PREFIX="../dist/" -DCMAKE_BUILD_TYPE="Release" -DOPENCV_EXTRA_MODULES_PATH=/LIBS/opencv_contrib/modules -DBUILD_opencv_xfeatures2d=OFF   -DBUILD_opencv_xobjdetect=OFF -DBUILD_opencv_ximgproc=ON   -DBUILD_opencv_xphoto=OFF -DBUILD_opencv_superres=OFF  -DBUILD_opencv_surface_matching=OFF  -DBUILD_opencv_structured_light=OFF  -DBUILD_opencv_stitching=OFF  -DBUILD_opencv_saliency=OFF  -DBUILD_opencv_phase_unwrapping=OFF  -DBUILD_opencv_bioinspired=OFF  -DBUILD_opencv_aruco=OFF  -DBUILD_opencv_dnn=OFF  -DBUILD_opencv_datasets=OFF  -DBUILD_opencv_python_bindings_generator=OFF  -DBUILD_opencv_fuzzy=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF && \
    make -j 8 && \
    make install



# Add the WASS source tree
WORKDIR /wass
ADD . /wass
USER root
RUN chown -R wass:wass /wass/
USER wass


# Build the backend
RUN git submodule init && git submodule update
RUN mkdir /wass/build
WORKDIR /wass/build
RUN rm -Rf *
RUN cmake ../src/ -DCMAKE_BUILD_TYPE="Release" -DOpenCV_DIR=/LIBS/opencv/build -DDISABLE_BOOST_LOG=ON && make && make install


# Run npm to install all WASSjs dependencies
USER root
RUN mkdir /home/wass && chown wass:wass /home/wass
USER wass
WORKDIR /wass/WASSjs
RUN npm install


# Final setup
ADD ./Docker/worksession.json /wass/WASSjs/worksession.json
ADD ./Docker/settings.json /wass/WASSjs/settings.json
USER root
RUN chown wass:wass /wass/WASSjs/worksession.json && chmod a+rw /wass/WASSjs/worksession.json
RUN chown wass:wass /wass/WASSjs/settings.json && chmod a+rw /wass/WASSjs/settings.json
USER wass
RUN echo 'export PATH=/wass/dist/bin:$PATH' >> /home/wass/.bashrc


USER root
ADD ./WASSjs/docker_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]


WORKDIR /DATA_IN
VOLUME ["/DATA_IN"]
WORKDIR /DATA_OUT
VOLUME ["/DATA_OUT"]
WORKDIR /DATA_CONF
VOLUME ["/DATA_CONF"]

USER wass
WORKDIR /wass/WASSjs
EXPOSE 8080
CMD ["/wass/WASSjs/run_as_daemon.sh"]







# Create the final optimized build
# -----------------------------------------------

FROM ubuntu:18.04
LABEL maintainer "filippo.bergamasco@unive.it"

ARG BUILD_DATE
ARG VCS_REF

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.description="WASS sea-waves 3D reconstruction pipeline"
LABEL org.label-schema.url="http://dais.unive.it/wass"
LABEL org.label-schema.build-date=$BUILD_DATE
LABEL org.label-schema.vcs-ref=$VCS_REF

RUN groupadd -r -g 999 wass && useradd -r -g wass -u 999 wass

RUN apt-get -y -qq update && apt-get -y -qq install zip libboost-system1.65.1 libboost-log1.65.1 libboost-program-options1.65.1 liblapack3

COPY --from=builder /usr/local/bin /usr/local/bin
COPY --from=builder /wass/dist/bin /wass/dist/bin
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_calib3d.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_highgui.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_imgcodecs.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_imgproc.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_core.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_features2d.so.3.4 /LIBS/opencv/dist/lib/
COPY --from=builder /LIBS/opencv/dist/lib/libopencv_flann.so.3.4 /LIBS/opencv/dist/lib/

RUN ln -s /LIBS/opencv/dist/lib /wass/dist/lib

COPY --from=builder /wass/WASSjs /wass/WASSjs
RUN rm -R /wass/WASSjs/ext
RUN ls -alh /wass/WASSjs/


# Install node
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get -yqq update && apt-get install -y nodejs


WORKDIR /DATA_IN
VOLUME ["/DATA_IN"]
WORKDIR /DATA_OUT
VOLUME ["/DATA_OUT"]
WORKDIR /DATA_CONF
VOLUME ["/DATA_CONF"]

USER wass
WORKDIR /wass/WASSjs
EXPOSE 8080

ENTRYPOINT ["/usr/local/bin/docker_entrypoint.sh"]
CMD ["/wass/WASSjs/run_as_daemon.sh"]
