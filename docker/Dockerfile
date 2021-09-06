# Copyright (c) 2020, Danish Technological Institute.
# All rights reserved.
#
# This source code is licensed under the GNU-style license found in the
# LICENSE file in the root directory of this source tree.
# 
# Original author: Nicolai Anton Lynnerup <nily@dti.dk>

FROM ubuntu:20.04

LABEL maintainer="Nicolai Anton Lynnerup <nily@dti.dk> & Lars Berscheid <lars.berscheid@kit.edu>"

# Set working directory

RUN mkdir -p /code
WORKDIR /code

# ------------ #
# Install deps #
# ------------ #

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    wget \
    iputils-ping \
    python3-dev \
    python3-pip \
    python3-setuptools \
    # Poco for libfranka \
    libpoco-dev \
    # Clear cache
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

# -------------- #
# Install Eigen3 #
# -------------- #

RUN git clone https://github.com/eigenteam/eigen-git-mirror.git \
    && cd eigen-git-mirror \
    && git checkout 3.3.7 \
    && mkdir build && cd build \
    && cmake .. \
    && make install

# -------------- #
# Install Franka #
# -------------- #

ARG libfranka_version=0.7.0

RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout $libfranka_version \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=ON .. \
    && make -j$(nproc) \
    && make install

# ---------------- #
# Install PyBind11 #
# ---------------- #

RUN git clone https://github.com/pybind/pybind11.git \
    && cd pybind11 \
    && git checkout v2.6 \
    && mkdir build && cd build \
    && cmake -DPYBIND11_TEST=OFF .. \
    && make -j$(nproc) \
    && make install

# ---------------- #
# Install Catch2   #
# ---------------- #

RUN git clone https://github.com/catchorg/Catch2.git \
    && cd Catch2 \
    && git checkout v2.5.0 \
    && mkdir build && cd build \
    && cmake -DCATCH_BUILD_TESTING=OFF -DCATCH_ENABLE_WERROR=OFF -DCATCH_INSTALL_DOCS=OFF -DCATCH_INSTALL_HELPERS=OFF .. \
    && make install

# ---------------- #
# Build frankx     #
# ---------------- #

RUN git clone --recursive https://github.com/pantor/frankx.git \
  && mkdir -p frankx/build && cd frankx/build \
  && cmake .. \
  && make -j$(nproc) \
  && make install \
  && ./unit-test

ENV PYTHONPATH=$PYTHONPATH:/code/frankx/build/ 
