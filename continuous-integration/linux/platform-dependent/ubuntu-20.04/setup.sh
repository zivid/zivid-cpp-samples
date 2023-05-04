#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit
apt-yes dist-upgrade || exit

apt-yes install \
    clang \
    clang-format-10 \
    clang-tidy-10 \
    cmake \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    wget \
    ninja-build ||
    exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit
    pushd $TMP_DIR || exit
    wget -q "$@" || exit
    echo "Installing Zivid debian package $1"
    apt-yes install --fix-broken ./*deb || exit
    popd || exit
    rm -r $TMP_DIR || exit
}

install_www_deb https://downloads.zivid.com/sdk/releases/2.9.0+9ebc01a6-1/u20/zivid-telicam-driver_3.0.1.1-3_amd64.deb || exit
install_www_deb https://downloads.zivid.com/sdk/releases/2.9.0+9ebc01a6-1/u20/zivid_2.9.0+9ebc01a6-1_amd64.deb || exit
