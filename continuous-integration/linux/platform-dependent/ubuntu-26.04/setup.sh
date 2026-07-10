#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit
apt-yes dist-upgrade || exit

apt-yes install \
    clang \
    clang-format \
    clang-tidy \
    cmake \
    g++ \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    wget \
    ninja-build ||
    exit $?

# clang++ links against the newest installed GCC toolchain, which dependencies
# may pull in ahead of the default g++; install its matching libstdc++ dev.
newest_gcc_major=$(ls -1 /usr/lib/gcc/x86_64-linux-gnu/ | sort --numeric-sort | tail --lines=1)
apt-yes install "libstdc++-${newest_gcc_major}-dev" || exit $?

source /etc/os-release || exit

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory zivid-sdk-install-www-deb-XXXX) || exit
    pushd $TMP_DIR || exit
    wget -nv "$@" || exit
    apt-yes install --fix-broken ./*deb || exit
    popd || exit
    rm -r $TMP_DIR || exit
}

install_www_deb "https://downloads.zivid.com/sdk/releases/2.18.0+1b44dbef-1/u${VERSION_ID:0:2}/zivid-opencl_2.18.0+1b44dbef-1_amd64.deb" || exit
install_www_deb "https://downloads.zivid.com/sdk/releases/2.18.0+1b44dbef-1/u${VERSION_ID:0:2}/zivid-genicam_2.18.0+1b44dbef-1_amd64.deb" || exit
