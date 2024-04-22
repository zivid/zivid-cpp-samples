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

source /etc/os-release || exit

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory zivid-sdk-install-www-deb-XXXX) || exit
    pushd $TMP_DIR || exit
    wget -nv "$@" || exit
    apt-yes install --fix-broken ./*deb || exit
    popd || exit
    rm -r $TMP_DIR || exit
}

install_www_deb "https://downloads.zivid.com/sdk/releases/2.12.0+6afd4961-1/u${VERSION_ID:0:2}/zivid_2.12.0+6afd4961-1_amd64.deb" || exit
