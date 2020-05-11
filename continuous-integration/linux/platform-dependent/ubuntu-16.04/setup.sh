#!/bin/bash

export DEBIAN_FRONTEND=noninteractive

function apt-yes {
    apt-get --assume-yes "$@"
}

apt-yes update || exit $?
apt-yes dist-upgrade || exit $?

apt-yes install software-properties-common || exit $?
add-apt-repository -y ppa:ubuntu-toolchain-r/test || exit $?
apt-yes update || exit $?

apt-yes install \
    g++ \
    clang \
    cmake \
    wget \
    ninja-build \
    libx11-dev \
    libgl1-mesa-glx \
    || exit $?

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory install_www_deb-XXXX) || exit $?
    pushd $TMP_DIR || exit $?
    wget -q "$@" || exit $?
    echo "Installing Zivid debian package $1"
    apt-yes install --fix-broken ./*deb || exit $?
    popd || exit $?
    rm -r $TMP_DIR || exit $?
}

install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.8.1+6967bc1b-1/u16/zivid-telicam-driver_3.0.1.1-1_amd64.deb || exit $?
install_www_deb https://www.zivid.com/hubfs/softwarefiles/releases/1.8.1+6967bc1b-1/u16/zivid_1.8.1+6967bc1b-1_amd64.deb || exit $?
