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
    ninja-build \
    "libc++1" \
    "libc++abi1" ||
    exit $?

function install_open3d_tar {
    TAR_BASE="open3d-devel-linux-x86_64-cxx11-abi-0.19.0"
    TAR_FILE="$TAR_BASE.tar.xz"
    OPEN3D_URL="https://github.com/isl-org/Open3D/releases/download/v0.19.0/$TAR_FILE"
    TMP_DIR=$(mktemp --tmpdir --directory open3d-tar-install-XXXX) || exit
    pushd $TMP_DIR || exit
    wget -nv "$OPEN3D_URL" || exit
    tar -xf "$TAR_FILE" || exit
    cp -r $TAR_BASE/* /usr/local/ || exit
    popd || exit
    rm -r $TMP_DIR || exit
}

install_open3d_tar || exit

source /etc/os-release || exit

function install_www_deb {
    TMP_DIR=$(mktemp --tmpdir --directory zivid-sdk-install-www-deb-XXXX) || exit
    pushd $TMP_DIR || exit
    wget -nv "$@" || exit
    apt-yes install --fix-broken ./*deb || exit
    popd || exit
    rm -r $TMP_DIR || exit
}

install_www_deb "https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u${VERSION_ID:0:2}/zivid_2.17.2+440b2367-1_amd64.deb" || exit
install_www_deb "https://downloads.zivid.com/sdk/releases/2.17.2+440b2367-1/u${VERSION_ID:0:2}/zivid-genicam_2.17.2+440b2367-1_amd64.deb" || exit
