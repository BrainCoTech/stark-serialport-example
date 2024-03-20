#!/bin/bash
set -e
cd ${0%/*}
SCRIPT_DIR=$(pwd)

if [[ "$OSTYPE" == "msys" ]]; then
    $COMSPEC /c download-lib.bat
    exit
fi

# libstark settings
LIB_VERSION="v0.0.3"
LIB_NAME=""
URL="https://app.brainco.cn/universal/stark-serialport-prebuild/${LIB_VERSION}"

# colorful echo functions
function echo_y() { echo -e "\033[1;33m$@\033[0m" ; }   # yellow
function echo_r() { echo -e "\033[0;31m$@\033[0m" ; }   # red

# check windows
if [[ "$OSTYPE" == "msys" ]]; then
    $COMSPEC /c download-lib.bat
    exit
fi

# 1. check libstark version from VERSION file
if [ -f VERSION ] && grep --fixed-strings --quiet ${LIB_VERSION} VERSION; then
    echo_y "[libstark] libstark (${LIB_VERSION}) is already installed"
    cat VERSION
    exit
fi

# clean files
rm -rf dist

# download libstark library
platform=$(uname)
if [ "$platform" == "Darwin" ]; then
    echo_y "[libstark] download libstark (${LIB_VERSION}) ..."
    LIB_NAME="mac"
elif [ "$(uname)" == "Linux" ]; then
    echo_y "[libstark] download libstark (${LIB_VERSION}) ..."  
    LIB_NAME="linux"
else
    echo_r "This script does not support your platform ($platform)"
    exit 1
fi

ZIP_NAME="$LIB_NAME.zip"
wget ${URL}/$ZIP_NAME
unzip -o $ZIP_NAME -d .
rm $ZIP_NAME
rm -rf __MACOSX

# 4. create VERSION file
echo "libstark Version: ${LIB_VERSION}" >  VERSION
echo "Update Time: $(date)"             >> VERSION

echo_y "[libstark] libstark (${LIB_VERSION}) is downloaded"
