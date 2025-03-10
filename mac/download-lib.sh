#!/bin/bash
set -e

cd ${0%/*}
SCRIPT_DIR=$(pwd)

# libstark settings
LIB_VERSION="v0.1.6"
LIB_NAME=""
BASE_URL="https://app.brainco.cn/universal/bc-device-sdk/libs/${LIB_VERSION}"

# colorful echo functions
function echo_y() { echo -e "\033[1;33m$@\033[0m"; } # yellow
function echo_r() { echo -e "\033[0;31m$@\033[0m"; } # red

# 1. check libstark version from VERSION file
if [ -f VERSION ] && grep --fixed-strings --quiet ${LIB_VERSION} VERSION; then
    echo_y "[libstark] libstark (${LIB_VERSION}) is already installed"
    cat VERSION
    exit
fi

# clean files
rm -rf dist
mkdir -p dist

# download libstark library
platform=$(uname)
if [ "$platform" == "Darwin" ]; then
    echo_y "[libstark] download libstark (${LIB_VERSION}) ..."
    LIB_NAME="mac"
elif [ "$(uname)" == "Linux" ]; then
    echo_y "[libstark] download libstark (${LIB_VERSION}) ..."
    LIB_NAME="linux"
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "20.04" ]]; then
            LIB_NAME="ubuntu-20"
        fi
    fi
else
    echo_r "This script does not support your platform ($platform)"
    exit 1
fi

ZIP_NAME="$LIB_NAME.zip"
# Download library
echo_y "[treadmill-sdk] Downloading (${LIB_VERSION})..."
if ! command -v wget >/dev/null 2>&1; then
  echo_r "Error: wget is not installed. Please install it and try again."
  exit 1
fi

DOWNLOAD_URL="${BASE_URL}/${ZIP_NAME}?$(date +%s)" # Use timestamp for uniqueness
wget -q --show-progress "$DOWNLOAD_URL" -O "${SCRIPT_DIR}/${ZIP_NAME}" || {
  echo_r "Error: Failed to download ${ZIP_NAME}"
  exit 1
}

# Extract and clean up
echo_y "[stark-sdk] Extracting ${ZIP_NAME}..."
unzip -o $ZIP_NAME -d .
rm $ZIP_NAME
rm -rf __MACOSX
find dist/include -type f ! -name stark-sdk.h -exec rm -f {} \;

# 4. create VERSION file
echo "libstark Version: ${LIB_VERSION}" >VERSION
echo "Update Time: $(date)" >>VERSION

echo_y "[libstark] libstark (${LIB_VERSION}) is downloaded"
