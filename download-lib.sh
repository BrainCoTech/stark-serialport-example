#!/bin/bash
set -e # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/dist"
VERSION_FILE="${SCRIPT_DIR}/VERSION"

# Configuration
LIB_VERSION="v1.0.4"
BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${LIB_VERSION}"

# Colorful echo functions
echo_y() { echo -e "\033[1;33m$*\033[0m"; } # Yellow
echo_r() { echo -e "\033[0;31m$*\033[0m"; } # Red

# Check if version is already installed
if [ -f "$VERSION_FILE" ] && grep -F --quiet "$LIB_VERSION" "$VERSION_FILE"; then
  echo_y "[bc-stark-sdk] (${LIB_VERSION}) is already installed"
  cat "$VERSION_FILE"
  exit 0
fi

# Determine platform and library name
OS_TYPE=$(uname -s)
ARCH=$(uname -m)
IS_ARM64=$([[ "$ARCH" == "aarch64" ]] && echo 1 || echo 0)
echo_y "OS type: $OS_TYPE, ARCH: $ARCH"
case "$OS_TYPE" in
"Linux")
  # 加载系统发行版信息
  if [ -f /etc/os-release ]; then
    . /etc/os-release
  fi

  # 根据系统和架构设置ZIP文件名
  if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "22.04" ]]; then
    LIB_PREFIX="linux"
    # LIB_PREFIX="ubuntu-22"
  else
    LIB_PREFIX="linux"
  fi

  ARM64_SUFFIX=""
  [[ $IS_ARM64 -eq 1 ]] && ARM64_SUFFIX="-arm64"
  LIB_NAME="${LIB_PREFIX}${ARM64_SUFFIX}"
  ;;
"Darwin")
  LIB_NAME="mac"
  ;;
"msys" | "MINGW"*)
  LIB_NAME="win"
  ;;
*)
  echo_r "Error: This script does not support your platform ($OS_TYPE)"
  exit 1
  ;;
esac

ZIP_NAME="${LIB_NAME}.zip"
DOWNLOAD_URL="${BASE_URL}/${ZIP_NAME}?$(date +%s)" # Timestamp for uniqueness

# Clean up previous files
echo_y "[bc-stark-sdk] Cleaning up previous distribution..."
rm -rf "$DIST_DIR" "${SCRIPT_DIR}/__MACOSX" "${SCRIPT_DIR}/${ZIP_NAME}"

# Create dist directory
mkdir -p "$DIST_DIR"

# Download library
echo_y "[bc-stark-sdk] Downloading (${LIB_VERSION}) for ${LIB_NAME}..."
if ! command -v wget >/dev/null 2>&1; then
  echo_r "Error: wget is not installed. Please install it and try again."
  exit 1
fi

wget -q --show-progress "$DOWNLOAD_URL" -O "${SCRIPT_DIR}/${ZIP_NAME}" || {
  echo_r "Error: Failed to download ${ZIP_NAME}"
  exit 1
}

# Extract and clean up
echo_y "[bc-stark-sdk] Extracting ${ZIP_NAME}..."
unzip -o -q "${SCRIPT_DIR}/${ZIP_NAME}" -d "$SCRIPT_DIR" || {
  echo_r "Error: Failed to unzip ${ZIP_NAME}"
  exit 1
}
rm -f "${SCRIPT_DIR}/${ZIP_NAME}"
rm -rf "${SCRIPT_DIR}/__MACOSX"
rm -rf "${DIST_DIR}/__MACOSX"
find dist/include \
  -type f \
  ! -name 'stark-sdk.h' \
  ! -path 'dist/include/zlgcan/*' \
  -exec rm -f {} \;

case "$OS_TYPE" in
"Linux")
  # 可以拷贝到系统目录
  # sudo cp -vf dist/shared/linux/*.so /usr/lib/
  # sudo ln -s /usr/lib/libusbcanfd.so /usr/lib/libusbcanfd.so.1.0.10
  ;;
"Darwin")
  ;;
"msys" | "MINGW"*)
  mkdir -p python/dll
  cp -vf dist/shared/win/*.dll python/dll/
  ;;
esac

# Create VERSION file
echo_y "[bc-stark-sdk] Creating version file..."
cat >"$VERSION_FILE" <<EOF
[bc-stark-sdk] Version: ${LIB_VERSION}
Update Time: $(date)
EOF

echo_y "[bc-stark-${LIB_NAME}-sdk] (${LIB_VERSION}) downloaded successfully to ${DIST_DIR}"
