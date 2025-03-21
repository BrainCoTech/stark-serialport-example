#!/bin/bash
set -e # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/dist"
VERSION_FILE="${SCRIPT_DIR}/VERSION"

# Configuration
LIB_VERSION="v0.1.9"
BASE_URL="https://app.brainco.cn/universal/bc-device-sdk/libs/${LIB_VERSION}"

# Colorful echo functions
echo_y() { echo -e "\033[1;33m$*\033[0m"; } # Yellow
echo_r() { echo -e "\033[0;31m$*\033[0m"; } # Red

# Check if version is already installed
if [ -f "$VERSION_FILE" ] && grep -F --quiet "$LIB_VERSION" "$VERSION_FILE"; then
  echo_y "[bc-device-sdk] (${LIB_VERSION}) is already installed"
  cat "$VERSION_FILE"
  exit 0
fi

# Determine platform and library name
PLATFORM=$(uname)
case "$PLATFORM" in
"Linux")
  LIB_NAME="linux"
  if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$ID" == "ubuntu" && "$VERSION_ID" == "20.04" ]]; then
      LIB_NAME="ubuntu-20"
    fi
  fi
  ;;
"Darwin")
  LIB_NAME="mac"
  ;;
"msys" | "MINGW"*)
  LIB_NAME="win"
  ;;
*)
  echo_r "Error: This script does not support your platform ($PLATFORM)"
  exit 1
  ;;
esac

ZIP_NAME="${LIB_NAME}.zip"
DOWNLOAD_URL="${BASE_URL}/${ZIP_NAME}?$(date +%s)" # Timestamp for uniqueness

# Clean up previous files
echo_y "[bc-device-sdk] Cleaning up previous distribution..."
rm -rf "$DIST_DIR" "${SCRIPT_DIR}/__MACOSX" "${SCRIPT_DIR}/${ZIP_NAME}"

# Create dist directory
mkdir -p "$DIST_DIR"

# Download library
echo_y "[bc-device-sdk] Downloading (${LIB_VERSION}) for ${LIB_NAME}..."
if ! command -v wget >/dev/null 2>&1; then
  echo_r "Error: wget is not installed. Please install it and try again."
  exit 1
fi

wget -q --show-progress "$DOWNLOAD_URL" -O "${SCRIPT_DIR}/${ZIP_NAME}" || {
  echo_r "Error: Failed to download ${ZIP_NAME}"
  exit 1
}

# Extract and clean up
echo_y "[bc-device-sdk] Extracting ${ZIP_NAME}..."
unzip -o -q "${SCRIPT_DIR}/${ZIP_NAME}" -d "$SCRIPT_DIR" || {
  echo_r "Error: Failed to unzip ${ZIP_NAME}"
  exit 1
}
rm -f "${SCRIPT_DIR}/${ZIP_NAME}"
rm -rf "${SCRIPT_DIR}/__MACOSX"
rm -rf "${DIST_DIR}/__MACOSX"
find dist/include -type f ! -name stark-sdk.h -exec rm -f {} \;

# copy the files to ros2_stark_ws
if [ "$PLATFORM" == "Linux" ]; then
  cp -vf dist/include/stark-sdk.h ros2_stark_ws/src/ros2_stark_controller/include/ros2_stark_controller/
  cp -vf dist/shared/linux/*.so ros2_stark_ws/src/ros2_stark_controller/lib/
fi

# Create VERSION file
echo_y "[bc-device-sdk] Creating version file..."
cat >"$VERSION_FILE" <<EOF
[bc-device-sdk] Version: ${LIB_VERSION}
Update Time: $(date)
EOF

echo_y "[bc-device-${LIB_NAME}-sdk] (${LIB_VERSION}) downloaded successfully to ${DIST_DIR}"
