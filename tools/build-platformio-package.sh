#!/usr/bin/env bash

set -euo pipefail

PACKAGE_NAME="framework-arduinoadafruitnrf52"
REPO_URL="https://github.com/stealth30110/Adafruit_nRF52_Arduino"

usage() {
  echo "Usage: $0 <version> [output-dir]" >&2
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

VERSION="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${2:-${REPO_ROOT}/dist}"
STAGE_ROOT="$(mktemp -d)"
PACKAGE_ROOT="${STAGE_ROOT}/${PACKAGE_NAME}"
ARCHIVE_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}.tar.bz2"

cleanup() {
  rm -rf "${STAGE_ROOT}"
}

trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}" "${PACKAGE_ROOT}"

# Copy only the files that are part of the framework package.
PACKAGE_CONTENTS=(
  boards.txt
  bootloader
  changelog.md
  CODE_OF_CONDUCT.md
  cores
  keywords.txt
  libraries
  LICENSE
  platform.txt
  programmers.txt
  README.md
  scripts
  tools
  variants
)

for entry in "${PACKAGE_CONTENTS[@]}"; do
  if [[ -e "${REPO_ROOT}/${entry}" ]]; then
    cp -a "${REPO_ROOT}/${entry}" "${PACKAGE_ROOT}/"
  fi
done

cat > "${PACKAGE_ROOT}/package.json" <<EOF
{
  "name": "${PACKAGE_NAME}",
  "version": "${VERSION}",
  "description": "Arduino Wiring-based Framework for Nordic Semiconductor nRF52 BLE SoC",
  "keywords": [
    "framework",
    "arduino",
    "nrf52"
  ],
  "repository": {
    "type": "git",
    "url": "${REPO_URL}"
  },
  "homepage": "${REPO_URL}",
  "license": "LGPL-2.1-or-later",
  "system": [
    "*"
  ]
}
EOF

rm -f "${ARCHIVE_PATH}"
tar -cjf "${ARCHIVE_PATH}" -C "${STAGE_ROOT}" "${PACKAGE_NAME}"

echo "Created ${ARCHIVE_PATH}"
