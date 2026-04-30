#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 <base-version> [run-number]" >&2
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

BASE_VERSION="$1"
RUN_NUMBER="${2:-${GITHUB_RUN_NUMBER:-0}}"

if [[ ! "${BASE_VERSION}" =~ ^[0-9]+\.[0-9]+$ ]]; then
  echo "Base version must look like MAJOR.MINOR, got: ${BASE_VERSION}" >&2
  exit 1
fi

if [[ ! "${RUN_NUMBER}" =~ ^[0-9]+$ ]]; then
  echo "Run number must be numeric, got: ${RUN_NUMBER}" >&2
  exit 1
fi

echo "${BASE_VERSION}.${RUN_NUMBER}"
