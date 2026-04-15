#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"

CMAKE_BIN="${ROOT_DIR}/vcpkg/downloads/tools/cmake-4.2.3-linux/cmake-4.2.3-linux-x86_64/bin/cmake"
TOOLCHAIN_FILE="${ROOT_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"

if [[ ! -x "${CMAKE_BIN}" ]]; then
  CMAKE_BIN="cmake"
fi

# 优先使用本地安装的 Autoconf（2.71+），避免系统版本过低导致 vcpkg 构建失败
if [[ -x "${ROOT_DIR}/.local/bin/autoconf" ]]; then
  export PATH="${ROOT_DIR}/.local/bin:${PATH}"
fi

"${CMAKE_BIN}" -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"

"${CMAKE_BIN}" --build "${BUILD_DIR}" -j
