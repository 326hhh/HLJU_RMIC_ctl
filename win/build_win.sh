#!/usr/bin/env bash
# ===========================================================================
#  build_win.sh —— 把 win/main_win.cpp 交叉编译成 Windows 版 gimbal_ctl.exe
#
#  用法:
#      ./build_win.sh                       # 默认查找 zig，找不到用 mingw-w64 g++
#      ./build_win.sh --debug               # -O0 -g（排查用）
#      GIMBAL_ROOT=~/gimbal ./build_win.sh  # 线协议头文件不在 ../../gimbal 时指定
#
#  产物: build/gimbal_ctl.exe （64 位 Windows，静态链接，拷过去双击就能跑）
# ===========================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${HERE}/build"
mkdir -p "${OUT_DIR}"

# 线协议头文件（gimbal/link/control_link.hpp）所在工程的根目录
GIMBAL_ROOT="${GIMBAL_ROOT:-${HERE}/../../gimbal}"
if [[ ! -f "${GIMBAL_ROOT}/include/gimbal/link/control_link.hpp" ]]; then
    echo "找不到 ${GIMBAL_ROOT}/include/gimbal/link/control_link.hpp" >&2
    echo "用 GIMBAL_ROOT=<gimbal 工程路径> ./build_win.sh 指定" >&2
    exit 1
fi

OPT_FLAGS=(-O2)
LIBS=(-lws2_32 -lgdi32 -luser32 -lwinmm)

case "${1:-}" in
    --debug) OPT_FLAGS=(-O0 -g) ;;
    "")      ;;
    *)       echo "未知参数: $1" >&2; exit 2 ;;
esac

# ---- 找编译器：优先 zig（自带 mingw-w64 sysroot，无需 root 装包）----
ZIG_BIN="$(command -v zig || true)"
if [[ -z "${ZIG_BIN}" ]]; then
    for cand in "${HOME}"/tools/zig-*/zig "${HERE}"/../tools/zig-*/zig; do
        if [[ -x "${cand}" ]]; then ZIG_BIN="${cand}"; break; fi
    done
fi

if [[ -n "${ZIG_BIN}" ]]; then
    echo "[build] 使用 zig: ${ZIG_BIN} ($("${ZIG_BIN}" version))"
    CXX_CMD=("${ZIG_BIN}" c++)
    CXX_FLAGS=(-target x86_64-windows-gnu -static)
elif command -v x86_64-w64-mingw32-g++ >/dev/null 2>&1; then
    echo "[build] 使用 x86_64-w64-mingw32-g++"
    CXX_CMD=(x86_64-w64-mingw32-g++)
    CXX_FLAGS=(-static)
else
    echo "没找到 zig，也没有 x86_64-w64-mingw32-g++" >&2
    echo "装一个: sudo apt install g++-mingw-w64-x86-64   （或下载 zig 解压到 ~/tools）" >&2
    exit 1
fi

echo "[build] gimbal 头文件: ${GIMBAL_ROOT}/include"
"${CXX_CMD[@]}" "${CXX_FLAGS[@]}" -std=c++17 "${OPT_FLAGS[@]}" \
    -Wall -Wextra -Wno-unused-parameter \
    -I"${HERE}" -I"${GIMBAL_ROOT}/include" \
    "${HERE}/main_win.cpp" \
    -o "${OUT_DIR}/gimbal_ctl.exe" \
    "${LIBS[@]}" 2>&1 | grep -v -E "libunwind|_Unwind_|previous declaration|warnings generated|^\s+\||^\s+[0-9]+ \|" || true

if [[ -f "${OUT_DIR}/gimbal_ctl.exe" ]]; then
    echo "[build] 完成: ${OUT_DIR}/gimbal_ctl.exe ($(stat -c%s "${OUT_DIR}/gimbal_ctl.exe") 字节)"
    file "${OUT_DIR}/gimbal_ctl.exe"
else
    echo "[build] 失败：没生成 exe" >&2
    exit 1
fi
