#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TX_BUILD_DIR="$REPO_ROOT/TX/SubGHz_Phy_PingPong/STM32CubeIDE/Debug"
OUT_DIR="$REPO_ROOT/TX/SubGHz_Phy_PingPong/STM32CubeIDE/variants"

REAL_GCC="${ARM_GCC_PATH:-}"
if [ -z "$REAL_GCC" ]; then
  if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    REAL_GCC="$(command -v arm-none-eabi-gcc)"
  else
    REAL_GCC="/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.macos64_1.0.100.202602081740/tools/bin/arm-none-eabi-gcc"
  fi
fi

if [ ! -x "$REAL_GCC" ]; then
  echo "arm-none-eabi-gcc not found. Set ARM_GCC_PATH to the compiler binary." >&2
  exit 1
fi

TOOLCHAIN_DIR="$(cd "$(dirname "$REAL_GCC")" && pwd)"
WRAPPER_DIR="$(mktemp -d)"
trap 'rm -rf "$WRAPPER_DIR"' EXIT

cat > "$WRAPPER_DIR/arm-none-eabi-gcc" <<'EOF'
#!/bin/zsh
set -euo pipefail
if [ -z "${REAL_GCC:-}" ]; then
  echo "REAL_GCC is not set" >&2
  exit 1
fi
if [ -n "${USE_TX_AES_OVERRIDE:-}" ]; then
  exec "$REAL_GCC" -DUSE_TX_AES="${USE_TX_AES_OVERRIDE}" "$@"
else
  exec "$REAL_GCC" "$@"
fi
EOF
chmod +x "$WRAPPER_DIR/arm-none-eabi-gcc"

mkdir -p "$OUT_DIR"

build_variant() {
  local aes_flag="$1"
  local stem="$2"

  echo "Building $stem"
  PATH="$WRAPPER_DIR:$TOOLCHAIN_DIR:$PATH" make -C "$TX_BUILD_DIR" clean >/dev/null
  PATH="$WRAPPER_DIR:$TOOLCHAIN_DIR:$PATH" USE_TX_AES_OVERRIDE="$aes_flag" \
    make -C "$TX_BUILD_DIR" all -j2 REAL_GCC="$REAL_GCC"

  cp "$TX_BUILD_DIR/TX.bin" "$OUT_DIR/${stem}.bin"
  cp "$TX_BUILD_DIR/TX.elf" "$OUT_DIR/${stem}.elf"
  cp "$TX_BUILD_DIR/TX.map" "$OUT_DIR/${stem}.map"
}

build_variant 1 "TX_with_aes"
build_variant 0 "TX_without_aes"

echo "Generated variants in $OUT_DIR"
