#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

AES_BIN="$REPO_ROOT/TX/SubGHz_Phy_PingPong/STM32CubeIDE/variants/TX_with_aes.bin"
NO_AES_BIN="$REPO_ROOT/TX/SubGHz_Phy_PingPong/STM32CubeIDE/variants/TX_without_aes.bin"
PROFILE_NAME="${PROFILE_NAME:-en_stm32}"
SERIAL_CAPTURE_SEC="${SERIAL_CAPTURE_SEC:-600}"
EXP_DURATION_MIN="${EXP_DURATION_MIN:-15}"
START_INDEX="${START_INDEX:-2}"
END_INDEX="${END_INDEX:-5}"

if [ ! -f "$AES_BIN" ] || [ ! -f "$NO_AES_BIN" ]; then
  echo "Missing compare binaries. Build TX_with_aes.bin and TX_without_aes.bin first." >&2
  exit 1
fi

MANIFEST_DIR="$REPO_ROOT/experiment"
MANIFEST_PATH="$MANIFEST_DIR/repeated_single_node_compare_manifest.txt"
touch "$MANIFEST_PATH"

run_one() {
  local bin_path="$1"
  local label="$2"
  local exp_id=""

  exp_id="$(
    zsh "$SCRIPT_DIR/run_iotlab_single_tx.sh" \
      --bin "$bin_path" \
      --label "$label" \
      --profile "$PROFILE_NAME" \
      --serial-sec "$SERIAL_CAPTURE_SEC" \
      --duration-min "$EXP_DURATION_MIN"
  )"
  printf "%s\t%s\n" "$exp_id" "$label" | tee -a "$MANIFEST_PATH"
}

printf "# Repeated single-node AES vs no-AES manifest\n" > "$MANIFEST_PATH"
printf "# Generated at %s\n" "$(date -u +"%Y-%m-%dT%H:%M:%SZ")" >> "$MANIFEST_PATH"
printf "# Profile=%s SerialCaptureSec=%s DurationMin=%s\n" "$PROFILE_NAME" "$SERIAL_CAPTURE_SEC" "$EXP_DURATION_MIN" >> "$MANIFEST_PATH"
printf "# Existing matched runs:\n" >> "$MANIFEST_PATH"
printf "437276\twith AES rerun\n" >> "$MANIFEST_PATH"
printf "437275\twithout AES\n\n" >> "$MANIFEST_PATH"
printf "# New runs:\n" >> "$MANIFEST_PATH"

for ((idx = START_INDEX; idx <= END_INDEX; idx++)); do
  run_one "$AES_BIN" "with AES run $(printf '%02d' "$idx")"
  run_one "$NO_AES_BIN" "without AES run $(printf '%02d' "$idx")"
done
