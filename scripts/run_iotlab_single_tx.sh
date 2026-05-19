#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

REMOTE="eckertre@grenoble.iot-lab.info"
SSH_BASE=(ssh -i ~/.ssh/id_rsa -o IdentitiesOnly=yes "$REMOTE")
SCP_BASE=(scp -i ~/.ssh/id_rsa -o IdentitiesOnly=yes)

usage() {
  cat <<'EOF'
Usage:
  run_iotlab_single_tx.sh --bin PATH --label LABEL [--profile PROFILE] [--serial-sec SEC] [--duration-min MIN]
EOF
}

BIN_PATH=""
LABEL=""
PROFILE_NAME="en_stm32"
SERIAL_CAPTURE_SEC="${SERIAL_CAPTURE_SEC:-600}"
EXP_DURATION_MIN="${EXP_DURATION_MIN:-15}"

while [ $# -gt 0 ]; do
  case "$1" in
    --bin)
      BIN_PATH="$2"
      shift 2
      ;;
    --label)
      LABEL="$2"
      shift 2
      ;;
    --profile)
      PROFILE_NAME="$2"
      shift 2
      ;;
    --serial-sec)
      SERIAL_CAPTURE_SEC="$2"
      shift 2
      ;;
    --duration-min)
      EXP_DURATION_MIN="$2"
      shift 2
      ;;
    *)
      usage >&2
      exit 1
      ;;
  esac
done

if [ -z "$BIN_PATH" ] || [ -z "$LABEL" ]; then
  usage >&2
  exit 1
fi

if [ ! -f "$BIN_PATH" ]; then
  echo "Missing firmware: $BIN_PATH" >&2
  exit 1
fi

TMP_LOG="$(mktemp)"
trap 'rm -f "$TMP_LOG"' EXIT

COMMANDS_LOG=""

run_logged_capture() {
  local expect_status="$1"
  shift
  local -a cmd=("$@")
  local output=""
  local exit_code=0

  {
    printf '$'
    printf ' %q' "${cmd[@]}"
    printf '\n'
  } | tee -a "${COMMANDS_LOG:-$TMP_LOG}" >/dev/null

  set +e
  output="$("${cmd[@]}" 2>&1)"
  exit_code=$?
  set -e

  if [ -n "$output" ]; then
    printf "%s\n" "$output" | tee -a "${COMMANDS_LOG:-$TMP_LOG}" >/dev/null
  else
    printf "(no stdout)\n" | tee -a "${COMMANDS_LOG:-$TMP_LOG}" >/dev/null
  fi
  printf '\n' | tee -a "${COMMANDS_LOG:-$TMP_LOG}" >/dev/null

  if [ "$exit_code" -ne "$expect_status" ]; then
    return 1
  fi

  printf "%s" "$output"
  return 0
}

retry_logged() {
  local attempts="$1"
  local sleep_sec="$2"
  local expect_status="$3"
  shift 3
  local i

  for ((i = 1; i <= attempts; i++)); do
    if run_logged_capture "$expect_status" "$@" >/dev/null; then
      return 0
    fi
    sleep "$sleep_sec"
  done
  return 1
}

slugify() {
  printf "%s" "$1" | tr '[:upper:]' '[:lower:]' | sed 's/[^a-z0-9]/_/g; s/__*/_/g; s/^_//; s/_$//'
}

LABEL_SLUG="$(slugify "$LABEL")"
REMOTE_BIN="TX_${LABEL_SLUG}.bin"

submit_output="$(
  run_logged_capture 0 "${SSH_BASE[@]}" \
    "iotlab-experiment submit -n wl55_tx_${LABEL_SLUG} -d ${EXP_DURATION_MIN} -l grenoble,nucleo-wl55jc,1"
)"
EXP_ID="$(printf "%s\n" "$submit_output" | python3 -c 'import json,sys; print(json.load(sys.stdin)["id"])')"

EXP_DIR="$REPO_ROOT/experiment/${EXP_ID} - ${LABEL}"
RAW_DIR="$EXP_DIR/raw"
INTERACTION_DIR="$EXP_DIR/interaction"
PROCESSED_DIR="$EXP_DIR/processed"
mkdir -p "$RAW_DIR" "$INTERACTION_DIR" "$PROCESSED_DIR"

COMMANDS_LOG="$INTERACTION_DIR/commands.log"
{
  printf "# IoT-LAB interaction log for experiment %s\n" "$EXP_ID"
  printf "# Generated at %s\n" "$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
  printf "# Variant label: %s\n" "$LABEL"
  printf "# Energy profile: %s\n" "$PROFILE_NAME"
  printf "# TX.bin sha256: %s\n\n" "$(shasum -a 256 "$BIN_PATH" | awk '{print $1}')"
  cat "$TMP_LOG"
} > "$COMMANDS_LOG"

retry_logged 12 10 0 "${SSH_BASE[@]}" "hostname"
retry_logged 12 10 0 "${SSH_BASE[@]}" "iotlab-experiment wait -i ${EXP_ID} --state Running --step 1 --timeout 180"
retry_logged 6 10 0 "${SCP_BASE[@]}" "$BIN_PATH" "${REMOTE}:~/${REMOTE_BIN}"
retry_logged 6 10 0 "${SSH_BASE[@]}" "iotlab-node -i ${EXP_ID} --profile ${PROFILE_NAME} -l grenoble,nucleo-wl55jc,1"
retry_logged 6 10 0 "${SSH_BASE[@]}" "iotlab-node -i ${EXP_ID} --flash ~/${REMOTE_BIN} -l grenoble,nucleo-wl55jc,1"
retry_logged 6 10 0 "${SSH_BASE[@]}" "iotlab-node -i ${EXP_ID} --reset -l grenoble,nucleo-wl55jc,1"

retry_logged 3 10 0 "${SSH_BASE[@]}" \
  "nohup sh -c 'timeout ${SERIAL_CAPTURE_SEC}s nc nucleo-wl55jc-1 20000 > ~/serial_${EXP_ID}_raw.log' >/dev/null 2>&1 &"
sleep "$((SERIAL_CAPTURE_SEC + 10))"

retry_logged 6 10 0 "${SSH_BASE[@]}" "iotlab-experiment stop -i ${EXP_ID}"
retry_logged 12 10 0 "${SSH_BASE[@]}" "iotlab-experiment wait -i ${EXP_ID} --state Stopped,Terminated,Error --step 1 --timeout 180"
retry_logged 6 10 0 "${SSH_BASE[@]}" "iotlab-experiment get -i ${EXP_ID} -a"

retry_logged 6 10 0 "${SCP_BASE[@]}" "${REMOTE}:~/${EXP_ID}.tar.gz" "$EXP_DIR/${EXP_ID}.tar.gz"
retry_logged 6 10 0 "${SCP_BASE[@]}" \
  "${REMOTE}:~/.senslab/${EXP_ID}/log/nucleo_wl55jc_1.log" \
  "$RAW_DIR/nucleo_wl55jc_1.log"
retry_logged 6 10 0 "${SCP_BASE[@]}" \
  "${REMOTE}:~/.senslab/${EXP_ID}/consumption/nucleo_wl55jc_1.oml" \
  "$RAW_DIR/nucleo_wl55jc_1.oml"
retry_logged 6 10 0 "${SCP_BASE[@]}" "${REMOTE}:~/serial_${EXP_ID}_raw.log" "$RAW_DIR/serial.log"

tar -xzf "$EXP_DIR/${EXP_ID}.tar.gz" -C "$EXP_DIR"

ts="$(date +%s)"
awk -v ts="$ts" '{print ts "\t" $0}' "$RAW_DIR/serial.log" > "$RAW_DIR/serial_ts.log"

if ! python3 "$REPO_ROOT/experiment/plot_energy.py" --exp-dir "$EXP_DIR"; then
  printf "\n# plot_energy.py fallback: matplotlib unavailable, using single-run summary processor\n" >> "$COMMANDS_LOG"
  python3 "$REPO_ROOT/scripts/process_iotlab_single_run.py" --exp-dir "$EXP_DIR"
fi

printf "%s\n" "$EXP_ID"
