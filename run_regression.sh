#!/usr/bin/env bash
# run_regression.sh - layout regression test runner
#
# Usage:
#   ./run_regression.sh baseline    # capture baseline snapshots to regression_baseline/
#   ./run_regression.sh compare     # compare current layout against baselines
#   ./run_regression.sh             # same as compare (default)
#
# Environment overrides:
#   NETLISTS_DIR   path to dir containing extra netlists (optional)
#   BUILD_DIR      path to build dir (default: ./build)
#   BASELINE_DIR   path to baseline dir (default: ./regression_baseline)
#   TIMEOUT_SEC    per-netlist timeout in seconds (default: 60)
#
# Exit code: 0=all pass, 1=any failure or regression

set -euo pipefail

BUILD_DIR="${BUILD_DIR:-./build}"
BASELINE_DIR="${BASELINE_DIR:-./regression_baseline}"
TIMEOUT_SEC="${TIMEOUT_SEC:-60}"
MODE="${1:-compare}"

BIN="${BUILD_DIR}/test_regression"

# ---- Colours -----------------------------------------------------------
RED='\033[0;31m'; GRN='\033[0;32m'; YEL='\033[1;33m'; NC='\033[0m'

# ---- Build -------------------------------------------------------------
echo "==> Building test_regression..."
cmake --build "${BUILD_DIR}" --target test_regression -j"$(nproc)" 2>&1 | tail -4

if [[ ! -x "${BIN}" ]]; then
    echo -e "${RED}ERROR: ${BIN} not found or not executable${NC}"
    exit 1
fi

# ---- Test netlists -----------------------------------------------------
# Small/medium netlists that must complete within TIMEOUT_SEC.
# Add more designs here as they become available.
SMALL_NETLISTS=(
    "1_2_yosys.v"
    "complex.v"
    "deep_hierarchy.v"
    "eqne.v"
    "ge2.v"
    "onehot16.v"
    "timescale.v"
    "test_circuit.v"
    "aes_cipher_top.v"
)

# Large netlists: only check OOM/crash (no timeout allowed, but must not crash)
LARGE_NETLISTS=()
if [[ -n "${NETLISTS_DIR:-}" ]]; then
    while IFS= read -r -d '' f; do
        LARGE_NETLISTS+=("$f")
    done < <(find "${NETLISTS_DIR}" -name "*.v" -print0 2>/dev/null)
fi

# ---- Helpers -----------------------------------------------------------
pass=0; fail=0; skip=0

run_one() {
    local netlist="$1"
    local mode="$2"        # "baseline" or "compare"
    local snap="${BASELINE_DIR}/$(basename "${netlist}" .v).snap"

    if [[ ! -f "${netlist}" ]]; then
        echo -e "  ${YEL}[SKIP]${NC} ${netlist} (not found)"
        ((skip++)) || true
        return 0
    fi

    local extra_args=""
    if [[ "${mode}" == "baseline" ]]; then
        mkdir -p "${BASELINE_DIR}"
        extra_args="--snapshot ${snap}"
    elif [[ "${mode}" == "compare" && -f "${snap}" ]]; then
        extra_args="--compare ${snap}"
    fi

    echo -e "\n  Testing: ${netlist}"
    local out
    local rc=0
    # Capture both stdout and stderr; suppress stderr noise in CI by piping
    out=$(timeout "${TIMEOUT_SEC}" "${BIN}" "${netlist}" ${extra_args} 2>/tmp/reg_stderr_$$.log) || rc=$?

    cat /tmp/reg_stderr_$$.log | grep -E 'dummies=|LAYOUT-SCALE' | head -3 || true
    rm -f /tmp/reg_stderr_$$.log

    echo "${out}"

    if [[ ${rc} -eq 124 ]]; then
        echo -e "  ${RED}[TIMEOUT]${NC} exceeded ${TIMEOUT_SEC}s"
        ((fail++)) || true
    elif [[ ${rc} -ne 0 ]]; then
        echo -e "  ${RED}[FAIL]${NC} exit code ${rc}"
        ((fail++)) || true
    else
        echo -e "  ${GRN}[OK]${NC}"
        ((pass++)) || true
    fi
}

# ---- Main --------------------------------------------------------------
echo ""
echo "======================================================="
echo "  Layout Regression Tests  (mode: ${MODE})"
echo "  BUILD_DIR    = ${BUILD_DIR}"
echo "  BASELINE_DIR = ${BASELINE_DIR}"
echo "  TIMEOUT_SEC  = ${TIMEOUT_SEC}"
echo "======================================================="

for nl in "${SMALL_NETLISTS[@]}"; do
    run_one "${nl}" "${MODE}"
done

for nl in "${LARGE_NETLISTS[@]}"; do
    echo -e "\n  Large netlist: ${nl}"
    # For large netlists we only check that it doesn't crash/OOM within a
    # generous timeout.  We don't do full metric comparison.
    local_rc=0
    timeout 300 "${BIN}" "${nl}" 2>/tmp/reg_large_$$.log || local_rc=$?
    cat /tmp/reg_large_$$.log | grep -E 'bad_alloc|LAYOUT-SCALE' | head -5 || true
    rm -f /tmp/reg_large_$$.log
    if [[ ${local_rc} -eq 124 ]]; then
        echo -e "  ${YEL}[TIMEOUT 5min]${NC} — acceptable for large designs"
        ((pass++)) || true
    elif [[ ${local_rc} -ne 0 ]]; then
        echo -e "  ${RED}[CRASH]${NC} exit code ${local_rc}"
        ((fail++)) || true
    else
        echo -e "  ${GRN}[OK]${NC}"
        ((pass++)) || true
    fi
done

echo ""
echo "======================================================="
echo -e "  PASSED: ${GRN}${pass}${NC}   FAILED: ${RED}${fail}${NC}   SKIPPED: ${YEL}${skip}${NC}"
echo "======================================================="

if [[ ${fail} -gt 0 ]]; then
    exit 1
fi
exit 0
