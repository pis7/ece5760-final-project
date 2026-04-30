#!/bin/bash
#
# Run the placer with an increasing max-outer-iter from 1 to 16, capture the
# final placement JSON from each run, render a PNG of each via the visualizer,
# and stitch the PNGs into a looping GIF + MP4. Stops early once the placer
# converges in fewer iterations than the requested cap (with a 2-frame
# minimum so the slideshow always has something to animate).
#
# Usage:
#   ./placer-sweep.sh <mode> <benchmark-path> [extra]
#
# Modes (mirrors run-placer.sh):
#   sw                          Local C++ placer, double precision
#   golden                      Local C++ placer with fixed-point golden CG
#   verilated [v2|v3]           Local C++ placer with Verilator RTL CG
#                               (default v3; slow per iteration)
#   arm                         Cross-compile, run on DE1-SoC ARM
#   fpga                        Cross-compile FPGA-accelerated placer, run on
#                               DE1-SoC (FPGA bitstream must already be loaded)
#
# Examples:
#   ./placer-sweep.sh sw benchmarks/iccad04/DMA
#   ./placer-sweep.sh verilated benchmarks/custom/tiny3
#   ./placer-sweep.sh verilated benchmarks/custom/tiny3 v2
#   ./placer-sweep.sh fpga benchmarks/custom/tiny1

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# DE1-SoC board settings (mirror run-placer.sh).
BOARD="root@10.253.17.19"
PASS="greatpassword123!"

MAX_SWEEP_ITER=16
MIN_FRAMES=2

# -------------------------------------------------------------------
# Parse arguments
# -------------------------------------------------------------------

MODE="${1:-}"
BENCH_PATH="${2:-}"
EXTRA="${3:-}"

usage() {
    echo "Usage: ./placer-sweep.sh <mode> <benchmark-path> [extra]"
    echo ""
    echo "Modes:"
    echo "  sw                  Local C++ placer, double precision"
    echo "  golden              Local C++ placer with fixed-point golden CG"
    echo "  verilated [v2|v3]   Local C++ placer with Verilator RTL CG (default v3)"
    echo "  arm                 Cross-compile, run on DE1-SoC ARM"
    echo "  fpga                Cross-compile FPGA-accelerated placer, run on DE1-SoC"
}

if [ -z "$MODE" ] || [ -z "$BENCH_PATH" ]; then
    usage
    exit 1
fi

case "$MODE" in
    sw|golden|verilated|arm|fpga) ;;
    *)
        echo "Error: unknown mode '$MODE'"
        usage
        exit 1
        ;;
esac

HW_VERSION=""
if [ "$MODE" = "verilated" ]; then
    HW_VERSION="${EXTRA:-v3}"
    if [ "$HW_VERSION" != "v2" ] && [ "$HW_VERSION" != "v3" ]; then
        echo "Error: verilated mode expects 'v2' or 'v3' as the third arg (got '$HW_VERSION')"
        exit 1
    fi
fi

IS_REMOTE=0
if [ "$MODE" = "arm" ] || [ "$MODE" = "fpga" ]; then
    IS_REMOTE=1
fi

# -------------------------------------------------------------------
# Step 1: Parse LEF/DEF to JSON
# -------------------------------------------------------------------

echo "=== Generating JSON ==="
uv run "$SCRIPT_DIR/python-utils/lefdef-parser.py" "$BENCH_PATH"

JSON_FILE=$(ls -t *.json 2>/dev/null | head -1)
if [ -z "$JSON_FILE" ]; then
    echo "Error: no JSON file generated"
    exit 1
fi
DESIGN_NAME=${JSON_FILE%.json}
echo "  Design: $DESIGN_NAME ($JSON_FILE)"

# -------------------------------------------------------------------
# Step 2: Build placer for the chosen mode
# -------------------------------------------------------------------

case "$MODE" in
sw)
    echo "=== Building C++ placer (software) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/"
    make -j
    ;;
golden)
    echo "=== Building C++ placer (FP golden CG) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/" -DUSE_FP_GOLDEN=ON
    make -j
    ;;
verilated)
    echo "=== Building C++ placer (Verilator CG, $HW_VERSION) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/" -DUSE_HW_CG=ON -DHW_CG_VERSION="$HW_VERSION"
    make -j
    ;;
arm)
    echo "=== Cross-compiling placer for ARM ==="
    cmake -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DSTATIC_BUILD=ON \
        "$SCRIPT_DIR/sw-baseline-c/"
    make -j
    ;;
fpga)
    echo "=== Cross-compiling FPGA placer for ARM ==="
    cmake -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DSTATIC_BUILD=ON \
        "$SCRIPT_DIR/fpga/sw/"
    make -j
    ;;
esac

# -------------------------------------------------------------------
# Step 2b: Stage to board (arm/fpga only)
# -------------------------------------------------------------------

if [ $IS_REMOTE -eq 1 ]; then
    if [ "$MODE" = "fpga" ]; then
        BOARD_DIR="/home/root/build-${DESIGN_NAME}-fpga-sweep"
    else
        BOARD_DIR="/home/root/build-${DESIGN_NAME}-sweep"
    fi
    echo "=== Copying to board ($BOARD_DIR) ==="
    sshpass -p "$PASS" ssh "$BOARD" "mkdir -p $BOARD_DIR"
    sshpass -p "$PASS" scp placer "$BOARD":"$BOARD_DIR"/
    sshpass -p "$PASS" scp "$JSON_FILE" "$BOARD":"$BOARD_DIR"/
fi

# -------------------------------------------------------------------
# Step 3: Sweep loop
# -------------------------------------------------------------------

TMP_OUT=$(mktemp)
trap 'rm -f "$TMP_OUT"' EXIT

FRAMES=()

echo "=== Running sweep (1..$MAX_SWEEP_ITER) ==="
for N in $(seq 1 $MAX_SWEEP_ITER); do
    NN=$(printf "%02d" "$N")

    # Run placer (locally or on the board). Use plain ssh, NOT ssh -t -- a
    # TTY corrupts the marker lines we need to grep out of stdout.
    RC=0
    if [ $IS_REMOTE -eq 1 ]; then
        sshpass -p "$PASS" ssh "$BOARD" \
            "cd $BOARD_DIR && ./placer $JSON_FILE $N" \
            > "$TMP_OUT" 2>&1 || RC=$?
    else
        ./placer "$JSON_FILE" "$N" > "$TMP_OUT" 2>&1 || RC=$?
    fi
    if [ $RC -ne 0 ]; then
        cat "$TMP_OUT" >&2
        echo "Placer failed at N=$N (rc=$RC)" >&2
        exit 1
    fi

    # Parse the markers printed at the end of placer.cpp main().
    ITERS_USED=$(grep "^Outer iterations used:" "$TMP_OUT" | awk '{print $4}')
    CONVERGED=$(grep "^Converged:" "$TMP_OUT" | awk '{print $2}')
    REVERTED=$(grep "^Reverted:" "$TMP_OUT" | awk '{print $2}')
    if [ -z "$ITERS_USED" ] || [ -z "$CONVERGED" ] || [ -z "$REVERTED" ]; then
        cat "$TMP_OUT" >&2
        echo "Could not find marker lines in placer output at N=$N." >&2
        echo "Did placer.cpp print 'Outer iterations used: K'?" >&2
        exit 1
    fi

    # Capture the per-iter final JSON before the next run overwrites it.
    PER_ITER_JSON="${DESIGN_NAME}-final-iter${NN}.json"
    if [ $IS_REMOTE -eq 1 ]; then
        sshpass -p "$PASS" scp \
            "$BOARD":"$BOARD_DIR/${DESIGN_NAME}-final.json" "$PER_ITER_JSON"
    else
        if [ ! -f "${DESIGN_NAME}-final.json" ]; then
            echo "Error: expected ${DESIGN_NAME}-final.json after placer run" >&2
            exit 1
        fi
        mv "${DESIGN_NAME}-final.json" "$PER_ITER_JSON"
    fi

    # Render a PNG frame for this iteration.
    PER_ITER_PNG="${DESIGN_NAME}-final-iter${NN}.png"
    uv run "$SCRIPT_DIR/python-utils/visualizer.py" \
        --png "$PER_ITER_PNG" "$PER_ITER_JSON" > /dev/null
    FRAMES+=("$PER_ITER_PNG")

    if [ "$CONVERGED" = "true" ]; then
        TAG="converged"
    elif [ "$REVERTED" = "true" ]; then
        TAG="reverted"
    else
        TAG="capped"
    fi
    printf "  iter %2d/%d: kept=%d (%s)\n" \
        "$N" "$MAX_SWEEP_ITER" "$ITERS_USED" "$TAG"

    # Early-stop: if the placer didn't actually need N iterations, the result
    # at N+1 would be identical. Always emit at least MIN_FRAMES frames first
    # so the slideshow has something to animate.
    if [ "$ITERS_USED" -lt "$N" ] && [ "$N" -ge "$MIN_FRAMES" ]; then
        echo "  Placer used only $ITERS_USED of $N iterations -- stopping sweep."
        break
    fi
done

# -------------------------------------------------------------------
# Step 4: Slideshow
# -------------------------------------------------------------------

echo "=== Building slideshow (${#FRAMES[@]} frames) ==="
uv run "$SCRIPT_DIR/python-utils/slideshow.py" \
    "${DESIGN_NAME}-sweep" "${FRAMES[@]}"

echo "=== Done ==="
echo "  Frames: ${DESIGN_NAME}-final-iter*.png (${#FRAMES[@]} files)"
echo "  Slideshow: ${DESIGN_NAME}-sweep.gif, ${DESIGN_NAME}-sweep.mp4"
