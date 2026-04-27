#!/bin/bash
#
# Run the analytical placer end-to-end.
#
# Usage:
#   ./run-placer.sh <mode> <path>
#
# Modes:
#   python    <benchmark-path>   Baseline Python placer
#   sw        <benchmark-path>   Full software C++ placer (double precision)
#   golden    <benchmark-path>   C++ placer with fixed-point golden model CG
#   verilated <benchmark-path>   C++ placer with Verilator RTL CG simulation
#   arm       <benchmark-path>   Cross-compile SW placer, run on DE1-SoC ARM
#   fpga      <benchmark-path>   Cross-compile FPGA-accelerated placer, run on DE1-SoC
#                                (FPGA bitstream must already be loaded on the board)
#   vis       <json-file>        Launch the Tk visualizer on a placement JSON
#
# Examples:
#   ./run-placer.sh python benchmarks/iccad04/DMA
#   ./run-placer.sh sw benchmarks/custom/tiny3
#   ./run-placer.sh golden benchmarks/custom/tiny3
#   ./run-placer.sh arm benchmarks/custom/tiny1
#   ./run-placer.sh fpga benchmarks/custom/tiny1
#   ./run-placer.sh vis tiny3-final.json

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# DE1-SoC board settings
BOARD="root@10.253.17.19"
PASS="greatpassword123!"

# -------------------------------------------------------------------
# Parse arguments
# -------------------------------------------------------------------

MODE="${1:-}"
ARG="${2:-}"

if [ -z "$MODE" ] || [ -z "$ARG" ]; then
    echo "Usage: ./run-placer.sh <mode> <path>"
    echo ""
    echo "Modes:"
    echo "  python    <benchmark-path>   Baseline Python placer"
    echo "  sw        <benchmark-path>   Full software C++ placer (double precision)"
    echo "  golden    <benchmark-path>   C++ placer with fixed-point golden CG"
    echo "  verilated <benchmark-path>   C++ placer with Verilator RTL CG simulation"
    echo "  arm       <benchmark-path>   Cross-compile SW placer, run on DE1-SoC ARM"
    echo "  fpga      <benchmark-path>   Cross-compile FPGA-accelerated placer, run on DE1-SoC"
    echo "  vis       <json-file>        Launch the Tk visualizer on a placement JSON"
    exit 1
fi

# -------------------------------------------------------------------
# Visualizer mode (no build needed)
# -------------------------------------------------------------------

if [ "$MODE" = "vis" ]; then
    if [ ! -f "$ARG" ]; then
        echo "Error: $ARG not found"
        exit 1
    fi
    echo "=== Launching visualizer ==="
    uv run "$SCRIPT_DIR/design-file-tools/visualizer.py" "$ARG"
    exit 0
fi

# -------------------------------------------------------------------
# Step 1: Parse LEF/DEF to JSON
# -------------------------------------------------------------------

BENCH_PATH="$ARG"

echo "=== Generating JSON ==="
uv run "$SCRIPT_DIR/design-file-tools/lefdef-parser.py" "$BENCH_PATH"

JSON_FILE=$(ls -t *.json 2>/dev/null | head -1)
if [ -z "$JSON_FILE" ]; then
    echo "Error: no JSON file generated"
    exit 1
fi
DESIGN_NAME=$(echo "$JSON_FILE" | sed 's/\.json$//')
echo "  Design: $DESIGN_NAME ($JSON_FILE)"

# -------------------------------------------------------------------
# Step 2: Build and run placer
# -------------------------------------------------------------------

case "$MODE" in

python)
    echo "=== Running Python placer ==="
    uv run "$SCRIPT_DIR/sw-baseline-python/placer.py" "$JSON_FILE"
    ;;

sw)
    echo "=== Building C++ placer (software) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/"
    make -j"$(nproc)"

    echo "=== Running placer ==="
    ./placer "$JSON_FILE"
    ;;

golden)
    echo "=== Building C++ placer (FP golden CG) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/" -DUSE_FP_GOLDEN=ON
    make -j"$(nproc)"

    echo "=== Running placer ==="
    ./placer "$JSON_FILE"
    ;;

verilated)
    echo "=== Building C++ placer (Verilator CG) ==="
    cmake "$SCRIPT_DIR/sw-baseline-c/" -DUSE_HW_CG=ON
    make -j"$(nproc)"

    echo "=== Running placer ==="
    ./placer "$JSON_FILE"
    ;;

arm)
    echo "=== Cross-compiling placer for ARM ==="
    cmake -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DSTATIC_BUILD=ON "$SCRIPT_DIR/sw-baseline-c/"
    make -j"$(nproc)"

    BOARD_DIR="/home/root/build-${DESIGN_NAME}"
    echo "=== Copying to board ($BOARD_DIR) ==="
    sshpass -p "$PASS" ssh "$BOARD" "mkdir -p $BOARD_DIR"
    sshpass -p "$PASS" scp placer "$BOARD":"$BOARD_DIR"/
    sshpass -p "$PASS" scp "$JSON_FILE" "$BOARD":"$BOARD_DIR"/

    echo "=== Running placer on board ==="
    sshpass -p "$PASS" ssh -t "$BOARD" "cd $BOARD_DIR && ./placer $JSON_FILE"

    echo "=== Copying results back ==="
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-initial.json .
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-final.json .
    ;;

fpga)
    echo "=== Cross-compiling FPGA placer for ARM ==="
    cmake -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DSTATIC_BUILD=ON "$SCRIPT_DIR/fpga/sw/"
    make -j"$(nproc)"

    BOARD_DIR="/home/root/build-${DESIGN_NAME}-fpga"
    echo "=== Copying to board ($BOARD_DIR) ==="
    sshpass -p "$PASS" ssh "$BOARD" "mkdir -p $BOARD_DIR"
    sshpass -p "$PASS" scp placer "$BOARD":"$BOARD_DIR"/
    sshpass -p "$PASS" scp "$JSON_FILE" "$BOARD":"$BOARD_DIR"/

    echo "=== Running placer on board (FPGA bitstream must already be loaded) ==="
    sshpass -p "$PASS" ssh -t "$BOARD" "cd $BOARD_DIR && ./placer $JSON_FILE"

    echo "=== Copying results back ==="
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-initial.json .
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-final.json .
    ;;

*)
    echo "Error: unknown mode '$MODE'"
    exit 1
    ;;

esac

echo "=== Done ==="
echo "  Results: ${DESIGN_NAME}-initial.json, ${DESIGN_NAME}-final.json"
