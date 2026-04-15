#!/bin/bash

BOARD="root@10.253.17.19"
PASS="greatpassword123!"

# Parse arguments
ARM=false
BENCH_PATH=""
for arg in "$@"; do
    if [ "$arg" = "--arm" ]; then
        ARM=true
    else
        BENCH_PATH="$arg"
    fi
done

if [ -z "$BENCH_PATH" ]; then
    echo "Usage: ./run-placer.sh <benchmark-path> [--arm]"
    echo "  e.g. ./run-placer.sh benchmarks/iccad04/DMA"
    echo "       ./run-placer.sh benchmarks/custom/tiny1 --arm"
    exit 1
fi

# Generate the design JSON
echo "=== Generating JSON ==="
uv run ../design-file-tools/lefdef-parser.py "$BENCH_PATH"
JSON_FILE=$(ls -t *.json 2>/dev/null | head -1)
if [ -z "$JSON_FILE" ]; then
    echo "Error: no JSON file generated"
    exit 1
fi
DESIGN_NAME=$(echo "$JSON_FILE" | sed 's/\.json$//')
echo "  Design: $DESIGN_NAME ($JSON_FILE)"

if [ "$ARM" = true ]; then
    # Cross-compile placer for ARM
    echo "=== Cross-compiling placer ==="
    cmake -DCMAKE_CXX_COMPILER=arm-linux-gnueabihf-g++ -DSTATIC_BUILD=ON ../sw-baseline-c/
    make

    # Copy placer binary and JSON to a new build directory on the board
    BOARD_DIR="/home/root/build-${DESIGN_NAME}"
    echo "=== Copying to board ($BOARD_DIR) ==="
    sshpass -p "$PASS" ssh "$BOARD" "mkdir -p $BOARD_DIR"
    sshpass -p "$PASS" scp placer "$BOARD":"$BOARD_DIR"/
    sshpass -p "$PASS" scp "$JSON_FILE" "$BOARD":"$BOARD_DIR"/

    # Run placer on the board
    echo "=== Running placer on board ==="
    sshpass -p "$PASS" ssh -t "$BOARD" "cd $BOARD_DIR && ./placer $JSON_FILE"

    # Copy back initial and final JSONs
    echo "=== Copying results back ==="
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-initial.json .
    sshpass -p "$PASS" scp "$BOARD":"$BOARD_DIR"/${DESIGN_NAME}-final.json .
else
    # Build placer natively
    echo "=== Building placer ==="
    cmake ../sw-baseline-c/
    make

    # Run placer
    echo "=== Running placer ==="
    ./placer "$JSON_FILE"
fi

echo "  Results: ${DESIGN_NAME}-initial.json, ${DESIGN_NAME}-final.json"
