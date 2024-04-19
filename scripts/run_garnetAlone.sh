#!/bin/bash

# Check if the correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <cfc_value> <injection_rate>"
    exit 1
fi

# Assign input parameters to variables
cfc=$1
injectionrate=$2
test_id=$3
# Create a new directory for each run
timestamp=$(date +"%H%M%S")
datastamp=$(date +"%Y%m%d")
outdir="./myM5out/${datastamp}_test${test_id}/cfc_${cfc}_inj_${injectionrate}_${timestamp}"
mkdir -p "$outdir"

# "uniform_random",
# "tornado",
# "bit_complement",
# "bit_reverse",
# "bit_rotation",
# "neighbor",
# "shuffle",
# "transpose",
# Run gem5 with the provided parameters
touch "$outdir/out.log"
./build/Garnet_standalone/gem5.opt \
-d "$outdir" \
configs/example/garnet_synth_traffic.py \
            --topology=Chiplets_Mesh_distrbuteL2_MC_se \
            --mesh-rows=4 \
            --num-cpus=64 \
            --num-dirs=64 \
            --num-l2caches=64 \
            --network=garnet \
            --router-latency=1 \
            --vcs-per-vnet=1 \
            --sim-cycles=1000000 \
            --injectionrate="$injectionrate" \
            --synthetic=transpose \
            --routing-algorithm=2 \
            --cfc="$cfc" > "$outdir/out.log" 2>&1

echo "Simulation output is saved in: $outdir/out.log"
