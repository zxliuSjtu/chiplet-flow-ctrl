#!/bin/bash

# Check if the correct number of arguments are provided
if [ "$#" -ne 8 ]; then
    echo "Usage: $0 <cfc_value> <fastpass_value> <upp_value> <injection_rate> <test_id> <traffic_type> <slot_length> <routing_algorithm>"
    exit 1
fi

# Assign input parameters to variables
cfc=$1
fastpass=$2
upp=$3
injectionrate=$4
test_id=$5
traffic_type=$6
slot_length=$7
routing_algorithm=$8
# Create a new directory for each run
timestamp=$(date +"%H%M%S")
datastamp=$(date +"%Y%m%d")
outdir="./myM5out/${datastamp}_mesh${test_id}/${traffic_type}/${slot_length}_${cfc}${fastpass}${upp}_inj_${injectionrate}_${timestamp}"
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
            --topology=Mesh_XY \
            --mesh-rows=4 \
            --num-cpus=16 \
            --num-dirs=16 \
            --network=garnet \
            --router-latency=3 \
            --vcs-per-vnet=2 \
            --inj-vnet=0 \
            --sim-cycles=1000000 \
            --injectionrate="$injectionrate" \
            --synthetic="$traffic_type" \
            --slotLength="$slot_length" \
            --routing-algorithm="$routing_algorithm" \
            --fastpass="$fastpass" \
            --upp="$upp" \
            --cfc="$cfc" > "$outdir/out.log" 2>&1

echo "Simulation output is saved in: $outdir/out.log"

#--vcs-per-vnet=2 \ 2 vcs each vnet
#--inj-vnet=0 \ means use 1 vnet
#             --link-width-bits=128 \
#            --num-l2caches=64 \
