#!/bin/bash

test_id=$1
# Run gem5 for injection rates ranging from 0.05 to 1 in steps of 0.05
# traffic_types=("uniform_random" "bit_complement" "bit_reverse" "bit_rotation" "shuffle" "transpose")
traffic_types=("bit_rotation" "shuffle" "transpose")
# traffic_types=("shuffle")
# traffic_types=("tornado" "neighbor")
for traffic_type in "${traffic_types[@]}"; do

    for injectionrate in $(seq 0.01 0.01 0.19); do
        # Run gem5 for cfc
        bash ./scripts/run_garnetAloneMesh.sh 1 0 0 "$injectionrate" "$test_id" "$traffic_type" 5 1
        # Run gem5 for fastpass
        bash ./scripts/run_garnetAloneMesh.sh 0 1 0 "$injectionrate" "$test_id" "$traffic_type" 100 1
        # Run gem5 for nothing
        bash ./scripts/run_garnetAloneMesh.sh 0 0 0 "$injectionrate" "$test_id" "$traffic_type" 20 1
    done

done
