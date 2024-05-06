#!/bin/bash

test_id=$1
# Run gem5 for injection rates ranging from 0.05 to 1 in steps of 0.05
traffic_types=("uniform_random" "bit_complement" "bit_reverse" "bit_rotation" "shuffle" "transpose")
# traffic_types=("tornado" "neighbor")
for traffic_type in "${traffic_types[@]}"; do

    for injectionrate in $(seq 0.02 0.02 0.16); do
        # Run gem5 for cfc=0
        bash ./scripts/run_garnetAlone.sh 0 "$injectionrate" "$test_id" "$traffic_type"

        # Run gem5 for cfc=1
        bash ./scripts/run_garnetAlone.sh 1 "$injectionrate" "$test_id" "$traffic_type"
    done

done
