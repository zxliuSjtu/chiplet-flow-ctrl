#!/bin/bash

test_id=$1
# Run gem5 for injection rates ranging from 0.05 to 1 in steps of 0.05
for injectionrate in $(seq 0.02 0.01 0.17); do
    # Run gem5 for cfc=0
    bash ./scripts/run_garnetAlone.sh 0 "$injectionrate" "$test_id"

    # Run gem5 for cfc=1
    bash ./scripts/run_garnetAlone.sh 1 "$injectionrate" "$test_id"
done
