test_id=$1
traffic_types=("uniform_random" "bit_complement" "bit_reverse" "bit_rotation" "shuffle" "transpose")
timestamp=20241024

for traffic_type in "${traffic_types[@]}"; do
    python ./python_scripts/plot_latency.py \
    --test_date="$timestamp" \
    --test_id="$test_id" \
    --traffic_type="$traffic_type"
done
