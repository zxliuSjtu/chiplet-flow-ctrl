# traffic_types=("uniform_random" "tornado" "bit_complement" "bit_reverse" "bit_rotation" "neighbor" "shuffle" "transpose")
test_id=6
traffic_type=uniform_random
timestamp=$(date +%Y%m%d)

python ./python_scripts/plot_latency.py \
    --test_date="$timestamp" \
    --test_id="$test_id" \
    --traffic_type="$traffic_type"
