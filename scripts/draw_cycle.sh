test_id=$1
traffic_types=("uniform_random" "bit_complement" "bit_reverse" "bit_rotation" "shuffle" "transpose")
timestamp=$(date +%Y%m%d)

for traffic_type in "${traffic_types[@]}"; do
    bash ./scripts/extract_latency_and_draw.sh \
    "$test_id" \
    ./myM5out/${timestamp}_mesh"$test_id"/"$traffic_type"/output.png \
    "$traffic_type" \
    "$timestamp"
done
