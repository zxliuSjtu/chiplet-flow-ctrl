#!/bin/bash
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <text_id> <output_file> <traffic_type> <date_today>"
    exit 1
fi
# Function to extract data from stats.txt
extract_data() {
    local outdir=$1
    local data=$(grep -oP 'system\.ruby\.network\.average_flit_latency\s*\K\d+' "$outdir/stats.txt") # Replace 'your_pattern_here' with the pattern you're searching for
    echo "$data"
}
test_id=$1
output_file=$2
traffic_type=$3
date=$4

# Cleanup last temporary files
# rm ./myM5out/${date}_test$test_id/$traffic_type/*.txt

# Main script
# Loop through each directory in m5out
for outdir in ./myM5out/"$date"_test"$test_id"/"$traffic_type"/*; do
    if [ -d "$outdir" ]; then
        injectionrate=$(echo "$outdir" | grep -oP '(?<=inj_)\d+\.*\d*')
        cfc=$(echo "$outdir" | grep -oP '(?<=cfc_)\d')
        data=$(extract_data "$outdir")

        # Store data in a temporary file for plotting
        echo "$injectionrate $data" >> \
        ./myM5out/"$date"_test"$test_id"/"$traffic_type"/temp_data_cfc$cfc.dat
    fi
done

# Plotting using gnuplot
gnuplot -persist <<-EOF
    set xlabel "Injection Rate"
    set ylabel "Average Flit Latency"
    set title "$traffic_type"
    set terminal png
    set output "$output_file"
    plot "./myM5out/${date}_test$test_id/$traffic_type/temp_data_cfc0.dat" using 1:2 w linespoints lt 2 title "UPP", \
         "./myM5out/${date}_test$test_id/$traffic_type/temp_data_cfc1.dat" using 1:2 w linespoints lt 3 title "CFC"
EOF


# Cleanup temporary files and rename with timestamp

for temp_file in ./myM5out/20240429_test$test_id/$traffic_type/temp_data_cfc*.dat; do
    if [ -f "$temp_file" ]; then
        cfc_num=$(basename "$temp_file" | grep -oP 'cfc\K\d')
        new_file="./myM5out/${date}_test$test_id/$traffic_type/cfc${cfc_num}.txt"
        mv "$temp_file" "$new_file"
    fi
done

python ./python_scripts/plot_latency.py \
    --test_date="$date" \
    --test_id="$test_id" \
    --traffic_type="$traffic_type"
