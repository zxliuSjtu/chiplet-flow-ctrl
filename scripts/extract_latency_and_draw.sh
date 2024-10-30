#!/bin/bash
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <test_id> <output_file> <traffic_type> <date_today>"
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
for outdir in ./myM5out/"$date"_mesh"$test_id"/"$traffic_type"/*; do
    if [ -d "$outdir" ]; then
        injectionrate=$(echo "$outdir" | grep -oP '(?<=inj_)\d+\.\d*')
        slot_length=$(echo "$outdir" | grep -oP '\d+(?=_\d+_inj_)')
        raw_method=$(echo "$outdir" | grep -oP '\d+(?=_inj_)')

        echo "Extracted values from $outdir:"
        echo "  injectionrate: $injectionrate"
        echo "  slot_length: $slot_length"
        echo "  raw_method: $raw_method"

        if [ "$raw_method" -eq 100 ]; then
            method="BuffeRS"
        elif [ "$raw_method" -eq 010 ]; then
            method="FastPass"
        else
            method="noFlowControl"
        fi

        data=$(extract_data "$outdir")

        # Store data in a temporary file for plotting
        echo "$slot_length $method $injectionrate $data" >> \
        ./myM5out/"$date"_mesh"$test_id"/"$traffic_type"/temp_data_${method}.dat
    fi
done

# # Plotting using gnuplot
# gnuplot -persist <<-EOF
#     set xlabel "Injection Rate"
#     set ylabel "Average Flit Latency"
#     set title "$traffic_type"
#     set terminal png
#     set output "$output_file"
#     plot "./myM5out/${date}_mesh$test_id/$traffic_type/temp_data_*.dat" using 1:2 w linespoints lt 2 title "UPP", \
#          "./myM5out/${date}_mesh$test_id/$traffic_type/temp_data_*.dat" using 1:2 w linespoints lt 3 title "CFC"
# EOF


# Cleanup temporary files and rename with timestamp

# 搜索匹配模式的文件
shopt -s nullglob  # 确保未找到文件时模式不会保持不变
files=(./myM5out/"${date}_mesh${test_id}/${traffic_type}/temp_data_*.dat")

if [ ${#files[@]} -eq 0 ]; then
    echo "Warning: No files found matching pattern ./myM5out/${date}_mesh${test_id}/${traffic_type}/temp_data_*.dat"
    exit 0
fi

for temp_file in ./myM5out/"$date"_mesh"$test_id"/"$traffic_type"/temp_data_*.dat; do
    if [ -f "$temp_file" ]; then
        method=$(basename "$temp_file" | grep -oP '(?<=temp_data_).*(?=\.dat)')
        new_file="./myM5out/${date}_mesh${test_id}/${traffic_type}/${method}.txt"
        awk '{print $(NF-1), $NF}' "$temp_file" > "$new_file"
    fi
done

python ./python_scripts/plot_latency.py \
    --test_date="$date" \
    --test_id="$test_id" \
    --traffic_type="$traffic_type"
