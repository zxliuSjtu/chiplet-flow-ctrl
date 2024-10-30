#!/bin/bash

# Function to extract data from stats.txt
extract_data() {
    local outdir=$1
    local data=$(grep -oP 'system\.ruby\.network\.average_flit_latency\s*\K\d+' "$outdir/stats.txt") # Replace 'your_pattern_here' with the pattern you're searching for
    echo "$data"
}

output_file=$2
# Main script
# Loop through each directory in m5out
for outdir in ./myM5out/20240425_test2/*; do
    if [ -d "$outdir" ]; then
        injectionrate=$(echo "$outdir" | grep -oP '(?<=inj_)\d+\.*\d*')
        cfc=$(echo "$outdir" | grep -oP '(?<=cfc_)\d')
        data=$(extract_data "$outdir")

        # Store data in a temporary file for plotting
        echo "$injectionrate $data" >> temp_data_cfc$cfc.dat
    fi
done

# Plotting using gnuplot
gnuplot -persist <<-EOF
    set xlabel "Injection Rate"
    set ylabel "Average Flit Latency"
    set title "bit complement"
    set terminal png
    set output "$output_file"
    plot "temp_data_cfc1.dat" w linespoint lt 2, \
         "temp_data_cfc0.dat" w linespoint lt 3
EOF

# Cleanup temporary files
 rm temp_data_cfc*.dat
