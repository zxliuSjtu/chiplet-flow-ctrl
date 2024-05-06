import argparse
import datetime
import os

import matplotlib.pyplot as plt
from matplotlib.ticker import (
    AutoMinorLocator,
    MultipleLocator,
)

# # Directory paths
# read_dir = "/gem5/chiplet-flow-ctrl/myM5out/test_data/"
# current_datetime = datetime.datetime.now()
# save_dir = os.path.join("/gem5/chiplet-flow-ctrl/myM5out/", current_datetime.strftime("%d-%m-%Y_%H-%M-%S"))

# 创建一个解析器对象
parser = argparse.ArgumentParser(description="Plot data with gnuplot")

# 添加命令行参数
parser.add_argument(
    "--test_date", type=str, required=True, help="The test date"
)
parser.add_argument("--test_id", type=str, required=True, help="The test ID")
parser.add_argument(
    "--traffic_type", type=str, required=True, help="The traffic type"
)

# 解析命令行参数
args = parser.parse_args()

test_date = args.test_date
test_id = args.test_id
traffic_type = args.traffic_type

# 读取文件夹路径
read_dir_template = "./myM5out/{test_date}_test{test_id}/{traffic_type}/"
read_dir = read_dir_template.format(
    test_date=test_date, test_id=test_id, traffic_type=traffic_type
)

# 创建输出文件夹路径
save_dir_template = "./pythonOut/{test_date}_test{test_id}/{traffic_type}/"
save_dir = save_dir_template.format(
    test_date=test_date, test_id=test_id, traffic_type=traffic_type
)
# Create the directory if it doesn't exist
os.makedirs(save_dir, exist_ok=True)

# File paths
read_files = [
    os.path.join(read_dir, "cfc0.txt"),
    os.path.join(read_dir, "cfc1.txt"),
    os.path.join(read_dir, "cfc2.txt"),
]

# Initialize lists to store injection rates and latency values for each file
injection_rates = [[] for _ in range(len(read_files))]
latency_values = [[] for _ in range(len(read_files))]

# Read data from each file
for i, file_path in enumerate(read_files):
    with open(file_path) as file:
        data = file.readlines()

    # Extracting injection rates and latency values
    for line in data:
        parts = line.split()
        parts[1] = float(parts[1]) / 500
        injection_rates[i].append(float(parts[0]))
        latency_values[i].append(float(parts[1]))

# Plotting the data
plt.figure(figsize=(10, 5))  # Adjust figure size if needed
for i in range(len(read_files)):
    marker_styles = ["o", "s", "^", "x"]  # Define marker styles
    lines_tyle = ["-", "--", "-.", ":"]
    labels = ["UPP", "CFC", "FastPass", "MTR"]
    plt.plot(
        injection_rates[i],
        latency_values[i],
        marker=marker_styles[i],
        markersize=6,
        linewidth=2,
        linestyle=lines_tyle[i],
        label=labels[i],
    )

# Adding labels and title
plt.xlabel("Injection Rate (flits/node/cycle)", fontsize=14)
plt.ylabel("Average Packet Latency (Cycles)", fontsize=14)
plt.title(traffic_type, fontsize=14)

# Display the legend
plt.legend(fontsize=14)
plt.xticks(fontsize=12)
plt.yticks(fontsize=12)
# Display the plot
# plt.gca().xaxis.set_major_locator(MultipleLocator(0.01))  # Adjust the step value as needed
# plt.gca().yaxis.set_major_locator(MultipleLocator(50000))  # Adjust the step value as needed
# plt.gca().xaxis.set_minor_locator(AutoMinorLocator())
# plt.gca().yaxis.set_minor_locator(AutoMinorLocator())
plt.grid(True, which="both", linestyle="--", linewidth=0.5)

# Save the plot as a file
save_file = os.path.join(save_dir, "line_chart.png")
plt.savefig(save_file)

# Show the plot
plt.show()