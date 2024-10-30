import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

input_dir = "/gem5/chiplet-flow-ctrl/myM5out/20240429_test6/sat_throughput.txt"
# 指定的输出目录
output_dir = "/gem5/chiplet-flow-ctrl/myM5out/20240429_test6/"
# 图片文件名
output_filename = "sat_throughput.png"

# 读取数据
data = pd.read_csv(input_dir, sep=r"\s+", header=0, index_col=0)

# 绘制柱状图
fig, ax = plt.subplots(figsize=(10, 5))

# 设置柱状图的宽度
bar_width = 1
spacing = 4  # 每组柱状图之间的间距，大于1

# 计算柱状图的x轴位置
n_groups = len(data.index)
index_pos = np.arange(0, n_groups * spacing, spacing)

print(index_pos)

ax.grid(axis="y", linestyle="--", color="gray", alpha=0.5, zorder=0)

# 绘制每个实验组，并考虑间距
upp_bars = ax.bar(
    index_pos,
    data["upp"],
    bar_width,
    color="aliceblue",
    edgecolor="gray",
    linewidth=2,
    hatch="/",
    label="upp",
    zorder=1,
)
fastpass_bars = ax.bar(
    index_pos + bar_width,
    data["fastpass"],
    bar_width,
    color="wheat",
    edgecolor="gray",
    linewidth=2,
    hatch="o",
    label="fastpass",
    zorder=1,
)
cfc_bars = ax.bar(
    index_pos + 2 * (bar_width),
    data["cfc"],
    bar_width,
    color="indianred",
    edgecolor="black",
    linewidth=2,
    label="cfc",
    zorder=1,
)


# 添加图例
ax.legend(loc="best", fontsize=12)

# 设置x轴标签和刻度
# ax.set_xlabel('Synthetic Traffic Name')
ax.set_xticks(index_pos + bar_width)  # 将刻度放在每个实验组的中间
ax.set_xticklabels(data.index, fontsize=11, rotation=12)  # 设置x轴标签的字体大小和角度

# 设置y轴标签
ax.set_ylabel("Throughput (Gbps)", fontsize=12)
ax.set_ylim(500, 685)

for tick in ax.get_yticklabels():
    tick.set_fontsize(12)  # 设置y轴刻度标签字体大小为12

# 设置标题
ax.set_title(
    "Saturation Throughput of Different Synthetic Traffic", fontsize=14
)

# 保存图片到指定目录
plt.savefig(output_dir + output_filename, dpi=300)

# 显示图形
plt.show()
