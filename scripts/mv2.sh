# 复制最后8行到cfc2.txt
tail -n 7 cfc1.txt > cfc2.txt

# 计算总行数并删除cfc1.txt的最后8行
lines=$(wc -l < cfc1.txt)
head -n $((lines - 7)) cfc1.txt > cfc1.tmp && mv cfc1.tmp cfc1.txt
