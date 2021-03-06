import numpy as np

n = int(input())

d = []
res = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
d_new = dict()

for _ in range(n):
    num, x_ice, y_ice, x_drone, y_drone, height = map(float, input().split())
    d.append([num, x_ice, y_ice, x_drone, y_drone, height])
def size(height, x_in_pixels, y_in_pixels):
    x = (0.004/640*(x_in_pixels-320)*(height-4))/0.003
    y = (0.003/480*(240-y_in_pixels)*(height-4))/0.003
    return x, y
coords = []
for i in range(len(d)):
    temp_arr = [[], []]
    x, y = size(d[i][-1], d[i][1], d[i][2])
    if d_new.get(int(d[i][0]), False):
        d_new[int(d[i][0])] += [[x+d[i][3], y+d[i][4]]]
    else:
        d_new[int(d[i][0])] = [[x+d[i][3], y+d[i][4]]]

x_x = [[[],[]],[[],[]],[[],[]]]

sorted_ = sorted(d_new.items(), key=lambda kv: kv[1])
for i in range(len(sorted_)):
    sorted_[i] = list(sorted_[i])
    s = [0, 0]
    for j in range(len(sorted_[i][1])):
        s[0] += sorted_[i][1][j][0]
        s[1] += sorted_[i][1][j][1]
    sorted_[i][1] = [s[0]/len(sorted_[i][1]), s[1]/len(sorted_[i][1])]
counter = 0
count = 0
for i in range(len(sorted_)):
    if count < 4:
        x_x[counter][0].append(sorted_[i][0])
        x_x[counter][1].append(sorted_[i][1][1])
    count += 1
    if count > 3:
        count = 0
        counter += 1

for i in range(len(x_x)):
    x_x[i][0] = [x for _, x in sorted(zip(x_x[i][1],x_x[i][0]), key=lambda pair: pair[0])]

for i in range(3):
    for j in range(4):
        res[j][i] = x_x[i][0][3-j]
for i in res:
    print(*i)
