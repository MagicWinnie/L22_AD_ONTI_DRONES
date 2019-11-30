import numpy as np
n = int(input())

d = []
res = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
d_new = dict()

for _ in range(n):
    num, x_ice, y_ice, x_drone, y_drone, height = map(float, input().split())
    d.append([num, x_ice, y_ice, x_drone, y_drone, height])


def size(height, x_in_pixels, y_in_pixels):
    how_much_distance_in_one_pixel = 0.004/640
    x = (how_much_distance_in_one_pixel*(x_in_pixels-320)*(height-4))/0.003
    y = (how_much_distance_in_one_pixel*(240-y_in_pixels)*(height-4))/0.003
    return x, y

coords = []
for i in range(len(d)):
    temp_arr = [[], []]
    x, y = size(d[i][-1], d[i][1], d[i][2])
    # try:
    #     d_new[i] += [temp_arr[0], temp_arr[1]]#[np.median(temp_arr[0]), np.median(temp_arr[1])]
    # except:
    #     d_new[] = [temp_arr[0], temp_arr[1]]
    # coords.append([x+d[i][3], y+d[i][4]])
    try:
        d_new[int(d[i][0])] += [x+d[i][3], y+d[i][4]]#[np.median(temp_arr[0]), np.median(temp_arr[1])]
    except:
        d_new[int(d[i][0])] = [x+d[i][3], y+d[i][4]]
#print(d_new)

x_x = [[[],[]],[[],[]],[[],[]]]

sorted_ = sorted(d_new.items(), key=lambda kv: kv[1])

counter = 0
count = 0
print(sorted_)
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
