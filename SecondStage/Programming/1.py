n = int(input())

d = dict()
res = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
d_new = dict()
for _ in range(n):
    num, x_ice, y_ice, x_drone, y_drone, height = map(float, input().split())
    try:
        d[int(num)] += [[x_ice, y_ice, x_drone, y_drone, height]]
    except:
        d[int(num)] = [[x_ice, y_ice, x_drone, y_drone, height]]

def size(height):
    x = round((4*height)/3, 2)
    y = round((3*height)/3, 2)
    return x, y

for key, item in zip(d.keys(), d.values()):
    temp_arr = [[], []]
    for i in range(len(item)):
        x, y = size(item[i][-1])
        
        r_x = round(item[i][0] * x / 640, 2)
        r_y = round(item[i][1] * y / 480, 2)
        if item[i][0] > 320:
            real_x = item[i][2] + (r_x-(x//2))
        else:
            real_x = item[i][2] - ((x//2)-r_x)
        if item[i][1] > 240:
            real_y = item[i][3] - ((y//2)-r_y)
        else:
            real_y = item[i][3] + (r_y-(y//2))
        temp_arr[0].append(real_x)
        temp_arr[1].append(real_y)

    d_new[key] = [round(sum(temp_arr[0])/len(temp_arr[0])), round(sum(temp_arr[1])/len(temp_arr[1]))]        
        
x_0 = [[],[]]
x_1 = [[],[]]
x_2 = [[],[]]
max_x = -11111
min_x = 1000000
max_y = -11111
min_y = 1000000
for key, item in zip(d_new.keys(), d_new.values()):
    if item[0] > max_x:
        max_x = item[0]
    if item[1] > max_y:
        max_y = item[1]
    if item[0] < min_x:
        min_x = item[0]
    if item[1] < min_y:
        min_y = item[1]

thr_x = (max_x + min_x + 2)/3
#thr_y = (max_y + max_y + 2)/4

for key, item in zip(d_new.keys(), d_new.values()):
    if item[0] < thr_x:
        x_0[0].append(key)
        x_0[1].append(item[1])
    if item[0] < thr_x*2 and item[0] > thr_x:
        x_1[0].append(key)
        x_1[1].append(item[1])
    if item[0] < thr_x*3 and item[0] > thr_x*2:
        x_2[0].append(key)
        x_2[1].append(item[1])

res[0][0] = x_0[0][x_0[1].index(max(x_0[1]))]
res[3][0] = x_0[0][x_0[1].index(min(x_0[1]))]
res[1][0] = x_0[0][x_0[1].index(max(x_0[1][1:3]))]
res[2][0] = x_0[0][x_0[1].index(min(x_0[1][1:3]))]

res[0][1] = x_1[0][x_1[1].index(max(x_1[1]))]
res[3][1] = x_1[0][x_1[1].index(min(x_1[1]))]
res[1][1] = x_1[0][x_1[1].index(max(x_1[1][1:3]))]
res[2][1] = x_1[0][x_1[1].index(min(x_1[1][1:3]))]

res[0][2] = x_2[0][x_2[1].index(max(x_2[1]))]
res[3][2] = x_2[0][x_2[1].index(min(x_2[1]))]
res[1][2] = x_2[0][x_2[1].index(max(x_2[1][1:3]))]
res[2][2] = x_2[0][x_2[1].index(min(x_2[1][1:3]))]


for i in res:
    print(*i)
