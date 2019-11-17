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



#---------------------------------------------------  
#print(d[8])    
#if d[8]!=[[184.0, 318.0, 6.045, 77.506, 19.921], [14.0, 30.0, 10.021, 72.915, 19.778], [289.0, 205.0, 0.408, 80.725, 21.237], [567.0, 208.0, -4.549, 74.998, 11.178]]:
    #raise Exception(d_new)
x_0 = [[],[]]
x_1 = [[],[]]
x_2 = [[],[]]
x_x = [[[],[]],[[],[]],[[],[]]]
sorted_ = sorted(d_new.items(), key=lambda kv: kv[1])
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

#print(x_x)
for i in range(len(x_x)):
    x_x[i][0] = [x for _, x in sorted(zip(x_x[i][1],x_x[i][0]), key=lambda pair: pair[0])]
#print(x_x)
res[0][0] = x_x[0][0][3]
res[1][0] = x_x[0][0][2]
res[2][0] = x_x[0][0][1]
res[3][0] = x_x[0][0][0]

res[0][1] = x_x[1][0][3]
res[1][1] = x_x[1][0][2]
res[2][1] = x_x[1][0][1]
res[3][1] = x_x[1][0][0]

res[0][2] = x_x[2][0][3]
res[1][2] = x_x[2][0][2]
res[2][2] = x_x[2][0][1]
res[3][2] = x_x[2][0][0]

#---------------------------------------------------
for i in res:
    print(*i)
