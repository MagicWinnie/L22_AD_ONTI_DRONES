n = int(input())

d = dict()

for _ in range(n):
    num, x_ice, y_ice, x_drone, y_drone, height = map(float, input().split())
    try:
        d[num] += [[x_ice, y_ice, x_drone, y_drone, height]]
    except:
        d[num] = [[x_ice, y_ice, x_drone, y_drone, height]]

print(d)
