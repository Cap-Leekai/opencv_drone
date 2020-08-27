#coding=utf8

List = [[0, 1, 2, 3, 4],
        [12, 32, 23, 4, 5],
        [6, 2, 3, 2, 1]]

for i in range(len(List[0])):
    for j in range(len(List)):
        print "Столбец -> ", i, "Строка -> ", j
