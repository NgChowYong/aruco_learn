# data analysis: angle problem
# code: 2020 - 10 - 05
# experiment data: 2020 - 09 - 25
# experiment is done on a plane, and tested for the effect of stationary place rotation wrt stationary point

# imported lib
import matplotlib.pyplot as plt
import math

PI = 3.14159265

# open file
str_1 = "err_"
str_2 = "cm.txt"
str_3 = [20,40,50,60,80,100,120]
data_ = []
for j in range(len(str_3)):
    f = open(str_1+str(str_3[j])+str_2,'r')
    # read file
    l = f.read()
    # data read in following format:
    # x y z , x y z w

    # print(l)
    data_pt = [[],[],[]] # use to store x y z data
    l = l.split('\n')

    for i in l:
        if i is not '':
            data = i.split(',')
            data_pt[0].append(float(data[0])*100)
            data_pt[1].append(float(data[1])*100)
            data_pt[2].append(float(data[2])*100)

    mean_ = [sum(data_pt[0]) / len(data_pt[0]),
             sum(data_pt[1]) / len(data_pt[1]),
             sum(data_pt[2]) / len(data_pt[2])]
    data_.append([str_3[j],mean_])

    f.close()

actual = [0]
measure = [0]
err = [0]
for i in range(len(data_)):
    actual.append(data_[i][0])
    measure.append(data_[i][1][1])
    err.append(math.fabs(data_[i][0]-data_[i][1][1]))

print(err)

plt.figure("Error measurement for distance")
plt.subplot(211)
plt.plot(actual,measure,'rx',label='measure')
plt.plot(actual,actual,label='actual')
plt.ylabel('measure distance in cm')
plt.xlabel('actual distance in cm')
plt.legend()
plt.grid()

plt.subplot(212)
plt.plot(actual,err,'rx',label='error')
plt.ylabel('distance error in cm')
plt.xlabel('actual distance in cm')
plt.legend()
plt.grid()
plt.show()