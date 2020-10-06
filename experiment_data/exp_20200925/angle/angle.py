# data analysis: angle problem
# code: 2020 - 10 - 05
# experiment data: 2020 - 09 - 25
# experiment is done on a plane, and tested for the effect of stationary place rotation wrt stationary point

# imported lib
import matplotlib.pyplot as plt
import math

PI = 3.14159265

def quat_to_euler(x,y,z,w):
    # roll calculation
    s = 2*(w*x + y*z)
    c = 1 - 2*(x*x + y*y)
    roll = math.atan2(s,c)

    # pitch calculation
    s = 2*(w*y - z*x)
    # if abs(s) >= 1:
    #     pitch = math.copysign(3.1415/2,s)
    # else:
    #     pitch = math.asin(s)
    pitch = math.asin(s)

    # yaw calculation
    s = 2*(w*z + x*y)
    c = 1 - 2*(y*y + z*z)
    yaw = math.atan2(s,c)

    return [roll,pitch,yaw]


def rad2deg(rad):
    deg = rad
    for i in range(len(rad)):
        deg[i] = rad[i]*180/PI
    return deg


# open file
str_1 = "angle_"
str_2 = "cm.txt"
str_3 = [str(20),str(50),str(100)]
for i in range(len(str_3)):
    f = open(str_1+str_3[i]+str_2,'r')
    # read file
    l = f.read()
    # data read in following format:
    # x y z , x y z w

    # print(l)
    data_pt = [[],[],[],[],[],[]] # use to store x y z data
    distance = []
    l = l.split('\n')

    for i in l:
        if i is not '':
            data = i.split(',')
            data_pt[0].append(float(data[0])*100)
            data_pt[1].append(float(data[1])*100)
            data_pt[2].append(float(data[2])*100)
            distance.append(math.sqrt(float(data[0])*float(data[0]) + float(data[1])*float(data[1]) + float(data[2])*float(data[2]))*100)
            angle = quat_to_euler(float(data[3]),float(data[4]),float(data[5]),float(data[6]))
            angle = rad2deg(angle)
            data_pt[3].append(angle[0])
            data_pt[4].append(angle[1])
            data_pt[5].append(angle[2])
            #if len(data_pt[0]) > 175:
            #    break

    # plt.plot(data_pt[1],'bx')
    plt.subplot(211)
    #plt.plot(data_pt[3],'bx',label='roll')
    #plt.plot(data_pt[4],'gx',label='pitch')
    plt.plot(data_pt[5],'bx',label='yaw')
    plt.ylabel('angle in degree')
    plt.xlabel('data no')
    plt.legend()
    plt.grid()

    #plt.figure()
    plt.subplot(212)
    plt.plot(distance,'ro',label='distance')
    plt.ylabel('distance in cm')
    plt.xlabel('data no')
    plt.legend()
    plt.grid()
    plt.show()

    f.close()

