# data analysis: angle problem
# code: 2020 - 10 - 05
# experiment data: 2020 - 09 - 25
# experiment is done on a plane, and tested for the effect of stationary place rotation wrt stationary point

# imported lib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

PI = 3.14159265

# open file
f_coodinate = open('coordinate.txt', 'r')
f_plane = open('plane_.txt', 'r')
f_plane_measure_before = open('plane_measure.txt', 'r')
f_plane_measure_after = open('plane_measure2.txt', 'r')
f_plane_Matrix = open('Correction_Matrix.txt', 'r')
f_plane_Robot = open('robot_corr_0904_1.txt','r')

# read file
d_coodinate = f_coodinate.read()
d_plane = f_plane.read()
d_plane_measure_before = f_plane_measure_before.read()
d_plane_measure_after = f_plane_measure_after.read()
d_plane_Matrix = f_plane_Matrix.read()
d_plane_Robot = f_plane_Robot.read()

# close file
f_coodinate.close()
f_plane.close()
f_plane_measure_before.close()
f_plane_measure_after.close()
f_plane_Matrix.close()
f_plane_Robot.close()

def decode_actual(data):
    data = data.split('\n')
    plane = []
    for i in data:
        if i != '':
            temp = i.split(',')
            temp.pop(0)
            for j in range(len(temp)):
                temp[j] = float(temp[j])
            print(temp)
            plane.append(temp)
    return plane

def decode_whole_plane(data):
    data = data.split('\n')
    plane_whole = []
    l = len(data)
    for i in range(l):
        data_ = data[i]
        temp = data_.split(',')
        if len(temp) > 1:
            print(temp)
            for q in range(len(temp)):
                temp[q] = float(temp[q])
            data_ = temp
            plane_whole.append(data_)
    return plane_whole

def decode_plane(data):
    data = data.split('\n')
    plane_after = []
    plane_before = []
    l = len(data)
    if l % 2 != 0: # even
        l = l -1

    for i in range(l):
        data_ = data[i]
        temp = data_.split(',')
        for q in range(len(temp)):
            temp[q] = float(temp[q])
        data_ = temp
        if i < l/2:
            plane_before.append(data_)
        else:
            plane_after.append(data_)
    return [plane_before, plane_after]

def decode_matrix(data):
    data = data.split('\n')
    m = []
    l = len(data)
    for i in range(l):
        data_ = data[i]
        temp = data_.split(',')
        if len(temp) > 1:
            print(temp)
            temp_arr = []
            for q in range(len(temp)):
                # temp[q] = float(temp[q])
                temp_arr.append(float(temp[q]))
            data_ = temp_arr
            m.append(data_)
    return m

def decode_robot(data):
    # b,21,-0.0251752179325,0.345578340649,-0.079894705574
    data = data.split('\n')
    d = []
    for i in range(len(data)):
        d_temp = data[i]
        d_temp = d_temp.split(',')
        if len(d_temp) > 1:
            d_temp2 = [float(d_temp[2]),float(d_temp[3]),float(d_temp[4])]
            d.append(d_temp2)
    return d

def print_3D(data):
    fig = plt.figure("Plane correction")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X label')
    ax.set_ylabel('Y label')
    ax.set_zlabel('Z label')
    data_xyz=[[],[],[]]
    # print(data[0])
    for j in data[0]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'o',label='before')
    data_xyz=[[],[],[]]
    for j in data[1]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'^',label='after')

    plt.legend()
    plt.grid()
    plt.show()

def print_whole_3D(data, data_plane):
    fig = plt.figure("Plane correction")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X label')
    ax.set_ylabel('Y label')
    ax.set_zlabel('Z label')
    data_xyz=[[],[],[]]
    # print(data[0])
    for j in data[0]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'b', label='before')

    data_xyz=[[],[],[]]
    for j in data[1]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'r', label='after')
    for j in data_plane:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'g',label='data')

    plt.legend()
    plt.grid()
    plt.show()

def Matrix_dot(Matrix,vector):
    ret = [0,0,0]
    for i in range(3):
        sum = 0
        for j in range(3):
            sum += Matrix[i][j]*vector[j]
        sum += Matrix[i][3]
        ret[i] = sum
    return ret

def print_whole_3D_corrected(data, data_plane, Matrix):
    fig = plt.figure("Plane correction")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X label')
    ax.set_ylabel('Y label')
    ax.set_zlabel('Z label')
    data_xyz=[[],[],[]]
    # print(data[0])
    for j in data[0]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'b', label='before')

    data_xyz=[[],[],[]]
    for j in data[1]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'r', label='after')

    data_xyz=[[],[],[]]
    for j in data_plane:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'g',label='data')

    data_xyz=[[],[],[]]
    for j in data_plane:
        # matrix calc
        output = Matrix_dot(Matrix,j)
        data_xyz[0].append(int(output[0]*100))
        data_xyz[1].append(int(output[1]*100))
        data_xyz[2].append(int(output[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'c',label='data_after')

    plt.legend()
    plt.grid()
    plt.show()

def print_whole_3D_corrected_with_robot(data, data_plane, Matrix,data_robot):
    fig = plt.figure("Plane correction with robot path")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X label')
    ax.set_ylabel('Y label')
    ax.set_zlabel('Z label')
    data_xyz=[[],[],[]]
    # print(data[0])
    for j in data[0]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'b', label='before')

    data_xyz=[[],[],[]]
    for j in data[1]:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0], data_xyz[1], data_xyz[2], 'r', label='after')

    data_xyz=[[],[],[]]
    for j in data_plane:
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'g',label='data')

    data_xyz=[[],[],[]]
    for j in data_plane:
        # matrix calc
        output = Matrix_dot(Matrix,j)
        data_xyz[0].append(int(output[0]*100))
        data_xyz[1].append(int(output[1]*100))
        data_xyz[2].append(int(output[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'c',label='data_after')

    data_xyz=[[],[],[]]
    for j in data_robot:
        # matrix calc
        # output = Matrix_dot(Matrix,j)
        data_xyz[0].append(int(j[0]*100))
        data_xyz[1].append(int(j[1]*100))
        data_xyz[2].append(int(j[2]*100))
    ax.scatter(data_xyz[0],data_xyz[1],data_xyz[2],'k',label='robot')

    plt.legend()
    plt.grid()
    plt.show()

def error_3D(measure,actual):
    err_before = [[],[],[],[]]
    err_after = [[],[],[],[]]
    measure_before = measure[0]
    measure_after = measure[1]
    for i in range(len(measure_before)):
        err_s = 0
        for j in range(3):
            err_d = abs(int((measure_before[i][j] - actual[i][j]) *1000))
            err_s += err_d*err_d
            err_before[j].append(err_d)
        err_before[3].append(math.sqrt(err_s))
    for i in range(len(measure_after)):
        err_s = 0
        for j in range(3):
            err_d = abs(int((measure_after[i][j] - actual[i][j])*1000))
            err_s += err_d*err_d
            err_after[j].append(err_d)
        err_after[3].append(math.sqrt(err_s))
    print("error of measurement before correction in mm:")
    for i in range(len(measure_before)):
        print(err_before[0][i],err_before[1][i],err_before[2][i],err_before[3][i])
    print("error of measurement after correction in mm:")
    for i in range(len(measure_after)):
        print(err_after[0][i],err_after[1][i],err_after[2][i],err_after[3][i])

    plt.figure("error for plane")
    plt.subplot(221)
    plt.title('x error')
    plt.plot(err_before[0][:],'bx',label='x_before')
    plt.plot(err_after[0][:],'rx',label='x_after')
    plt.ylabel('error in distance in mm')
    plt.xlabel('distance in mm')
    plt.legend()
    plt.grid()

    plt.subplot(222)
    plt.title('y error')
    plt.plot(err_before[1][:],'bx',label='y_before')
    plt.plot(err_after[1][:],'rx',label='y_after')
    plt.ylabel('error in distance in mm')
    plt.xlabel('distance in mm')
    plt.legend()
    plt.grid()

    plt.subplot(223)
    plt.title('z error')
    plt.plot(err_before[2][:], 'bx', label='z_before')
    plt.plot(err_after[2][:], 'rx', label='z_after')
    plt.ylabel('error in distance in mm')
    plt.xlabel('distance in mm')
    plt.legend()
    plt.grid()

    plt.subplot(224)
    plt.title('distance error')
    plt.plot(err_before[3][:], 'bx', label='z_before')
    plt.plot(err_after[3][:], 'rx', label='z_after')
    plt.ylabel('error in distance in mm')
    plt.xlabel('distance in mm')
    plt.legend()
    plt.grid()

    plt.show()

# print 3D plane
data_3D = decode_plane(d_plane)
print_3D(data_3D)

# calc error for 3D plane
data_actual = decode_actual(d_coodinate)
error_3D(data_3D,data_actual)

# calc error for 3D whole plane
data_plane = decode_whole_plane(d_plane_measure_before)
print_whole_3D(data_3D,data_plane)

# calc error for 3D whole plane
data_plane = decode_whole_plane(d_plane_measure_after)
print_whole_3D(data_3D,data_plane)

data_matrix = decode_matrix(d_plane_Matrix)
print_whole_3D_corrected(data_3D,data_plane,data_matrix)

# calc error for 3D whole plane and robot
data_robot = decode_robot(d_plane_Robot)
print_whole_3D_corrected_with_robot(data_3D,data_plane,data_matrix,data_robot)