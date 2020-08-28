from localization.srv import correction_service,correction_serviceResponse
import rospy
import numpy as np

def handle_correction_service(req):
    n = 4
    m = req.size
    Actual_Data = np.zeros([m, 3])
    Measure_Data = np.zeros([m, 3])
    print("Collect Data... ")
    for i in range(m):
        temp = []
        for j in range(3):
            temp.append(req.Actual[i*3+j])        
        Actual_Data.append(temp)
    
    for i in range(m):
        temp = []
        for j in range(3):
            temp.append(req.Measure[i*3+j])        
        Measure_Data.append(temp)

    Actual_Data = np.array(Actual_Data)
    Measure_Data = np.array(Measure_Data)
    print("Collect Data... Done")
    print("\nfind matrix from measure to actual\n")
    print("Using transformation method to find matrix... ")
    # b = A x
    # x = (AT * A)^-1 * AT * b

    # x is n^2 x 1 size
    # A is m*3 x n*n
    # b is m*3 x 1 size
    n = 4
    m = Measure_Data.shape
    m = m[0]

    # A is input =
    # [ x y z 0 0 0 0 0 0
    #   0 0 0 x y z 0 0 0
    #   0 0 0 0 0 0 x y z ]
    A = Measure_Data
    Atemp = np.zeros([m * 3, n * 3])
    for i in range(m):
        for j in range(3):
            for k in range(n * 3):
                if k >= j * n and k < j * n + 3:
                    Atemp[j + i * 3][k] = A[i][k % n]
                elif k == j*n + 3:
                    Atemp[j + i * 3][k] = 1
    # print(A)
    A = Atemp
    # print(A)

    # b is output =
    # [ x
    #   y
    #   z ]
    b = Actual_Data
    b.resize([m*3, 1])

    # x is
    # [ x
    #   y
    #   z ]
    AT = A.transpose()
    temp = AT.dot(A)
    ATAinv = np.linalg.inv(temp)
    x = ATAinv.dot(AT).dot(b)
    ##M = np.zeros([3, 4])
    ##for i in range(3):
    ##    for j in range(4):
    ##        M[i][j] = x[i*4+j]
    ##print(M)
    # need to transform to R|T matrix
    print("Using transformation method to find matrix... done")
    ### testing for both case
    ##print("should get actual point,\n")
    ##print(dest)
    ##point1 = Measure_Data.transpose()
    ##print('using cv2 method : \n',M.dot(point1))
    ##point1 = np.vstack((point1, np.ones((1, m))))
    ##print('using my method : \n',M2.dot(point1))

    ret = AddTwoIntsResponse()
    ret.Matrix = x
    
    return ret

if __name__ == "__main__":
    rospy.init_node('correction_service')
    s = rospy.Service('correction_service', correction_service, handle_correction_service)
    print("Ready to do correction")
    rospy.spin()
