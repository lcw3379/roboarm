import math
import numpy as np
import cv2
import serial
import time

#Link_1 = 40
Link_2 = 120
Link_3 = 150
pi = 3.141592



def xyz_to_angle(xyz) :    #(X,Y,Z) 값을 리스트로 받는다
    xyz[2] = xyz[2] - 19
    t = math.sqrt(xyz[0]**2 + xyz[1]**2) - 12   #베이스축에서 링크1축까지의 t거리 1cm 추가됨
    
    t = t-30
    xyz[2] = xyz[2] + 35

    CosTheta2 = (t**2 + xyz[2]**2 - Link_2**2 - Link_3**2)/(2*Link_2*Link_3)
    if((CosTheta2<-1)or(CosTheta2>1)):
        print("Unable to convert angle!", xyz)
        return 0
    SinTheta2 = -1*math.sqrt(1-(CosTheta2)**2)
    Theta2 = math.degrees(math.atan2(SinTheta2, CosTheta2))
    
    k1 = Link_2 + Link_3*CosTheta2
    k2 = Link_3*SinTheta2
    
    Theta1 =math.degrees(math.atan2(xyz[2],t) - math.atan2(k2,k1))
    angle_base = math.degrees(math.atan2(xyz[1],xyz[0]))
    angle_1 = Theta1 + 90
    angle_2 = abs(Theta2) - Theta1 + 90
    
    
    #실제 긴 링크의 앵글값(angle_1)은 90을 빼야 지표면에서 링크까지의 각도가 나옴
    if angle_base < 0:
        angle_base = angle_base + 360
    
    #모터 각도 범위내에 존재해야 실행 되게 만들기.
    if((angle_base>=0)and(angle_base<=350)):
        if((angle_1>0 + 90)and(angle_1< 160 + 90)):
            if((angle_2>-12 + 90)and(angle_2<90 + 90)):
                #print("필요각도 : ",angle_base,angle_1-90,angle_2)
                
                angle_base = angle_base*4095/360
                angle_1 = angle_1*4095/360
                angle_2 = angle_2*4095/360
                a = [angle_base, angle_1, angle_2]
                
                a_r = list(map(int, a))
                return a_r
    
    #print(angle_base,angle_1,angle_2)
    print("The angle is over the range : .", xyz )
    print("angle : .", angle_base,angle_1,angle_2)
    return 0


def pos_to_angle(pos) :
    a = [pos[0]*360/4095,pos[1]*360/4095, pos[2]*360/4095]
    #a_r = list(map(int, a))
    return a

def RotZ(theta):
    theta = theta * pi / 180
    a = [[math.cos(theta),-math.sin(theta),0,0],[math.sin(theta),math.cos(theta),0,0],[0,0,1,0],[0,0,0,1]]
    return a
    
def RotX(theta):
    theta = theta * pi / 180
    a = [[1,0,0,0], [0,math.cos(theta),-math.sin(theta),0], [0,math.sin(theta),math.cos(theta),0], [0,0,0,1]]
    return a
    
def Trans(x, y, z):
    a = [[1,0,0,x],[0 ,1,0,y], [0,0,1,z], [0,0,0,1]]
    return a


def prexyz(pos):
    anglelist = pos_to_angle(pos)
    #print("모터각도 현재 포지션 : ",anglelist)
    anglelist[0] = anglelist[0] - 90  #매트랩에서도 이렇게 했다. 실제 1관절값이랑 90도 차이나지 않는지 주의할것.
    anglelist[1] = anglelist[1] - 90
    anglelist[2] = anglelist[2] - 90
    angle3 = anglelist[2]
    anglelist[2] = -(anglelist[2] + anglelist[1])
    #print("변환된 각도 리스트 : ",anglelist)
    T_0_0 = np.dot(RotZ(anglelist[0]),Trans(0, 12, 19))
    T_0_1 = np.dot(RotX(anglelist[1]),Trans(0, Link_2, 0))
    T_1_2 = np.dot(RotX(anglelist[2]),Trans(0, Link_3, 0))
    T_2_G = np.dot(RotX(angle3),Trans(0, 30, -35))
    T_total = np.dot(T_0_0,np.dot(T_0_1,np.dot(T_1_2,T_2_G)))

    # print("T : ",T_0_0)
    #print("T total : ",T_total)
    a=np.array(T_total)

    #print("3x3 : ",a[0:3,0:3])
    
    # ASD = T_total[0:3, 0:3]
    # ASR = T_total[0:3, 3:4] 
    # print("T의 R행렬 : ", ASD)
    # print("T의 T행렬 : ", ASR)
    
    
    y0 = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    y0_0 = np.dot(T_0_0,y0)
    y0_1 = np.dot(T_0_0,np.dot(T_0_1 ,y0))
    y0_2 = np.dot(T_total,y0)

    originX = [y0[0][3],y0_0[0][3] ,y0_1[0][3], y0_2[0][3]]
    originY = [y0[1][3],y0_0[1][3] ,y0_1[1][3], y0_2[1][3]]
    originZ = [y0[2][3],y0_0[2][3] ,y0_1[2][3], y0_2[2][3]]
    # print("origin : ",originX, originY, originZ)
    xyz = np.array([[originX[3]],[originY[3]],[originZ[3]]])
    R_total = np.array(T_total[0:3,0:3])
    
    #print(R_total)
#     existing_data_list1 = np.array(np.load('Robot_rotation_vectors.npy'))
#     existing_data_list2 = np.array(np.load('Robot_translation_vectors.npy'))
   
#     print("R : ",R_total)
#     print("T : ",xyz)
# # Append the new data to the existing list
#     result1 = np.block(([[[existing_data_list1]],[[R_total]]]))
#     result2 = np.block(([[[existing_data_list2]],[[xyz]]]))
    
#     print("resul1 : ", result1)
    
#     updated_data1 = np.array(result1)
#     updated_data2 = np.array(result2)

#     np.save("Robot_rotation_vectors.npy", updated_data1)
#     np.save("Robot_translation_vectors.npy", updated_data2)
    return (originX[3],originY[3],originZ[3])



def camera_xyz(pos):
    anglelist = pos_to_angle(pos)
    #print("모터각도 현재 포지션 : ",anglelist)
    anglelist[0] = anglelist[0] - 90  #매트랩에서도 이렇게 했다. 실제 1관절값이랑 90도 차이나지 않는지 주의할것.
    anglelist[1] = anglelist[1] - 90
    anglelist[2] = anglelist[2] - 90
    angle3 = anglelist[2]
    anglelist[2] = -(anglelist[2] + anglelist[1])

    #print("변환된 각도 리스트 : ",anglelist)
    T_0_0 = np.dot(RotZ(anglelist[0]),Trans(0, 12, 19))
    T_0_1 = np.dot(RotX(anglelist[1]),Trans(0, Link_2, 0))
    T_1_2 = np.dot(RotX(anglelist[2]),Trans(0, Link_3, 0))
    #T_2_G = np.dot(RotX(angle3),Trans(0, 30, -35))
    T_2_G = np.dot(RotX(angle3),Trans(0, 67, 0))
    T_total = np.dot(T_0_0,np.dot(T_0_1,np.dot(T_1_2,T_2_G)))
    # T_G_C = np.dot([[ 0.93588985, -0.34634197,  0.06447814,0],
    #                 [-0.32773048, -0.92307506 ,-0.20130865,0],
    #                 [ 0.1292398 ,  0.16727127 ,-0.97740339,0],
    #                 [ 0,0 ,0 ,1]], Trans(-6.30740413e-02,1.24620323e+01,1.09266330e-15))
    
    # T_total = np.dot(T_total,T_G_C)

    np.dot(T_0_0,np.dot(T_0_1,T_1_2))
    
    
    y0 = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    y0_0 = np.dot(T_0_0,y0)
    y0_1 = np.dot(T_0_0,np.dot(T_0_1 ,y0))
    y0_2 = np.dot(np.dot(T_0_0,np.dot(T_0_1,T_1_2)),y0)
    y0_3 = np.dot(T_total,y0)

    originX = [y0[0][3],y0_0[0][3] ,y0_1[0][3], y0_2[0][3],y0_3[0][3]]
    originY = [y0[1][3],y0_0[1][3] ,y0_1[1][3], y0_2[1][3],y0_3[1][3]]
    originZ = [y0[2][3],y0_0[2][3] ,y0_1[2][3], y0_2[2][3],y0_3[2][3]]
    print("camera pos : ",originX[4], originY[4], originZ[4])
    return (originX[4],originY[4],originZ[4])
    


def first_xyz(pos):
    anglelist = pos_to_angle(pos)
    #print("모터각도 현재 포지션 : ",anglelist)
    anglelist[0] = anglelist[0] - 90  #매트랩에서도 이렇게 했다. 실제 1관절값이랑 90도 차이나지 않는지 주의할것.
    anglelist[1] = anglelist[1] - 90
    anglelist[2] = anglelist[2] - 90
    angle3 = anglelist[2]
    anglelist[2] = -(anglelist[2] + anglelist[1])
    #print("변환된 각도 리스트 : ",anglelist)
    T_0_0 = np.dot(RotZ(anglelist[0]),Trans(0, 12, 19))
    T_0_1 = np.dot(RotX(anglelist[1]),Trans(0, Link_2, 0))
    T_1_2 = np.dot(RotX(anglelist[2]),Trans(0, Link_3, 0))
    T_2_G = np.dot(RotX(angle3),Trans(0, 31, -35))
    T_total = np.dot(T_0_0,np.dot(T_0_1,np.dot(T_1_2,T_2_G)))

    # print("T : ",T_0_0)
    #print("T total : ",T_total)
    a=np.array(T_total)

    #print("3x3 : ",a[0:3,0:3])
    
    # ASD = T_total[0:3, 0:3]
    # ASR = T_total[0:3, 3:4] 
    # print("T의 R행렬 : ", ASD)
    # print("T의 T행렬 : ", ASR)
    
    
    y0 = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    y0_0 = np.dot(T_0_0,y0)
    y0_1 = np.dot(T_0_0,np.dot(T_0_1 ,y0))
    y0_2 = np.dot(T_total,y0)

    originX = [y0[0][3],y0_0[0][3] ,y0_1[0][3], y0_2[0][3]]
    originY = [y0[1][3],y0_0[1][3] ,y0_1[1][3], y0_2[1][3]]
    originZ = [y0[2][3],y0_0[2][3] ,y0_1[2][3], y0_2[2][3]]
    # print("origin : ",originX, originY, originZ)
    
    R_total = T_total[0:3,0:3]
    
    #print(R_total)
    
    xyz = np.array([[originX[3]],[originY[3]],[originZ[3]]])


    np.save("Robot_rotation_vectors.npy", R_total)
    np.save("Robot_translation_vectors.npy", xyz)
    return (originX[3],originY[3],originZ[3])


                    


    
    
