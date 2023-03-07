import math
import numpy as np

Link_1 = 40
Link_2 = 150
Link_3 = 200
pi = 3.141592

def xyz_to_angle(x, y, z) :
    z = z -40
    t = math.sqrt(x**2 + y**2) 
    CosTheta2 = (t**2 + z**2 - Link_2**2 - Link_3**2)/(2*Link_2*Link_3)
    if((CosTheta2<-1)or(CosTheta2>1)):
        print("Unable to convert angle!")
        return 0
    SinTheta2 = -1*math.sqrt(1-(CosTheta2)**2)
    Theta2 = math.degrees(math.atan2(SinTheta2, CosTheta2))
    
    k1 = Link_2 + Link_3*CosTheta2
    k2 = Link_3*SinTheta2
    
    Theta1 =math.degrees(math.atan2(z,t) - math.atan2(k2,k1))
    angle_base = math.degrees(math.atan2(y,x))
    angle_1 = Theta1
    angle_2 = 180-abs(Theta2) + Theta1
    
    
    #모터 각도 범위내에 존재해야 실행 되게 만들기.
    if((angle_base>=-90)and(angle_base<=90)):
        if((angle_1>5)and(angle_1<130)):
            if((angle_2>90)and(angle_2<175)):
                #print(angle_base,angle_1,angle_2)
                return [angle_base, angle_1, angle_2]
    
    #print(angle_base,angle_1,angle_2)
    print("The angle is over the range.")
    return 0



def angle_to_pos(angle) :
    a = [angle[0]*4095/360,angle[1]*4095/360,angle[2]*4095/360]
    a_r = list(map(int, a))
    return a_r

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
    anglelist[0] = anglelist[0] - 90
    anglelist[2] = anglelist[2] - anglelist[1] - 180
    print(anglelist)
    T_0_0 = np.dot(RotZ(anglelist[0]),Trans(0, 0, Link_1))
    T_0_1 = np.dot(RotX(anglelist[1]),Trans(0, Link_2, 0))
    T_1_2 = np.dot(RotX(anglelist[2]),Trans(0, Link_3, 0))
    T_total = np.dot(np.dot(T_0_0,T_0_1),T_1_2)
    print("T : ",T_0_0)
    print("T total : ",T_total)
    
    y0 = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    y0_0 = np.dot(T_0_0,y0)
    y0_1 = np.dot(np.dot(T_0_0,T_0_1) ,y0)
    y0_2 = np.dot(T_total,y0)
    
    originX = [y0[0][3],y0_0[0][3] ,y0_1[0][3], y0_2[0][3]]
    originY = [y0[1][3],y0_0[1][3] ,y0_1[1][3], y0_2[1][3]]
    originZ = [y0[2][3],y0_0[2][3] ,y0_1[2][3], y0_2[2][3]]
    print("origin : ",originX, originY, originZ)
    return (originX[3],originY[3],originZ[3])
    
    