import os
import sys
import numpy as np
import ast
import cv2
import math
import RPi.GPIO as GPIO
import serial

import fun_def_modified as fun

from time import sleep
from multiprocessing import Process, Pipe, Manager, Value, Event
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QSlider
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage




if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.


# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    
    ADDR_PROFILE_ACCELERATION   = 108
    ADDR_PROFILE_VELOCITY       = 112
    
    ADDR_POSITION_P_GAIN        = 84
    ADDR_POSITION_I_GAIN        = 82
    ADDR_POSITION_D_GAIN        = 80

PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID1                      = 1
DXL_ID2                      = 2
DXL_ID3                      = 3

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'
#DEVICENAME                  = 'COM4'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
#dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]  # Goal position


thr_min = 84
thr_max = 256


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

#set profile acceleration, profile velocity
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_PROFILE_ACCELERATION, 12)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_PROFILE_VELOCITY, 80)  
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_PROFILE_ACCELERATION, 10)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_PROFILE_VELOCITY, 100) 
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_PROFILE_ACCELERATION, 10)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_PROFILE_VELOCITY, 100)                 

#PID SET
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID1, ADDR_POSITION_P_GAIN, 640)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID1, ADDR_POSITION_I_GAIN, 1200)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID1, ADDR_POSITION_D_GAIN, 3600)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, ADDR_POSITION_P_GAIN, 640)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, ADDR_POSITION_I_GAIN, 1200)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID2, ADDR_POSITION_D_GAIN, 3600)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, ADDR_POSITION_P_GAIN, 640)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, ADDR_POSITION_I_GAIN, 1200)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID3, ADDR_POSITION_D_GAIN, 3600)

First_xyz = [0,170,105]
First_pos = fun.xyz_to_angle(First_xyz)
print(First_pos)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION, First_pos[0])
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION, First_pos[1])
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_GOAL_POSITION, First_pos[2])

STAY_STATE = False

if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


STOP  = 0
FORWARD  = 1
BACKWORD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

divs = 20
cols = 3
rows = divs 

class Box:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height

# 핀 설정 함수
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴 
    pwm = GPIO.PWM(EN, 100) 
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm

# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)  
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    #뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

        

def setMotor(ch, speed, stat):
    if ch == CH1:
        #핀 설정 후 pwm 핸들을 리턴 받음
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #핀 설정 후 pwm 핸들을 리턴 받음
        setMotorContorl(pwmB, IN3, IN4, speed, stat)


    
def servo_angle(data):
    if data >= 170:
        data = 170
    elif data <=0:
        data = 0
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(0.1) #wait for serial to open
        if arduino.isOpen():
            try:

                cmd=str(data)
                arduino.write(cmd.encode())
                arduino.flush()
                time.sleep(0.1) #wait for arduino to answer

                if arduino.inWaiting()>0: 
                    answer=arduino.readline()
                    print(answer)
                    arduino.flushInput() #remove data after reading


            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")


def check_box_size(size,d1,d2,functionangle):    # return height 
    if 750<size<1120:
        if abs(d1-d2)<25:
            return 1, -100 + 28
        elif abs(d1-d2) > 40:
            if (d1>d2 and functionangle>44.8) or (d1<d2 and functionangle<=44.8): # 원래 90도
                return 2, -100 + 21
            elif (d1<d2 and functionangle>44.8) or (d1>d2 and functionangle<=44.8):            
                return 4, -100 + 21
    # elif 750 < size < 880:
    #     return 1, -100+28        
    if 1120<size<1550:
        return 3, -100 + 28
    else:
        return 5, 0


def servoangle_cal(x,y,angle1,angle2, d1, d2, size):
    theta = math.degrees(math.atan(abs(x/y)))
    clock = True
    recbox = 0
    if x < 0:
        angle = theta + angle2 #시계
        angle_result = 90 + angle
        clock = True 
        if abs(d1-d2)>40 and d1>d2 and 750<size<1100:
            recbox = 4
        elif abs(d1-d2)>40 and d1<d2 and 750<size<1100:
            recbox = 2
        if angle > 80:
            angle = 90 - theta - angle2 #반시계
            angle_result = 90 - angle
            clock = False
            if abs(d1-d2)>40 and d1>d2 and 750<size<1100:
                recbox = 2
            elif abs(d1-d2)>40 and d1<d2 and 750<size<1100:
                recbox = 4
    
    elif x > 0:
        angle = theta + angle1 #반시계
        angle_result = 90 - angle
        clock = False
        if abs(d1-d2)>40 and d1>d2 and 750<size<1100:
            recbox = 2
        elif abs(d1-d2)>40 and d1<d2 and 750<size<1100:
            recbox = 4
        if angle > 80:
            angle = 90 - theta - angle1 #시계
            angle_result = 90 + angle
            clock = True
            if abs(d1-d2)>40 and d1>d2 and 750<size<1100:
                recbox = 4
            elif abs(d1-d2)>40 and d1<d2 and 750<size<1100:
                recbox = 2
        
    return angle_result, angle, recbox
     
def on_thresold_min_change(pos):
    global thr_min
    thr_min = np.clip(pos * 2, 0, 255)

def on_thresold_max_change(pos):
    global thr_max 
    thr_max = np.clip(pos * 2, 0, 255)

class ImageProcessingAndGUIProcess(Process):
    def __init__(self, pipe_to_main,shared_stay,space_clear_event, first_pos_event, stop_event):
        super().__init__()
        self.pipe_to_main = pipe_to_main
        self.timer2 = False
        self.playing = True
        self.stay = shared_stay
        self.space_clear_event = space_clear_event
        self.first_pos_event = first_pos_event
        self.stop_event = stop_event
        self.thr_min = 188
        self.thr_max = 255   
        self.begin = time.time()

    def stop_motor(self):
        if self.stop_event.is_set():
            self.stop_event.clear()
        else:
            self.stop_event.set()
    def space_clear_button_clicked(self):
        self.space_clear_event.set() 
        
    def first_pos_event_clicked(self):
        self.first_pos_event.set()
        
    def set_wait_state(self):
        self.state_label.setText("Wait")

    def set_detect_state(self):
        self.state_label.setText("Detect")

    def set_move_state(self):
        self.state_label.setText("Move")    
    def update_thr_min(self, value):
        self.thr_min = value

    def update_thr_max(self, value):
        self.thr_max = value   
        
        
    def run(self):
        app = QApplication([])

        window = QMainWindow()
        window.setWindowTitle("Video Player")
        window.setGeometry(160, 80, 1600, 750) # x, y, width, height

        layout = QVBoxLayout(window)
        
        state_layout = QVBoxLayout()
        
        self.state_label = QPushButton("Wait",window)
        self.state_label.resize(120,60)
        self.state_label.move(720,30)
        font = self.state_label.font()
        font.setPointSize(20)
        self.state_label.setFont(font)
        state_layout.addWidget(self.state_label)
        
        layout.addLayout(state_layout)
        
        video_layout = QHBoxLayout()
        # Create two image labels
        image_label1 = QLabel(window)
        image_label1.resize(640,480)
        image_label1.move(105,100)
        video_layout.addWidget(image_label1)
        
        image_label2 = QLabel(window)
        image_label2.resize(640,480)
        image_label2.move(850,100)
        video_layout.addWidget(image_label2)

        layout.addLayout(video_layout)
        
        control_layout = QHBoxLayout()

        # Create "Start" and "Stop" buttons
        # start_button = QPushButton("RESET", window)
        # start_button.clicked.connect(self.start_video)
        # start_button.resize(200,100)
        # start_button.move(100,650)
        # font = start_button.font()
        # font.setPointSize(35)
        # start_button.setFont(font)
        # control_layout.addWidget(start_button)
        
        stop_button = QPushButton("STOP", window)
        stop_button.clicked.connect(self.stop_motor)
        stop_button.resize(200,80)
        stop_button.move(200,650)
        font = stop_button.font()
        font.setPointSize(35)
        stop_button.setFont(font)
        control_layout.addWidget(stop_button)
        
        space_clear_button = QPushButton("SPACE CLEAR", window)
        space_clear_button.clicked.connect(self.space_clear_button_clicked)
        space_clear_button.resize(200,80)
        space_clear_button.move(680,650)
        font = space_clear_button.font()
        font.setPointSize(20)
        space_clear_button.setFont(font)
        control_layout.addWidget(space_clear_button)
        
        first_pos_event = QPushButton("RESET", window)
        first_pos_event.clicked.connect(self.first_pos_event_clicked)
        first_pos_event.resize(200,80)
        first_pos_event.move(1180,650)
        font = first_pos_event.font()
        font.setPointSize(25)
        first_pos_event.setFont(font)
        control_layout.addWidget(first_pos_event)
        
        layout.addLayout(control_layout)
        
        window.setLayout(layout)
        # Open the webcam
        bar_layout = QHBoxLayout()
        thr_min_slider = QSlider(Qt.Horizontal, window)
        thr_max_slider = QSlider(Qt.Horizontal, window)

        thr_min_slider.setRange(0, 255)
        thr_max_slider.setRange(0, 255)
        
        thr_min_slider.resize(640,30)
        thr_max_slider.resize(640,30)
        
        thr_min_slider.move (850,580)
        thr_max_slider.move (850,610)

        thr_min_slider.setValue(self.thr_min)
        thr_max_slider.setValue(self.thr_max)

        bar_layout.addWidget(thr_min_slider)
        bar_layout.addWidget(thr_max_slider)

        # Connect slider value changes to functions for threshold adjustment
        thr_min_slider.valueChanged.connect(self.update_thr_min)
        thr_max_slider.valueChanged.connect(self.update_thr_max)

        layout.addLayout(bar_layout)
     
        window.setLayout(layout)        
        
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  #cv2.CAP_V4L2, cv2.CAP_DSHOW
        
        h = 480   #width, height
        w = 640
        mtx = np.array([[1.06831554e+03, 0.00000000e+00, 3.51022111e+02],
                [0.00000000e+00, 1.06416474e+03, 1.60115465e+02],
                [0.00000000e+00,0.00000000e+00, 1.00000000e+00]])
                
        dist = np.array([[-0.38148692,  0.16373289 , 0.01313154 , 0.0044631 ,  1.61561833]])
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        def update_image():
            if self.playing:   # self.stay.value == True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Can't read camera")
                    print(ret)
                    return 0
                frame = cv2.rotate(frame, cv2.ROTATE_180)

                
                dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
                x,y,w,h = roi
                dst = dst[y:y+h, x:x+w]
                # aff = np.array([[1,0,-80],[0,1,0]], dtype=np.float32)
                # dst = cv2.warpAffine(dst, aff, (0,0))
                hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

                # Define the lower and upper HSV thresholds for orange color
                lower_orange = np.array([10, 6, 40])  # Adjust these values as needed
                upper_orange = np.array([38, 255, 255])  # Adjust these values as needed
                
                mask = cv2.inRange(hsv, lower_orange, upper_orange)
                result = cv2.bitwise_and(dst, dst, mask=mask)
                
                gray2 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
                gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

                ret, binary = cv2.threshold(gray2, self.thr_min, self.thr_max   , cv2.THRESH_BINARY)  #여기 숫자값이 thr_min과 thr_max인데 이거 조정해서 노이즈 정하기
                #binary = cv2.bitwise_not(binary)  # 검은색 바탕에 흰색 검출이면 이거 없어도 됨.

                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    end = time.time()
                    approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                    if len(approx) == 4 and self.stay.value == True:
                        perimeter = cv2.arcLength(contour, True)
                        rotateRect = cv2.minAreaRect(contour)  #좌표 감싸는 최소한의 사각형 계산
                        vertex = cv2.boxPoints(rotateRect)  #최소한의 사각형에서 꼭지점 좌표 반환
                        vertex = np.int0(vertex)
                        #distance = np.sqrt(np.sum((vertex[-1] - vertex[-2])**2))
                        # calculate the time elapsed between the last two positions

                        # calculate the velocity as the distance divided by time
                        if 750<perimeter < 1300: 

                            if self.timer2 == False:
                                self.begin = time.time()
                                self.set_detect_state()
                                self.timer2 = True
                            #중심점좌표
                            if end-self.begin >=1.4 and self.timer2 == True and end-self.begin < 1.5: #타이머 2초 경과 시 실행
                                self.set_move_state()
                                x0 = (vertex[0][0] + vertex[2][0])/2 
                                y0 = (vertex[0][1] + vertex[2][1])/2
                                    
                                #기울어진 각도
                                x1 = abs(vertex[1][0] - vertex[0][0]) + 1
                                y1 = abs(vertex[0][1] - vertex[1][1])
                                
                                x2 = abs(vertex[1][0] - vertex[2][0]) + 1
                                y2 = abs(vertex[1][1] - vertex[2][1])
                                theta1 = math.degrees(math.atan(y1/x1))
                                theta2 = math.degrees(math.atan(y2/x2))
                                
                                
                                d1 = abs(x1**2 + y1**2)**(1/2)
                                d2 = abs(x2**2 + y2**2)**(1/2)      
                                #566*105, 456*100
                                x_dist = (x0 - 640/2 )/5.5
                                y_dist = (y0 - 480/2)/5.5
                                
                                if x_dist < 0 and x_dist > -20:
                                    x_dist -=5
                                elif x_dist<=-20 and x_dist > -40:
                                    x_dist -= 7
                                elif x_dist >0 and x_dist < 10:
                                    x_dist -= 2
                                elif x_dist >= 10 and x_dist < 30:
                                    x_dist -= 5
                                elif x_dist >= 30 and x_dist < 50:
                                    x_dist -= 7
                                
                                data = [[x_dist,y_dist],[theta1,theta2],[d1,d2],[perimeter, 0]]
                                
                                self.pipe_to_main.send(data)
                                # print("data send")
                                time.sleep(0.4)
                                #conn.close()
                                self.timer2 = False  
                            if self.stay.value == True:
                                for j in range(4):
                                    cv2.drawContours(dst, [vertex], 0, (0, 0, 255), 2) 
                                    cv2.putText(dst, str(tuple(vertex[j])), tuple(vertex[j]), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 1) 
                            break                                   
                    if end-self.begin > 1.5 and self.stay.value == True:
                        self.set_wait_state()
                        self.begin = time.time()
                        self.timer2 = False


                # Convert frames to RGB format for PyQt
                frame_original_rgb = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)
                frame_processed_rgb = cv2.cvtColor(binary, cv2.COLOR_BGR2RGB)

                # Convert NumPy arrays to QImage and QPixmap
                h1,w1,c1 = frame_original_rgb.shape
                h2,w2,c2 = frame_processed_rgb.shape
                
                qimage_original = QImage(frame_original_rgb, w1, h1, w1*c1,QImage.Format_RGB888)
                pixmap_original = QPixmap.fromImage(qimage_original)

                qimage_processed = QImage(frame_processed_rgb, w2, h2, w2*c2, QImage.Format_RGB888)
                pixmap_processed = QPixmap.fromImage(qimage_processed)

                # Set the QPixmap objects as the labels' pixmaps
                image_label1.setPixmap(pixmap_original)
                image_label2.setPixmap(pixmap_processed)
                    #self.label_processed.setPixmap(pixmap_processed)

        timer = QTimer()
        timer.timeout.connect(update_image)
        timer.start(50)  # Update the frame every 1 millisecond (you can adjust this interval)

        window.show()
        app.exec_()
    
def initialize_space():
    return np.zeros((150, 150, 150), dtype=int)

def find_position(space, box):
    # Bottom-Left-Fill algorithm (BLF)
    x, y, z = 0, 0, 0
    for z in range(space.shape[0] - box.height + 1):
        for y in range(space.shape[1] - box.width + 1):
            for x in range(space.shape[2] - box.length + 1):
                if check_space_available(space, x, y, z, box):
                    return x, y, z
    return None  

def check_space_available(space, x, y, z, box):
    for dz in range(box.height):
        for dy in range(box.width):
            for dx in range(box.length):
                if space[z+dz, y+dy, x+dx]:
                    return False
    return True

def load_box(space, box):
    x, y, z = find_position(space, box)
    if x is not None:
        for dz in range(box.height):
            for dy in range(box.width):
                for dx in range(box.length):
                    space[z+dz, y+dy, x+dx] = 1 # 적재공간 [150,150,150]인 3차원 배열을 전부 0으로 초기화하고 적재하는 공간만 1로 만들어 할당하는 방식
        # Calculate the center position (x, y) and height (z) of the box
        center_x = x + box.length // 2
        center_y = y + box.width // 2
        box_height = z + box.height
        xyz = [center_x, center_y, box_height]
        return xyz
    return None  # Box could not be loaded

def coor_cali(xyz):
    xyz = [-287+xyz[1],-75+xyz[0],-100+xyz[2]] 

    return xyz    
def load_box_angle(xyz):
    load_angle = math.degrees(math.atan(abs(xyz[1]/xyz[0])))
    if xyz[1] < 0: return 90 + load_angle
    elif xyz[1] >=0: return 90 - load_angle

def linear_generator(xyz,next_xyz):
    i = 0
    A = 1
    linear_trans = [[0 for j in range(cols)] for i in range(rows)]
    
    while i in range(0,divs):  #좌표 사이를 divs개로 나눈 배열 묶음 저장
  
        linear_trans[i][0] = xyz[0] - (xyz[0] - next_xyz[0])*i/divs
        linear_trans[i][1] = xyz[1] - (xyz[1] - next_xyz[1])*i/divs
        linear_trans[i][2] = xyz[2] - (xyz[2] - next_xyz[2])*i/divs
        #print(linear_trans)
        i += 1
        
    return linear_trans


if __name__ == '__main__':
    space = initialize_space() #적재공간 초기화
    GPIO.setmode(GPIO.BCM) 
    box_count = 0
    
    #모터 핀 설정
    #핀 설정후 PWM 핸들 얻어옴 
    pwmA = setPinConfig(ENA, IN1, IN2)
    pwmB = setPinConfig(ENB, IN3, IN4)
    
    loading_space = (150, 150, 150) # 적재공간과 박스 크기 정의
    box1 = Box(35, 35, 32)
    box2 = Box(51, 35, 25)
    box4 = Box(35, 51, 25)
    box3 = Box(54, 54, 30)
    
    main_to_combined, combined_to_main = Pipe() # Pipe 사용하여 변수 프로세스 간 변수 전송

    stay = Value('i', 1)
    space_clear_event = Event() # 버튼 이벤트 설정
    first_pos_event = Event() # 버튼 이벤트 설정
    stop_event = Event() # 버튼 이벤트 설정
    stop_event.set() 
    # 멀티프로세스 정의
    combined_process = ImageProcessingAndGUIProcess(combined_to_main,stay,space_clear_event, first_pos_event,stop_event)
    combined_process.start() # 멀티프로세스 시작
    time.sleep(2)
    while True:
        if stay.value == True:
            if space_clear_event.is_set(): #버튼눌렀으면 적재공간 초기화
                space = initialize_space()
                print("SPACE CLEAR!")
                space_clear_event.clear()
            data_from_motion_contour = main_to_combined.recv()
            if data_from_motion_contour is None:
                break
            
            print("data : ", data_from_motion_contour)
            box_count += 1
            
            dxl_present_position0, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_POSITION)
            dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION)
            dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRESENT_POSITION)
            campos = [dxl_present_position0,dxl_present_position1,dxl_present_position2]
            result = fun.camera_xyz(campos)     #현재 카메라 좌표
            
            angle, xyzangle, recbox = servoangle_cal(result[0] + data_from_motion_contour[0][0],result[1] - data_from_motion_contour[0][1],data_from_motion_contour[1][0],data_from_motion_contour[1][1],data_from_motion_contour[2][0],data_from_motion_contour[2][1],data_from_motion_contour[3][0])
            box_num, box_height = check_box_size(data_from_motion_contour[3][0],data_from_motion_contour[2][0],data_from_motion_contour[2][1], data_from_motion_contour[1][1])
            if box_num == 5: #사이즈를 벗어나는 크기를 입력받으면 다음꺼 기다리기
                break
            
            box = Box(0,0,0)
            
            if box_num == 1: box = box1
            elif recbox == 2: 
                box_num = recbox
                box = box2
            elif box_num == 3: box = box3
            elif recbox == 4: 
                box_num = recbox
                box = box4
            print ("BOX NUMBER : ", box_num)
            
            box_xyz = load_box(space, box)      
            box_xyz = coor_cali(box_xyz)  
            box_xyz[2] -= 18 
            
            xyz1 = [result[0] + data_from_motion_contour[0][0],result[1]- data_from_motion_contour[0][1], box_height+10]  
            xyz2 = [result[0] + data_from_motion_contour[0][0],result[1]- data_from_motion_contour[0][1], box_height]
            xyz3 = [result[0] + data_from_motion_contour[0][0], 160, 80] 
            xyz4 = [-130 ,0, 80]
            xyz5 = [-180,0, box_xyz[2]+ 80]
            xyz6 = [box_xyz[0],box_xyz[1],box_xyz[2]+ 80]
            xyz7 = [box_xyz[0],box_xyz[1],box_xyz[2]] #실제 적재할 좌표. 
            xyz8 = [box_xyz[0],box_xyz[1],box_xyz[2]+ 80]
            xyz9 = [-180,0,box_xyz[2]+ 80]
            xyz10 = [0,170,105]

            xyz = [xyz1,xyz2,xyz3,xyz4,xyz5,xyz6, xyz7, xyz8, xyz9,xyz10] 

            
            pos1 = fun.xyz_to_angle(xyz1)
            pos2 = fun.xyz_to_angle(xyz2)
            pos3 = fun.xyz_to_angle(xyz3)
            pos4 = fun.xyz_to_angle(xyz4)
            pos5 = fun.xyz_to_angle(xyz5)
            pos6 = fun.xyz_to_angle(xyz6)
            pos7 = fun.xyz_to_angle(xyz7)
            pos8 = fun.xyz_to_angle(xyz8)
            pos9 = fun.xyz_to_angle(xyz9)
            pos10 = fun.xyz_to_angle(xyz10)

            
            pos = [pos1,pos2,pos3,pos4,pos5,pos6,pos7, pos8,pos9,pos10]
            count = 1
            stay.value = 0
            
            print("load xyz : ", xyz7)
        
        
        elif stay.value == False:
            while count in range(11):
                time.sleep(0.3)
                A = 1
                if space_clear_event.is_set(): #버튼눌렀으면 적재공간 초기화
                    space = initialize_space()
                    print("SPACE CLEAR!")
                    space_clear_event.clear()
                    
                if not stop_event.is_set(): #버튼 예외처리 
                    stop_event.wait()
                if first_pos_event.is_set():
                    
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION, pos[9][0])
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION, pos[9][1])
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_GOAL_POSITION, pos[9][2])
                    count = 1
                    first_pos_event.clear()
                    time.sleep(5)
                    setMotor(CH1, 100, STOP)
                    sleep(0.7)
                    break
                
                if count == 7 or count == 8:
                    linear_trans = linear_generator(xyz[count-2],xyz[count-1])
                    while A in range(0,divs):  #저장된 배열 묶음 따라서 순서대로 이동
                        linear_pos = fun.xyz_to_angle(linear_trans[A]) 
                        
                        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION, linear_pos[0])
                        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION, linear_pos[1])
                        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_GOAL_POSITION, linear_pos[2])
                        
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s" % packetHandler.getRxPacketError(dxl_error))
                            
                        while 1:
                            # Read present position
                            dxl_present_position0, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_POSITION)
                            dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION)
                            dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRESENT_POSITION)
                            if dxl_comm_result != COMM_SUCCESS:
                                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                            elif dxl_error != 0:
                                print("%s" % packetHandler.getRxPacketError(dxl_error))

                            if not abs(linear_pos[0] - dxl_present_position0) > DXL_MOVING_STATUS_THRESHOLD:
                                if not abs(linear_pos[1] - dxl_present_position1) > DXL_MOVING_STATUS_THRESHOLD:
                                    if not abs(linear_pos[2] - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD:
                                        break
                            
                        A += 1
                    if count == 7:
                        setMotor(CH1, 100, STOP)
                        sleep(0.7)
                    count+=1    
                    
                else: 
                           
                 # Write goal position
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_POSITION, pos[count-1][0])
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION, pos[count-1][1])
                    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID3, ADDR_GOAL_POSITION, pos[count-1][2])
                    if dxl_comm_result != COMM_SUCCESS:
                        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print("%s" % packetHandler.getRxPacketError(dxl_error))
                        
                    if count == 1:
                        
                        setMotor(CH1, 100, FORWARD)
                        time.sleep(0.3)
                        servo_angle(int(angle))
                        
                    if count == 3:
                        servo_angle(90)    
                    if count == 6:
                        load_angle = load_box_angle(xyz7)
                        servo_angle(int(load_angle))
                    if count == 9:
                        servo_angle(90)    
                        
                    while 1:
                        # Read present position
                        dxl_present_position0, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRESENT_POSITION)
                        dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION)
                        dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRESENT_POSITION)
                        if dxl_comm_result != COMM_SUCCESS:
                            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                        elif dxl_error != 0:
                            print("%s" % packetHandler.getRxPacketError(dxl_error))


                        if not abs(pos[count-1][0] - dxl_present_position0) > DXL_MOVING_STATUS_THRESHOLD:
                            if not abs(pos[count-1][1] - dxl_present_position1) > DXL_MOVING_STATUS_THRESHOLD:
                                if not abs(pos[count-1][2] - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD:
                                    PRESENT = fun.prexyz(np.array([dxl_present_position0,dxl_present_position1,dxl_present_position2]))
                                    #print("현재위치 : ",PRESENT) 
                                    print("position : ", count)  
                                    break
                       
                                            
                    count += 1

            time.sleep(2)
            stay.value = 1
            
    GPIO.cleanup()
    combined_process.join()
    