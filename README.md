Description is available at http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/


환경

라즈베리파이 Ubuntu 22.04<br/>
Python 3.11<br/>

사용 라이브러리

OpenCV<br/>
PyQt<br/>
MultiProcessing<br/>
Dynamixel-SDK <br/>


![image](https://github.com/user-attachments/assets/3dc8b785-f954-4851-967b-d7f735976306)


기본적인 로봇 정기구학/역기구학과 OpenCV의 색깔 검출, 엣지 검출을 하였다.
또한 카메라와 로봇 좌표 연동을 하였다.

MultiProcessing으로 영상처리/UI 프로세스와 로봇제어 프로세스를 분리해서 Pipe와 Event를 이용한 통신을 하였다.

https://www.youtube.com/watch?v=oqY8NCK1Az8

작동영상이다.

###############
졸업작품을 제작할 때엔 ROS에 대한 지식이 충분하지 않아 Multiprocessing을 이용하는, 꽤나 번거로운 방법을 사용했다고 생각한다. 지금 이 프로젝트를 다시 만든다면 ROS2를 사용해서 실시간으로 로봇의 상황을 표시하고 프로세스도 나누는 구조로 제작을 할 것 같다.

![roboarm2](https://github.com/user-attachments/assets/6442825c-2dab-4e9b-8d15-48746d4a7dfd)
