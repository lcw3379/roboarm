import fun_def as fun
import matplotlib.pyplot as plt
import numpy as np

angle = fun.xyz_to_angle(150,200,20)
print(angle)

pos = fun.angle_to_pos(angle)

print(fun.prexyz(pos))
    
plt.plot([1,2,3,4,5,6,7,8,9,10], [1,4,9,16,25,36,49,64,81,100],[1,2,3,4,5,6,7,8,9,10],[10,10,10,10,10,10,10,10,10,10])
plt.show()