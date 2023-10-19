import math 

w = 0.8100780674079999
x = 0.00498656303846
y = 0.014852137789399998
z = 0.586112679352


#roll (x-axis rotation)
sinr_cosp = 2 * (w * x + y * z)
cosr_cosp = 1 - 2 * (x * x + y * y)
roll = math.atan2(sinr_cosp, cosr_cosp)

#pitch (y-axis rotation)
sinp = math.sqrt(1 + 2 * (w * y - x * z))
cosp = math.sqrt(1 - 2 * (w * y - x * z))
pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

#yaw (z-axis rotation)
siny_cosp = 2 * (w * z + x * y)
cosy_cosp = 1 - 2 * (y * y + z * z)
yaw = math.atan2(siny_cosp, cosy_cosp)

print(roll)
print(pitch)
print(yaw)
cr = math.cos(roll * 0.5)
sr = math.sin(roll * 0.5)
cp = math.cos(pitch * 0.5)
sp = math.sin(pitch * 0.5)
cy = math.cos(yaw * 0.5)
sy = math.sin(yaw * 0.5)

#Quaternion q;
w = cr * cp * cy + sr * sp * sy
x = sr * cp * cy - cr * sp * sy
y = cr * sp * cy + sr * cp * sy
z = cr * cp * sy - sr * sp * cy

print(x)
print(y)
print(z)
print(w)