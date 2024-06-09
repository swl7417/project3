from pymycobot.mycobot import MyCobot
import time
import keyboard

mc = MyCobot('/dev/ttyACM0',115200)


   
mc.release_all_servos()

time.sleep(15)

mc.power_on()

time.sleep(3)

angles = mc.get_angles()
print("angle: ", angles)
time.sleep(1)
