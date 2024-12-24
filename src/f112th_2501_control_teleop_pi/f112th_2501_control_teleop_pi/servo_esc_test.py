from  Rosmaster_Lib  import  Rosmaster
import time as tm

bot = Rosmaster()

def main():
    print('Hi from f112th_2501_control_teleop_pi.')
    try:
     while(1):
         sp = float(input("enter speed "))
         bot.set_pwm_servo(1,sp)
         tm.sleep(0.3)
    except KeyboardInterrupt:
     bot.set_pwm_servo(1,90)
if __name__ == '__main__':
    main()
