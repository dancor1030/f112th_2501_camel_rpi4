sp = 90

from  Rosmaster_Lib  import  Rosmaster 
import time as tm

bot = Rosmaster()

def main():
    print('Hi from f112th_2501_control_teleop_pi.')
    while(1):
        bot.set_pwm_servo(1,sp)
        tm.sleep(1)

if __name__ == '__main__':
    main()
