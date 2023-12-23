from action_msgs.msg import GoalStatus

import RPi.GPIO as GPIO

import timeout_decorator
import time
import rclpy
import math
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Range
from std_msgs.msg import String, ColorRGBA, Bool, Float64
from geometry_msgs.msg import Vector3, Twist

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Definition of  motor pins
IN1 = 20 # Forward Left
IN2 = 21 # Backward Left
IN3 = 19 # Forward Right
IN4 = 26 # Backward Right
ENA = 16 # Engage Left
ENB = 13 # Engage Right

# Horizontal Camera
ServoPinHC = 11

# Vertical Camera
ServoPinVC = 9

#Definition of Ultrasonic Servo
ServoPin = 23

#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

EchoPin = 0
TrigPin = 1

#TrackSensorLeftPin1 TrackSensorLeftPin2 TrackSensorRightPin1 TrackSensorRightPin2
#      3                 5                  4                   18
TrackSensorLeftPin1  =  3   #The first tracking infrared sensor pin on the left is connected to  BCM port 3 of Raspberry pi
TrackSensorLeftPin2  =  5   #The second tracking infrared sensor pin on the left is connected to  BCM port 5 of Raspberry pi
TrackSensorRightPin1 =  4    #The first tracking infrared sensor pin on the right is connected to  BCM port 4 of Raspberry pi
TrackSensorRightPin2 =  18   #The second tracking infrared sensor pin on the right is connected to  BCMport 18 of Raspberry pi

#Definition of buzzer pin
buzzer = 8
#Definition of start pin
start = 8

# Debug level
DEBUG = 0

# The Robot can force him self to go off by drain to mutch power / shorting the GPIO pins. This Flag limits the max value for the base movement and disables the power rate system
SAVETY_OVERRIDE = 0
# The incrementation value for move
POWER_RATE = 0.001
CYCLE_RATE = 100

class RPiGPIO(Node):

    def __init__(self):
        super().__init__('rpi_gpio_tank')

        self.declare_parameter('DEBUG')
        DEBUG = self.get_parameter('DEBUG')

        self.declare_parameter('SAVETY_OVERRIDE')
        SAVETY_OVERRIDE = self.get_parameter('SAVETY_OVERRIDE')

        self.subscription = self.create_subscription(Twist,'cmd_vel',self.process_twist_base,10)
        self.subscription = self.create_subscription(Twist,'pan_tilt_trajectory_controller/command',self.process_twist_pan,10)
        self.subscription = self.create_subscription(Twist,'ultra_sonic_controller/command',self.process_twist_us,10)
        self.subscription = self.create_subscription(ColorRGBA,'ultra_sonic_controller/colorrgba',self.process_colorrgba,10)
        self.subscription = self.create_subscription(Bool,'extras_controller/i_O',self.process_input,10)
        self.subscription = self.create_subscription(Float64,'extras_controller/timer',self.process_float,10)
        self.subscription  # prevent unused variable warning

        # Set Global Variables
        global pwm_servoUS
        global pwm_servoHC
        global pwm_servoVC
        global pos_memH
        global pos_memV
        global pwm_ENA
        global pwm_ENB

        global ServoPin
        global pwm_servo
        global pos_memHU
        global vel

        global pwm_rled
        global pwm_gled
        global pwm_bled

        global timer_period
        global IO_ONOFF

        # Set global Variables that we need later.
        self.vel = 0.0
        self.pos_memHU = 0
        self.pos_memH = 0
        self.pos_memV = 0
        timer_period = 3
        IO_ONOFF = False
        self.base_mode_mem = 'halt'

        ENA = 16 # Engage Left
        ENB = 13 # Engage Right

        # set ultrasonic servo pin to output -> he is an actor
        GPIO.setup(ServoPin, GPIO.OUT)

        # Set Global Variables
        #GPIO.setup(buzzer,GPIO.OUT)
        GPIO.setup(start,GPIO.IN)

        #set RGB pins to output -> this are ouputs to
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)

        GPIO.setup(ServoPinHC, GPIO.OUT)
        GPIO.setup(ServoPinVC, GPIO.OUT)

        GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
        GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
        GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)

        # Set IR sensors
        GPIO.setup(TrackSensorLeftPin1,GPIO.IN)
        GPIO.setup(TrackSensorLeftPin2,GPIO.IN)
        GPIO.setup(TrackSensorRightPin1,GPIO.IN)
        GPIO.setup(TrackSensorRightPin2,GPIO.IN)

        #Set the PWM pin and frequency is 1000hz
        pwm_rled = GPIO.PWM(LED_R, 1000)
        pwm_gled = GPIO.PWM(LED_G, 1000)
        pwm_bled = GPIO.PWM(LED_B, 1000)
        pwm_rled.start(0)
        pwm_gled.start(0)
        pwm_bled.start(0)

        #Set the PWM pin and frequency is 2000hz
        pwm_ENA = GPIO.PWM(ENA, 2000)
        pwm_ENB = GPIO.PWM(ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)

        #Set the PWM pin and frequency is 50hz
        #set Engage-pin for Ultrasonic Servo
        pwm_servoUS = GPIO.PWM(ServoPin, 50)
        pwm_servoUS.start(0)

        #set Engage-pin for PAN Servos
        pwm_servoHC = GPIO.PWM(ServoPinHC, 50)
        pwm_servoHC.start(0)

        pwm_servoVC = GPIO.PWM(ServoPinVC, 50)
        pwm_servoVC.start(0)

        # Set Ultrasonic Pins
        GPIO.setup(EchoPin,GPIO.IN)
        GPIO.setup(TrigPin,GPIO.OUT)

        # Start The Publisher for Ultrasonic sensor after we hat every Port configured
        self.publisherrange = self.create_publisher(Range, 'ultra_sonic_controller/range', qos_profile=qos_profile_sensor_data)
        timer_period_sensors = 0.5  # seconds
        self.timerUS = self.create_timer(timer_period_sensors, self.timer_callback_Range)

        # Start The Publisher for Ultrasonic sensor after we hat every Port configured
        self.publisherL1 = self.create_publisher(Bool, 'ir_controller/L1', qos_profile=qos_profile_sensor_data)
        self.publisherL2 = self.create_publisher(Bool, 'ir_controller/L2', qos_profile=qos_profile_sensor_data)
        self.publisherR1 = self.create_publisher(Bool, 'ir_controller/R1', qos_profile=qos_profile_sensor_data)
        self.publisherR2 = self.create_publisher(Bool, 'ir_controller/R2', qos_profile=qos_profile_sensor_data)
        timer_period_IR = 0.5  # seconds
        self.timerIR = self.create_timer(timer_period_IR, self.timer_callback_IR)

        # Start The Publisher for Start taste
        self.publishertaste = self.create_publisher(Bool, 'extras_controller/key', qos_profile=qos_profile_sensor_data)
        self.timertaste = self.create_timer(timer_period, self.timer_callback_extra)

    def san_input_float(self, value):
        type_var = type(value)
        if type_var == float:
            value_san = value
            ret = 0
        elif type_var == int:
            value_san = float(value)
            ret = 1
        else:
            value_san = 0
            ret = 1
        if ret > 0:
            self.get_logger().error('input value: %s is not of type float and get parsed in san_input_float()' % (value))
        return(ret, type_var, value_san)

    def san_input_int_and_range(self, value, start, end):
        type_var = type(value)
        if type_var == int:
            value_san = value
            ret = 0
        elif type_var == float:
            value_san = int(value)
            ret = 1
        else:
            value_san = 0
            ret = 1
        if ret > 0:
            self.get_logger().error('input value: %s is not of type int and get parsed in san_input_int_and_range()' % (value))
            return(ret, type_var, value_san)
        else:
            if start <= value_san <= end:
                return(0 , type_var, value_san)
        return(1, type_var, 0)

    def msg_prep(self,value):
        msg = Bool()
        msg.data=bool(value)
        return(msg)

    def whistle(self):
        GPIO.setup(start,GPIO.OUT)
        GPIO.output(start, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(start, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.setup(start,GPIO.IN)

    def process_input(self,input):
        global IO_ONOFF
        IO_ONOFF = input.data
        return

    def process_float(self, value):
        global timer_period
        if timer_period != value.data:
            timer_period = value.data
            self.destroy_timer(self.timertaste)
            self.timertaste = self.create_timer(timer_period, self.timer_callback_extra)
        return


    # Timer Clallback
    def timer_callback_extra(self):
        global IO_ONOFF
        if DEBUG >= 1:
            self.get_logger().info('Extras Timer is: "%s"' % IO_ONOFF)
        taste = Bool()
        try:
             taste=self.msg_prep(GPIO.input(start))
        except:
             taste = 0
        if DEBUG >= 1:
             self.get_logger().info('Extras Key is: "%s"' %  taste)
        try:
             self.publishertaste.publish(taste)
        except:
             if DEBUG >= 1:
                self.get_logger().info('Extras Key is: "%s"' %  "not publisht")

        if IO_ONOFF:
            self.whistle()
        return

    # simple publisher for the IR range data
    def timer_callback_IR(self):
        msgL1 = Bool()
        msgL2 = Bool()
        msgR1 = Bool()
        msgR2 = Bool()
        try:
            msgL1=self.msg_prep(GPIO.input(TrackSensorLeftPin1))
            msgL2=self.msg_prep(GPIO.input(TrackSensorLeftPin2))
            msgR1=self.msg_prep(GPIO.input(TrackSensorRightPin1))
            msgR2=self.msg_prep(GPIO.input(TrackSensorRightPin2))
        except:
            msgL1 = 0
            msgL2 = 0
            msgR1 = 0
            msgR2 = 0
        if DEBUG >= 1:
            self.get_logger().info('Publishing Range L1: "%s"' % msgL1)
            self.get_logger().info('Publishing Range L2: "%s"' % msgL2)
            self.get_logger().info('Publishing Range R1: "%s"' % msgR1)
            self.get_logger().info('Publishing Range R2: "%s"' % msgR2)
        self.publisherL1.publish(msgL1)
        self.publisherL2.publish(msgL2)
        self.publisherR1.publish(msgR1)
        self.publisherR2.publish(msgR2)
        return

    # simple publisher for the ultrasonic range data
    def timer_callback_Range(self):
        msg = Range()
        #msg.ULTRASOUND=1
        #msg.INFRARED=0
        #msg.header=80
        msg.radiation_type=1
        msg.field_of_view=self.vel
        msg.min_range=0.0
        msg.max_range=1000.0
        try:
            msg.range=self.Distance_test()
        except:
            msg.range=0.0
        if DEBUG >= 1:
            self.get_logger().info('Publishing Range: "%s"' % msg.range)
        self.publisherrange.publish(msg)
        return

    #Ultrasonic distance function
    @timeout_decorator.timeout(5)
    def Distance_test(self):
        GPIO.output(TrigPin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TrigPin,GPIO.LOW)
        while not GPIO.input(EchoPin):
            pass
            t1 = time.time()
        while GPIO.input(EchoPin):
            pass
            t2 = time.time()
        if DEBUG >= 2:
            print("distance is %d " % (((t2 - t1)* 340 / 2)) * 100)
        time.sleep(0.01)
        return ((t2 - t1)* 340 / 2) * 100

    def move(self, speed, steering):
        # Init Steps
        leftspeed = 0.0
        rightspeed = 0.0
        abs_speed = abs(speed)
        massage = "Normal"
        reverse_l = False
        reverse_r = False

        # Defining Mode by check speed value

        if speed > 0:
            base_mode = 'forward'
        elif speed == 0.0:
            base_mode = 'halt'
            if steering > 0:
                 base_mode = 'spinleft'
            elif steering < 0:
                 base_mode = 'spinright'
        elif speed < 0:
            base_mode = 'backward'

        # Stopping current operation if we had to do some different

        if self.base_mode_mem != base_mode:
            pwm_ENA.ChangeDutyCycle(0)
            pwm_ENB.ChangeDutyCycle(0)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
            time.sleep(1.0)

        if steering >= 0:
            rightspeed = abs(speed)
            leftspeed = abs(abs(speed) - abs(steering))
            if abs(speed) < abs(steering):
                reverse_l = True
        elif steering < 0:
            leftspeed = abs(speed)
            rightspeed = abs(abs(speed) - abs(steering))
            if abs(speed) < abs(steering):
                reverse_r = True

        # Default for forward
        if base_mode == 'forward':
            if reverse_l:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
            else:
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
            if reverse_r:
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.HIGH)
            else:
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW)
            time.sleep(0.8)

        elif base_mode == 'backward':
            if reverse_l:
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
            else:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)

            if reverse_r:
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW)
            else:
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.HIGH)
            time.sleep(0.8)
        elif base_mode == 'spinright':
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
            time.sleep(0.8)
        elif base_mode == 'spinleft':
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
            time.sleep(0.8)
        elif  base_mode == 'halt':
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)
            time.sleep(0.8)

        self.base_mode_mem = base_mode

        if base_mode == 'forward':
            # Moving Chase
            left_int = int(round(abs(leftspeed * 100)))
            right_int = int(round(abs(rightspeed * 100)))
            self.get_logger().info('driving mode: int_left: %s , int_right: %s , steering: %s ' % (left_int,right_int,steering))
        elif base_mode == 'backward':
            # Moving Chase
            left_int = int(round(abs(leftspeed * 100)))
            right_int = int(round(abs(rightspeed * 100)))
            self.get_logger().info('driving mode: int_left: %s , int_right: %s , steering: %s ' % (left_int,right_int,steering))
        elif base_mode == 'spinleft':
            # Standing Spin case
            left_int = int(round(abs(steering) * 100))
            right_int = int(round(abs(steering) * 100))
            self.get_logger().info('rotete mode: int_left: %s , int_right: %s , steering: %s ' % (left_int,right_int,steering))
        elif base_mode == 'spinright':
            # Standing Spin case
            left_int = int(round(abs(steering) * 100))
            right_int = int(round(abs(steering) * 100))
            self.get_logger().info('rotete mode: int_left: %s , int_right: %s , steering: %s ' % (left_int,right_int,steering))
        elif base_mode == 'halt':
            # Standing Spin case
            left_int = int(round(abs(steering) * 100))
            right_int = int(round(abs(steering) * 100))
            self.get_logger().info('rotete mode: int_left: %s , int_right: %s , steering: %s ' % (left_int,right_int,steering))


        if SAVETY_OVERRIDE > 0:
            max_con_power = 100
        else:
            max_con_power = 50

        ret, ret_type, leftspeedret = self.san_input_int_and_range(left_int, 0, max_con_power)
        ret2, ret_type, rightspeedret = self.san_input_int_and_range(right_int, 0, max_con_power)

        if ret or ret2 > 0:
            if ret > 0:
                massage = "Error Left Value"
            if ret2 > 0:
                massage = "Error Right Value"
            if speed > max_con_power:
                massage = "Over Max Speed Value"

            self.get_logger().info('%s - speed: %s , steering: %s , lspeed: %s, rspeed: %s - %s' % (self.base_mode_mem, speed , steering, leftspeedret, rightspeedret, massage))
            return

        #if DEBUG >= 1:
        self.get_logger().info('%s - speed: %s , steering: %s , lspeed: %s, rspeed: %s - %s' % (self.base_mode_mem, speed , steering, leftspeedret, rightspeedret, massage))

        pwm_ENA.ChangeDutyCycle(leftspeedret)
        pwm_ENB.ChangeDutyCycle(rightspeedret)
        # Board needs time to react i guess
        time.sleep(0.8)

    # Ultrasonic module
    #The servo rotates to the specified angle
    def servo_appointed_detectionHCU(self, pos):
        for i in range(18):
            pwm_servoUS.ChangeDutyCycle(2.5 + 10 * pos/180)
        self.pos_memHU = pos   

    # Pan Tilt Mover Part
    #The servo rotates to the specified angle
    def servo_appointed_detectionHC(self, pos):
        for i in range(18):
            pwm_servoHC.ChangeDutyCycle(2.5 + 10 * pos/180)
        self.pos_memH = pos   

    # Pan Tilt Mover Part
    #The servo rotates to the specified angle
    def servo_appointed_detectionVC(self, pos):
        for i in range(18):
            pwm_servoVC.ChangeDutyCycle(2.5 + 10 * pos/180)
        self.pos_memV = pos   




    # Ultrasonic module
    def float_manituHCU(self, pos):
        print("input", pos) 
        if pos < 0:
            if DEBUG >= 2:
                print("if1", pos) 
            pos_out = round(90 - (( pos * 180 ) / 2 ) * -1)
        elif pos > 0: 
            pos_out = round(90 + (( pos * 180 ) / 2 ))
            if DEBUG >= 2:
                print("if2", pos)
        elif pos == 0:
            pos_out = 90
            if DEBUG >= 2:
                print("mid", pos)
        print("mem", self.pos_memHU)           
        print("output", pos_out) 

        if self.pos_memHU != pos_out:
            if DEBUG >= 2:
                print("change!")
            GPIO.setup(ServoPin, GPIO.OUT)       
            self.servo_appointed_detectionHCU(pos_out)  
            time.sleep(0.5)             
        elif self.pos_memHU == pos_out:
            if DEBUG >= 2:
                print("no change")
            GPIO.setup(ServoPin, GPIO.IN)

    # Pan Tilt Mover Part
    def float_manituVC(self, pos):
        if DEBUG >= 2:
            print("input", pos) 
        if pos < 0:
            if DEBUG >= 2:
                print("if1", pos) 
            pos_out = round(90 - (( pos * 180 ) / 2 ) * -1)
        elif pos > 0: 
            pos_out = round(90 + (( pos * 180 ) / 2 ))
            if DEBUG >= 2:
                print("if2", pos)
        elif pos == 0:
            # error to debug on 0 point
            #('input', 0.0)
            #('mid', 0.0)
            #('mem', 57.0)
            #('output', 90)
            #change!
            # workaround is set position up 0.10 !!! same here
            pos_out = 90
            if DEBUG >= 2:
                print("mid", pos)
        if DEBUG >= 1:
            print("mem", self.pos_memV)
            print("output", pos_out)

        if self.pos_memV != pos_out:
            if DEBUG >= 2:
                print("change!")
            GPIO.setup(ServoPinVC, GPIO.OUT)
            self.servo_appointed_detectionVC(pos_out)
            time.sleep(0.5)
        elif self.pos_memV == pos_out:
            if DEBUG >= 2:
                print("no change")
            GPIO.setup(ServoPinVC, GPIO.IN)

    # Pan Tilt Mover Part
    def float_manituHC(self, pos):
        if DEBUG >= 2:
            print("input", pos)
        if pos < 0:
            if DEBUG >= 2:
                print("if1", pos)
            pos_out = round(90 - (( pos * 180 ) / 2 ) * -1)
        elif pos > 0:
            pos_out = round(90 + (( pos * 180 ) / 2 ))
            if DEBUG >= 2:
                print("if2", pos)
        elif pos == 0:
            # error to debug on 0 point
            #('input', 0.0)
            #('mid', 0.0)
            #('mem', 57.0)
            #('output', 90)
            #change!
            # workaround is set position up 0.10
            pos_out = 90
            if DEBUG >= 1:
                print("mid", pos)
        if DEBUG >= 1:
            print("mem", self.pos_memH)
            print("output", pos_out)

        if self.pos_memH != pos_out:
            if DEBUG >= 2:
                print("change!")
            GPIO.setup(ServoPinHC, GPIO.OUT)
            self.servo_appointed_detectionHC(pos_out)
            time.sleep(0.5)
        elif self.pos_memH == pos_out:
            if DEBUG >= 2:
                print("no change")
            GPIO.setup(ServoPinHC, GPIO.IN)

    # Base part
    def process_twist_base(self, twist):
        """ Process cmd_vel messages from ROS and stash them as instance variables."""
        ret,var_type, self.vel = self.san_input_float(twist.linear.x)
        if ret > 0:
            self.get_logger().error('wrong Linear input for Base')
            return
        ret,var_type, self.omega = self.san_input_float(twist.angular.y)
        if ret > 0:
            self.get_logger().error('wrong Angular input for Base')
            return

        if DEBUG >= 1:
            self.get_logger().info('Value Movement X: %s' % (self.vel))
            self.get_logger().info('Value Movement X: %s' % (self.omega))
        if self.vel > 0:
            self.get_logger().info('Forward')
            self.move(self.vel, self.omega)
        elif self.vel < 0:
            self.get_logger().info('Backward')
            self.move(self.vel, self.omega)
        elif self.vel == 0:
        # We have no moving in forward or backward here, maybe to side?
            if self.omega < 0:
                self.get_logger().info('Rotation Right')
                self.move(self.vel, self.omega)
            elif self.omega > 0:
                self.get_logger().info('Rotation Left')
                self.move(self.vel, self.omega)
            else:
                # We have no Speed and we have no rotating order
                if DEBUG >= 1:
                    self.get_logger().info('No Movement Order - Waiting')
                self.move(0.0, 0.0)

    def float_manitu_colorrgba(self, number):
        # calc % of 100 from a float from 0 to 1
        tmp_number = 0
        if number < 0:
            tmp_number = round(( number * 100 ) * -1)
        elif number > 0: 
            tmp_number = round( number * 100 )
        # check if float is out of range 0 to 1 and correct ro maximum
        if tmp_number > 100:
            tmp_number = 100
        return(tmp_number)

    # Ultrasonic Part
    def manitu_colorrgba(self, r, g, b, a):
        tmp_r = self.float_manitu_colorrgba(r)
        tmp_g = self.float_manitu_colorrgba(g)
        tmp_b = self.float_manitu_colorrgba(b)
        tmp_a = self.float_manitu_colorrgba(a)
        if DEBUG >= 2:
            print("******")
            print(r)
            print(g)
            print(b)
            print(a)
            print("******")
            print("******")
            print(tmp_r)
            print(tmp_g)
            print(tmp_b)
            print(tmp_a)
            print("******")
        pwm_rled.ChangeDutyCycle(tmp_r)
        pwm_gled.ChangeDutyCycle(tmp_g)
        pwm_bled.ChangeDutyCycle(tmp_b)


    def process_twist_us(self, twist):
        """ Process cmd_vel messages from ROS and stash them as instance variables."""
        self.vel = twist.angular.x
        if DEBUG >= 1:
            print(self.vel) 
        self.float_manituHCU(self.vel)

    def process_colorrgba(self, colorrgba):
        """ Process cmd_vel messages from ROS and stash them as instance variables."""
        self.manitu_colorrgba(colorrgba.r, colorrgba.g, colorrgba.b, colorrgba.a)


    # Pan Tilt Mover Part
    def process_twist_pan(self, twist):
        """ Process cmd_vel messages from ROS and stash them as instance variables."""
        self.vel = twist.angular.x
        self.omega = twist.angular.y
        if DEBUG >= 1:
            print(self.vel) 
            print(self.omega) 
        self.float_manituHC(self.vel)
        self.float_manituVC(self.omega)

    def shutdown(self):
        print("in shutdown")
        GPIO.cleanup()


def main(args=None):
    rclpy.init()
    action_client = RPiGPIO()
    rclpy.spin(action_client)
    action_client.destroy_node()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
