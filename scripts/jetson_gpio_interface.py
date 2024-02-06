#! /usr/bin/env python3

from jetson_orin_agx_gpio_board.srv import GpioSetOutput                                                           
from jetson_orin_agx_gpio_board.msg import GpioStates

# import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node


class JetsonGPIOInterface(Node):

    def __init__(self):
        super().__init__('jetson_gpio_interface')
        
        # self.output_0_state = GPIO.LOW   
        # self.output_1_state = GPIO.LOW   
        # self.output_2_state = GPIO.LOW   
        # self.output_3_state = GPIO.LOW    

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(8, GPIO.IN)
        # GPIO.setup(9, GPIO.IN)
        # GPIO.setup(17, GPIO.OUT, initial=self.output_0_state)
        # GPIO.setup(27, GPIO.OUT, initial=self.output_1_state)
        # GPIO.setup(32, GPIO.OUT, initial=self.output_2_state)
        # GPIO.setup(35, GPIO.OUT, initial=self.output_3_state)

        # self.input_0_state = GPIO.input(8) 
        # self.input_1_state = GPIO.input(9)
         
        self.input_0_state = False  
        self.input_1_state = False  
        self.output_0_state = False    
        self.output_1_state = False     
        self.output_2_state = False     
        self.output_3_state = False  

        # Services
        self.gpio_srv = self.create_service(GpioSetOutput, '/gpio/set_output', self.gpio_set_output) 

        # Publishers
        self.status_message_publisher = self.create_publisher(GpioStates, '/gpio/states', 10)

        # Timers
        self.read_inputs_timer = self.create_timer(0.1, self.read_inputs_callback)  # 10Hz

    def read_inputs_callback(self, ):
        # self.input_0_state = GPIO.input(8)
        # self.input_1_state = GPIO.input(9)

        gpio_status_msg = GpioStates()
        gpio_status_msg.input_0 = self.input_0_state
        gpio_status_msg.input_1 = self.input_1_state
        gpio_status_msg.output_0 = self.output_0_state
        gpio_status_msg.output_1 = self.output_1_state
        gpio_status_msg.output_2 = self.output_2_state
        gpio_status_msg.output_3 = self.output_3_state

        self.status_message_publisher.publish(gpio_status_msg)

    def gpio_set_output(self, request, response):
        if request.output_id == 0:
            self.output_0_state = request.state
            # self.output_0_state = GPIO.HIGH if request.state > 0 else GPIO.LOW
            # GPIO.output(17, self.output_0_state)
            self.get_logger().info(f'Output 0 set to {self.output_0_state}') 
            response.response = True
        
        elif request.output_id == 1:
            self.output_1_state = request.state
            # self.output_1_state = GPIO.HIGH if request.state > 0 else GPIO.LOW
            # GPIO.output(27, self.output_1_state)
            self.get_logger().info(f'Output 1 set to {self.output_1_state}') 
            response.response = True
    
        elif request.output_id == 2:
            self.output_2_state = request.state
            # self.output_2_state = GPIO.HIGH if request.state > 0 else GPIO.LOW
            # GPIO.output(32, self.output_2_state)
            self.get_logger().info(f'Output 2 set to {self.output_2_state}') 
            response.response = True

        elif request.output_id == 3:
            self.output_3_state = request.state 
            # self.output_3_state = GPIO.HIGH if request.state > 0 else GPIO.LOW
            # GPIO.output(35, self.output_3_state)
            self.get_logger().info(f'Output 3 set to {self.output_3_state}') 
            response.response = True

        else:
            response.response = False

        return response

def main(args=None):
    rclpy.init(args=args)

    jetson_gpio_interface = JetsonGPIOInterface()

    rclpy.spin(jetson_gpio_interface)

    rclpy.shutdown()

    # GPIO.cleanup()


if __name__ == '__main__':
    main()