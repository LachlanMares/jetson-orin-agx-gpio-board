#! /usr/bin/env python3

import time
import Jetson.GPIO as GPIO


class JetsonGPIOInterface():
    def __init__(self):
        super().__init__()
        
        self.output_0_state = GPIO.LOW   
        self.output_1_state = GPIO.LOW   
        self.output_2_state = GPIO.LOW   
        self.output_3_state = GPIO.LOW    

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(22, GPIO.OUT, initial=self.output_0_state)  # 17
        GPIO.setup(15, GPIO.OUT, initial=self.output_1_state)  # 27
        GPIO.setup(13, GPIO.OUT, initial=self.output_2_state)  # 32
        GPIO.setup(18, GPIO.OUT, initial=self.output_3_state)  # 35

    def test_outputs(self, ):
        for i in range(129):
            if i % 2 == 0:
                if self.output_0_state == GPIO.LOW:
                    self.output_0_state = GPIO.HIGH
                else:
                    self.output_0_state = GPIO.LOW

                GPIO.output(22, self.output_0_state)

            if i % 4 == 0:
                if self.output_1_state == GPIO.LOW:
                    self.output_1_state = GPIO.HIGH
                else:
                    self.output_1_state = GPIO.LOW

                GPIO.output(15, self.output_1_state)

            if i % 8 == 0:
                if self.output_2_state == GPIO.LOW:
                    self.output_2_state = GPIO.HIGH
                else:
                    self.output_2_state = GPIO.LOW

                GPIO.output(13, self.output_2_state)

            if i % 16 == 0:
                if self.output_3_state == GPIO.LOW:
                    self.output_3_state = GPIO.HIGH
                else:
                    self.output_3_state = GPIO.LOW

                GPIO.output(18, self.output_3_state)

            time.sleep(0.1)

def main(args=None):

    jetson_gpio_interface = JetsonGPIOInterface()
    jetson_gpio_interface.test_outputs()

    GPIO.cleanup()


if __name__ == '__main__':
    main()