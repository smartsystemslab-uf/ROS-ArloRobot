import Jetson.GPIO as GPIO

# Set duty cycle changes the duty cycle of a specific pwm_object

def set_duty_cycle(duty_cycle, pwm_obj):
    pwm_obj.ChangeDutyCycle(duty_cycle)
def main():
    # General GPIO setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(33, GPIO.OUT)

    xy_pwm = GPIO.PWM(33, 100) # PWM object creation on pin 33, with 100Hz frequency
    xy_pwm.start(0)
    set_duty_cycle(70, xy_pwm)

if __name__ == "__main__":
    main()