import Jetson.GPIO as GPIO

def set_PWM_pin(PIN_nmb):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN_nmb, GPIO.OUT)
    xy_pwm = GPIO.PWM(PIN_nmb, 100)  # PWM object creation on pin 33, with 100Hz frequency
    xy_pwm.start(0)


# Set duty cycle changes the duty cycle of a specific pwm_object
def set_duty_cycle(duty_cycle, pwm_obj):
    pwm_obj.ChangeDutyCycle(duty_cycle)
def main():
    # General GPIO setup
    set_PWM_pin(33)
    set_duty_cycle(70, xy_pwm)

if __name__ == "__main__":
    main()