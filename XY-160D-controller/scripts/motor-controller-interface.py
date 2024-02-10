import Jetson.GPIO as GPIO
import time


def set_PWM_pin(PIN_nmb):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN_nmb, GPIO.OUT)
    xy_pwm = GPIO.PWM(PIN_nmb, 100)  # PWM object creation on pin 33, with 100Hz frequency
    xy_pwm.start(0)
    return xy_pwm


# Set duty cycle changes the duty cycle of a specific pwm_object
def set_duty_cycle(duty_cycle, pwm_obj):
    pwm_obj.ChangeDutyCycle(duty_cycle)


def main():
    # General GPIO setup
    xy_pwm = set_PWM_pin(33)
    # Jetson nano sample code to test pwm pin:
    val = 25
    incr = 5
    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)
            if val >= 100:
                incr = -incr
            if val <= 0:
                incr = -incr
            val += incr
            xy_pwm.ChangeDutyCycle(val)
    finally:
        xy_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
