#!/usr/bin/env python3
import sys
import time
import termios
import tty

# Try to import RPi.GPIO, otherwise use a mock for testing on non-Pi systems
try:
    import RPi.GPIO as GPIO
    MOCK_MODE = False
except (ImportError, RuntimeError):
    MOCK_MODE = True

# Configuration
BUZZER_PIN = 12
PWM_FREQ = 2000  # 2kHz frequency

def get_key():
    """Read a single keypress from stdin without echoing."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    pwm = None
    
    if MOCK_MODE:
        print("RPi.GPIO not found. Running in MOCK mode (visual feedback only).")
    else:
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(BUZZER_PIN, GPIO.OUT)
            pwm = GPIO.PWM(BUZZER_PIN, PWM_FREQ)
            pwm.start(0)  # Start silent (0% duty cycle)
        except Exception as e:
            print(f"GPIO Setup failed: {e}")
            return

    print(f"Ready! Press SPACE to buzz (GPIO {BUZZER_PIN}). Press 'q' to quit.\r")

    try:
        while True:
            key = get_key()
            
            if key == ' ':
                if not MOCK_MODE and pwm:
                    pwm.ChangeDutyCycle(100)  # 50% duty cycle for sound
                    time.sleep(0.1)          # Buzz for 100ms
                    pwm.ChangeDutyCycle(0)   # Silence
                else:
                    print("BEEP! (Space pressed)\r")
                
                # Flush input buffer to prevent lag from key repeat
                termios.tcflush(sys.stdin, termios.TCIFLUSH)
                    
            elif key.lower() == 'q':
                print("Exiting...\r")
                break
            
            # Handle Ctrl+C (ETX) manually since raw mode captures it
            elif key == '\x03':
                print("Exiting...\r")
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        print("Cleaning up...\r")
        if not MOCK_MODE:
            if pwm:
                pwm.stop()
            GPIO.cleanup()

if __name__ == "__main__":
    main()

