#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys

# Configuration
FAN_PIN = 5        # BCM pin number
PWM_FREQ = 100     # Frequency in Hz (100Hz is good for MOSFETs, 25000Hz for 4-pin PWM fans)

def main():
    print(f"Initializing Fan on GPIO {FAN_PIN} with {PWM_FREQ}Hz frequency")
    
    try:
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(FAN_PIN, GPIO.OUT)
        
        # Create PWM instance
        fan = GPIO.PWM(FAN_PIN, PWM_FREQ)
        fan.start(0) # Start at 0%
        
        print("\nFan Control Ready")
        print("-----------------")
        print("Enter a number 0-100 to set fan speed %")
        print("Enter 'q' to quit")
        
        current_dc = 0
        
        while True:
            try:
                user_input = input(f"\nDuty Cycle [{current_dc}%]: ").strip()
            except EOFError:
                break
                
            if user_input.lower() == 'q':
                break
            
            if not user_input:
                continue
                
            try:
                duty_cycle = float(user_input)
                if 0 <= duty_cycle <= 100:
                    fan.ChangeDutyCycle(duty_cycle)
                    current_dc = duty_cycle
                    print(f"-> Speed set to {duty_cycle}%")
                else:
                    print("-> Error: Please enter a value between 0 and 100")
            except ValueError:
                print("-> Error: Invalid number")
                
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
        if "No access to /dev/mem" in str(e):
            print("Try running with sudo")
    finally:
        print("Cleaning up GPIO...")
        try:
            if 'fan' in locals():
                fan.stop()
            GPIO.cleanup()
        except Exception:
            pass
        print("Done")

if __name__ == "__main__":
    main()

