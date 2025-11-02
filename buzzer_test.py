#!/usr/bin/env python3
from hw.buzzer import Buzzer

buzzer = Buzzer()
buzzer.beep_pattern(count=10, duration=0.2, pause=0.1)
buzzer.beep_pattern(count=10, duration=1.0, pause=0.1)
buzzer.cleanup()
