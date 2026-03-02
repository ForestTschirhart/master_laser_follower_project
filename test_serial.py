import serial
from fastapi import FastAPI, HTTPException, Body, Query
from fastapi.responses import ORJSONResponse, JSONResponse
import threading
import numpy as np
import matplotlib.pyplot as plt
import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)  # Suppresses the warning
GPIO.setmode(GPIO.BOARD)

MOD_FAILURE_PIN = 29
MOD_ACTIVE_PIN = 31
MOD_EN_PIN = 33
LOCK_STATE_PIN = 35
PEAKS_LOST_PIN = 38 



class ArduinoMonitor:
    def __init__(self, serial_port, baud,
                 push_fbk_en_pin, push_digilock_status_pin,
                 pull_fbk_active_pin, pull_fbk_failure_pin, pull_peaks_lost_pin):
        
        self.ser = serial.Serial(serial_port, baud, timeout=1)
        
        self.push_fbk_en_pin = push_fbk_en_pin
        self.push_digilock_status_pin = push_digilock_status_pin
        self.pull_fbk_active_pin = pull_fbk_active_pin
        self.pull_fbk_failure_pin = pull_fbk_failure_pin
        self.pull_peaks_lost_pin = pull_peaks_lost_pin
        GPIO.setup([push_fbk_en_pin, push_digilock_status_pin], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([pull_fbk_active_pin, pull_fbk_failure_pin, pull_peaks_lost_pin], GPIO.IN)
        
    def push_flag(self, pin, hilo):
        GPIO.output(pin, hilo)
        
    def pull_flag(self, pin):
        return GPIO.input(pin) == GPIO.HIGH
    
    #def pull_scope(self):
       # self.ser.write()


if __name__ == '__main__':
    mon = ArduinoMonitor('/dev/ttyACM0', 250000,
                             MOD_EN_PIN, LOCK_STATE_PIN,
                             MOD_ACTIVE_PIN, MOD_FAILURE_PIN, PEAKS_LOST_PIN)   
    
    time.sleep(4) # init proceedure
    mon.ser.write(b'Hello Due\n')
    line = mon.ser.readline().decode('utf-8').rstrip()
    print(f"Due says: {line}")
    line = mon.ser.readline().decode('utf-8').rstrip()
    print(line)
    time.sleep(2)
    
    
    init_state = mon.pull_flag(mon.pull_fbk_active_pin)
    print(init_state)
    mon.push_flag(mon.push_fbk_en_pin, True)
    time.sleep(2)
    final_state = mon.pull_flag(mon.pull_fbk_active_pin)
    print(final_state)
    
    while True:
        print(mon.pull_flag(mon.pull_fbk_active_pin), mon.pull_flag(mon.pull_peaks_lost_pin))
        time.sleep(0.5)
        

    
    