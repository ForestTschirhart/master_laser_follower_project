import serial
from digilock_remote import Digilock_UI
from fastapi import FastAPI, HTTPException, Body, Query
from fastapi.responses import ORJSONResponse, JSONResponse
import threading
import numpy as np
import time
import RPi.GPIO as GPIO
import requests

GPIO.setwarnings(False)  # Suppresses the warning
GPIO.setmode(GPIO.BOARD)

"""
Two threads monitor the two lasers through through the digilockRCI for signs of unlocking or if they are fully unlocked.
Another thread monitors the arduino pin logic flags.
Each thread modifies global variables stored in the GLOB_DICT in case the threads need to share blocking info later for feedback

When any of the monitor loops senses trouble they PUSH to the dash server, making sure that the logging functionality catches changes of state

Additionally, the two rci monitors and the arduino can be queried for scope traces and have updated feedback parameters pushed to them

"""

# defualt init vars for the rms monitors

RMS_THRESH_G = 100
RMS_THRESH_B = 80
WINDOW_LEN_G = 20
WINDOW_LEN_B = 20
F_SAMP = 2 #Hz for the RMS loops

# GPIO State pins for the arduino connection
PEAKS_LOST_PIN = 38 
MOD_FAILURE_PIN = 29
MOD_ACTIVE_PIN = 31
MOD_EN_PIN = 33
LOCK_STATE_PIN = 35

ARD_INIT_STATS_TO = 20 # sec
N = 500 # arduino trace length


glob_dict_lock = threading.Lock() # for different classes modifying the same dict 
tcp_lock = threading.Lock() # for interfacing with the digilock RCI over using tcp
serial_lock = threading.Lock() # for communitication over serial with the Arduino from in/out of a thread
push_2dash_lock = threading.Lock() # for PUSHING to the specific dash endpoint from inside a thread

# shared state variables 
GLOB_DICT = {        'blue happy': None,
                     'blue locked': None,
                     'blue rms': 0.0,
                     'blue mean': 0.0,
                     'green happy': None,
                     'green locked': None,
                     'green rms': 0.0,
                     'green mean': 0.0,
                     'ard peaks happy': None,
                     'ard fbk active' : None,
                     'ard fbk failed': None,
                     'ard peaks mean': 0.0,
                     'ard peaks stdev': 0.0,
                     'ard dac': 0
                    }

app = FastAPI() # pi's server
DASH_URL = "http://10.155.94.105:8050" # dash's server




### ---------------------------------------- RMS Monitor----------------------------------------- ###

class DUIMonitor:
    def __init__(self, name, ip, port, f_samp, window_len, rms_avg_thresh, trace_downsamp_factor=10):
        self.name = name
        print(name)
        self.dui = Digilock_UI(ip, port)
        print()
        
        self.locked = False 
        self.lock_happy = False
        self.prev_state = [self.locked, self.lock_happy]
        
        self.std = None
        self.mean = None
        
        self.window_len = window_len
        self.rms_avg_thresh = rms_avg_thresh
        self.running_window = np.zeros(window_len, dtype=float) # for dynamic size edit outside of here
        
        self.f_samp = f_samp
        self.cur_ctrl_en = False
        self.cur_ctrl_active = False
        
        self.downsamp = int(trace_downsamp_factor)
        
        self.thread_lock = threading.Lock() # for modifying class attributes non-atomically from outside/inside the monitor loop thread
        
    def simple_monitor_loop(self):
        while True:
            try:              
                ti=time.perf_counter()
                
                with tcp_lock:
                    self.rms = self.dui.query_numeric('scope:ch2:rms')
                    self.mean = self.dui.query_numeric('scope:ch1:mean')
                    self.locked = self.dui.query_bool('pid2:lock:state')
                
                with self.thread_lock:
                    self.running_window[1:] = self.running_window[:-1] #bitshift
                    self.running_window[0] = self.rms # add new rms to front of sliding window 
                    self.lock_happy = np.mean(self.running_window) <= self.rms_avg_thresh
                
                with glob_dict_lock:
                    GLOB_DICT[f"{self.name} happy"] = bool(self.lock_happy)
                    GLOB_DICT[f"{self.name} locked"] = bool(self.locked)
                    GLOB_DICT[f"{self.name} rms"] = self.rms
                    GLOB_DICT[f"{self.name} mean"] = self.mean
                
                # just for rms controlled feedback, now maybe defunct
                if not self.lock_happy and self.locked and self.cur_ctrl_en:  # set up for current bump 
                    self.trigger_current_bump()
                
                # for state change push (logging)
                new_state = [self.locked, self.lock_happy]
                if (new_state != self.prev_state):
                    self.prev_state = new_state
                    get_state_push_dash() # to be modded later
                                
                tf=time.perf_counter()
                wait = (1/self.f_samp) - (tf-ti)
                if wait > 0:
                    time.sleep(wait)
                else:
                    print(f"{self.name} DIGILOCK LOOP LAGGING DESIRED F_SAMP by: {-1*wait} s")
        
            except Exception as e:
                print(f"[{self.name} monitor loop error] {e}")
                time.sleep(2)
                
    
    def trigger_current_bump(self):
        print('current ctrl on')
        self.cur_ctrl_active = True
    
        
    def refresh_params(self, rms_avg_thresh: float, window_len: int):
        try:
            with self.thread_lock:
                self.rms_avg_thresh = rms_avg_thresh
                self.window_len = window_len
                self.running_window = np.zeros(window_len, dtype=float) # this is the one  that needs a lock (not atomic)
            #return [self.rms_thresh, self.window_len, fill_frac_thresh]
        except Exception as e:
            raise RuntimeError('Failed to Set Params') from e
    
    def get_traces(self):
        try:
            with tcp_lock:
                ch1, ch2 = self.dui.query_graph('scope:graph')   
            ch1, ch2 = ch1[::self.downsamp].tolist(), ch2[::self.downsamp].tolist()
            return ch1, ch2
        except Exception as e:
            print(f'{self.name} digilock trace fetch error: {e}')
            raise RuntimeError(f'{self.name} digilock trace fetch error') from e


@app.get("/digi_scope_traces", response_class=ORJSONResponse)
def get_scopes():
    try:
        b_ch1, b_ch2 = dui_blue.get_traces()
        g_ch1, g_ch2 = dui_green.get_traces()
        return {
            "b_ch1": b_ch1,
            "b_ch2": b_ch2,
            "g_ch1": g_ch1,
            'g_ch2': g_ch2
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Scope Retrieval Failed: {e}")
    

@app.post('/set_cur_ctrl')
def set_cur_ctrl(setting: dict = Body(...)):
    try:
        if setting['laser'] == 'blue':
            dui_blue.cur_ctrl_en = setting['value']
        elif setting['laser'] == 'green':
            dui_green.cur_ctrl_en = setting['value']
        else:
            raise ValueError('laser name invalid')
    except Exception as e:
        return HTTPException(status_code=500, detail=f'Current ctrl set failed: {e}')



@app.get('/init_monitor_params') 
def init_digi_monitor_params(laser: str = Query(...)):
    try:
        if laser == 'green':
            device = dui_green
        elif laser=='blue':
            device = dui_blue
        else:
            raise ValueError('laser name invalid')
        rms_avg_thresh = device.rms_avg_thresh
        window_len = device.window_len
        return {
            "rms avg threshold": rms_avg_thresh,
            "window length": window_len
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Monitor param retrieval failed: {e}')


@app.post("/refresh_params")
def set_digi_lock_params(params: dict = Body(...)):
    ''' params format: { name: str(green / blue)
                         window length: int
                         rms threshold: float
                         fill fraction threshold: float
                        }
    '''
    
    try:
        device = dui_green if params['name'] == "green" else dui_blue
        device.refresh_params(params['rms avg threshold'],
                              params['window length'])
        
        #return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Parameter Refresh Failed: {e}")




### -------------------------------------- Arduino Stuff ----------------------------------- ###        
        
class ArduinoMonitor:
    def __init__(self, serial_port, baud, gpio_samp_rate,
                 push_fbk_en_pin, push_digilock_status_pin,
                 pull_fbk_active_pin, pull_fbk_failure_pin, pull_peaks_lost_pin):
        
        try: 
            self.ser = serial.Serial(serial_port, baud, timeout=2)
            print("Arduino Connected!")
            print("---------------------------")
            print()
            time.sleep(0.5)  # Let serial port establish
            self.ser.reset_input_buffer()  # Clear arduino first time loop initialization message
                
        except Exception as e:
            print("Failed to open arduino serial port:", e)       
        
        self.push_fbk_en_pin = push_fbk_en_pin
        self.push_digilock_status_pin = push_digilock_status_pin
        self.pull_fbk_active_pin = pull_fbk_active_pin
        self.pull_fbk_failure_pin = pull_fbk_failure_pin
        self.pull_peaks_lost_pin = pull_peaks_lost_pin
        GPIO.setup([push_fbk_en_pin, push_digilock_status_pin], GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup([pull_fbk_active_pin, pull_fbk_failure_pin, pull_peaks_lost_pin], GPIO.IN)
        
        # get params from arduino when pi script boots
        
        self.trigger_holdoff = None
        self.samp_ct = None
        self.dac_start_val = None
        self.dac_min_val = None
        self.long_mem_n_stdev = None
        self.short_mem_n_stdev = None
        self.short_mem_len = None
        self.pk_fnd_thr = None
        self.dac_step = None
        
        self.query_params() # initializes params in group just above^
        
        self.digilock_state = False
        self.fbk_en = False
        self.peaks_happy = None        
        self.fbk_active = None
        self.fbk_failed = None
        self.prev_state = [self.peaks_happy, self.fbk_active, self.fbk_failed]        
        self.gpio_samp_rate = gpio_samp_rate
        
        
        self.pks_mean = None
        self.pks_std = None
        self.dac_lvl = None
        
        
    def gpio_mon_loop(self):
        while True:
            try:              
                ti=time.perf_counter()
                
                self.peaks_happy = not self.pull_flag(self.pull_peaks_lost_pin)
                self.fbk_active = self.pull_flag(self.pull_fbk_active_pin)
                self.fbk_failed = self.pull_flag(self.pull_fbk_failure_pin)
                new_state = [self.peaks_happy, self.fbk_active, self.fbk_failed]
                
                #print(f"ARD GPIO (happy, active, failed): {self.peaks_happy}, {self.fbk_active}, {self.fbk_failed}")
                
                #print(type(self.fbk_active), self.fbk_active)
                
                
                with glob_dict_lock:
                    GLOB_DICT['ard peaks happy'] = bool(self.peaks_happy)
                    GLOB_DICT['ard fbk active'] = bool(self.fbk_active)
                    GLOB_DICT['ard fbk failed'] = bool(self.fbk_failed)
                    self.digilock_state = bool(GLOB_DICT['blue locked'])
                
                #give ard lock status pin from blue digilock pi and fbk enable from dash
                self.push_flag(self.push_digilock_status_pin, self.digilock_state)
                self.push_flag(self.push_fbk_en_pin, self.fbk_en)
                
                if new_state != self.prev_state:
                    self.prev_state = new_state
                    get_state_push_dash()
                
                tf=time.perf_counter()
                wait = (1/self.gpio_samp_rate) - (tf-ti)
                if wait > 0:
                    time.sleep(wait)
                else:
                    print(f"ARDUINO GPIO MONITOR LOOP LAGGING DESIRED SAMP RATE by: {-1*wait} s")
                    
            except Exception as e:
                print(f"[Arduino gpio monitor loop error] {e}")
                time.sleep(2)
        
        
    def push_flag(self, pin, hilo):
        GPIO.output(pin, hilo)
        
        
    def pull_flag(self, pin):
        return GPIO.input(pin) == GPIO.HIGH
    
    
    def get_trace(self, N=500): 
        try: 
            with serial_lock:
                self.ser.write(b'T\n')
                self.ser.flush()        
                time.sleep(0.01)
                total_bytes = self.ser.in_waiting
                raw = self.ser.read(N*2) # times 2 is because serial works with 8 bit bytes but we are trying to send over N 16 bit floats
            
            print(f"Get Trace: Arduino sent {total_bytes} bytes")
            return np.frombuffer(raw, dtype='<u2').astype(np.int32)
        except Exception as e:
            print('Arduino trace fetch failed: {e}')
            raise RuntimeError('Failed to query arduino log info') from e 
    
    
    def get_ard_log_info(self):
        try:  
            with serial_lock:
                self.ser.write(b'L\n')
                self.ser.flush()       
                resp = self.ser.readline()
            print(f"Arduino log info: {resp}")
            resp = resp.decode('ascii').rstrip().split(",")
            self.pks_mean = float(resp[0])
            self.pks_std = float(resp[1])
            self.dac_lvl = int(resp[2])
            
            with glob_dict_lock:
                GLOB_DICT['ard peaks mean'] = self.pks_mean
                GLOB_DICT['ard peaks stdev'] = self.pks_std
                GLOB_DICT['ard dac'] = self.dac_lvl                
            
        except Exception as e:
            print('Failed to query arduino log info')
            raise RuntimeError('Failed to query arduino log info') from e        
    
    
    def query_params(self):
        try:
            with serial_lock:
                self.ser.write(b'P\n')
                self.ser.flush()
                resp = self.ser.readline()
            resp = resp.decode('ascii').rstrip().split(",")
            self.trigger_holdoff = int(resp[0])
            self.samp_ct = int(resp[1])
            self.dac_start_val = int(resp[2])
            self.dac_min_val = int(resp[3])
            self.long_mem_n_stdev = float(resp[4])
            self.short_mem_n_stdev = float(resp[5])
            self.short_mem_len = int(resp[6])
            self.pk_fnd_thr = int(resp[7])
            self.dac_step = int(resp[8])
            
        except Exception as e:
            print(f'Failed to query arduino params {e}')
            raise RuntimeError(f'Failed to query arduino params {e}') from e
    
    
    def refresh_params(self, trigger_holdoff:int, samp_ct:int, dac_start:int, dac_min:int,
                       long_mem_n_stdev:float, short_mem_n_stdev:float, short_mem_len:int,
                       pk_fnd_thr:int, dac_step:int):
        try:
            with serial_lock:
                self.ser.write((f'R{trigger_holdoff},{samp_ct},{dac_start},{dac_min},{long_mem_n_stdev},'
                                f'{short_mem_n_stdev},{short_mem_len},{pk_fnd_thr},{dac_step}\n').encode('ascii'))
                self.ser.flush()
                resp = self.ser.readline()
            resp = resp.decode('ascii').rstrip().split(",")
            if int(resp[0]) == 1:
                self.dac_start_val = dac_start # only change if ard said it was valid
            if int(resp[1]) == 1:
                self.dac_min_val = dac_min
            if int(resp[2]) == 1:
                self.dac_step = dac_step
            
            self.trigger_holdoff = trigger_holdoff
            self.samp_ct = samp_ct
            self.long_mem_n_stdev = long_mem_n_stdev
            self.short_mem_n_stdev = short_mem_n_stdev
            self.short_mem_len = short_mem_len
            self.pk_fnd_thr = pk_fnd_thr
            
            return resp
        except Exception as e:
            print(f'Failed to Set Arduino Params: {e}')
            raise RuntimeError(f'Failed to Set Arduino Params: {e}') from e
    
    
    def init_stats(self):
        try:
            with serial_lock:
                self.ser.write(b'I\n')
                self.ser.flush()
                start = time.time()
                while self.ser.in_waiting == 0:
                    if time.time() - start > ARD_INIT_STATS_TO:
                        raise RuntimeError(f"Timeout {ARD_INIT_STATS_TO} sec waiting for serial data")
                    time.sleep(0.1)
                resp = self.ser.readline()
            resp = resp.decode('ascii').rstrip().split(",")
            return float(resp[0]), float(resp[1])
        except Exception as e:
            print(f'Failed to init arduino stats {e}')
            raise RuntimeError('Failed to init arduino stats') from e
        
        
    def reset_feedback(self, full:int):
        try:
            self.fbk_en = False # disable feedback, arduino manually does this but push to pin just in case
            self.push_flag(self.push_fbk_en_pin, self.fbk_en)
            with serial_lock:
                self.ser.write(f'F{full}\n'.encode('ascii'))  # if 0 just resets failure flag, if 1 resets flag AND the output lvl
                self.ser.flush()
            print(f"Reset Ard Feedback")
                
        except Exception as e:
            print(f'Failed to Reset Arduino Feedback: {e}')
            raise RuntimeError(f'Failed to Reset Arduino Feedback: {e}') from e


@app.get("/arduino_trace", response_class=ORJSONResponse)
def get_arduino_trace():
    try:
        trace = ard_mon.get_trace()
        return {
            "trace": trace.tolist(),
            "meta" : 0
        }
    except Exception as e:
        print(f"Arduino Trace Retrieval Failed: {e}")
        raise HTTPException(status_code=500, detail=f"Arduino Trace Retrieval Failed: {e}")


@app.post("/reset_ard_fbk")
def reset_ard_feedback(setting: int = Body(...)):
    try:
        ard_mon.reset_feedback(setting) # if 0 just resets failure flag, if 1 resets flag AND the output lvl
    except Exception as e:
        print(f"Arduino Feedback Reset Failed: {e}")
        raise HTTPException(status_code=500, detail=f"Arduino Feedback Reset Failed: {e}")


@app.post('/set_ard_fbk')
def set_ard_fbk_en(setting: bool = Body(...)):
    try:
        ard_mon.fbk_en = setting
        print(f'{setting}={type(setting)}')
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Arduino FB set failed: {e}')


@app.get('/get_ard_fbk_en')
def init_arduino_params():
    try:
        return {"fbk enabled": bool(ard_mon.fbk_en)}
    except Exception as e:
        print(f'Arduino FB EN state query failed: {e}')
        raise HTTPException(status_code=500, detail=f'Arduino FB EN state query failed: {e}')


@app.get('/init_arduino_params')
def init_arduino_params():
    try:
        ard_mon.query_params() # refreshes the ard_mon object's knowledge of all arduino onboard params        
        return {
            "trigger delay": ard_mon.trigger_holdoff,
            "samp count" : ard_mon.samp_ct,
            "dac start val" : ard_mon.dac_start_val,
            "dac min val" : ard_mon.dac_min_val,
            "long memory N std thresh" : ard_mon.long_mem_n_stdev,
            "short memory N std thresh" : ard_mon.short_mem_n_stdev,
            "short memory length" : ard_mon.short_mem_len,
            "peakfind thresh" : ard_mon.pk_fnd_thr,
            "dac step": ard_mon.dac_step,
            
        }
    except Exception as e:
        print(f'Arduino param retrieval failed: {e}')
        raise HTTPException(status_code=500, detail=f'Arduino param retrieval failed: {e}')
   

@app.post("/refresh_arduino_params")
def set_ard_params(params: dict = Body(...)):    
    try:
        resp = ard_mon.refresh_params(params['trigger delay'],
                               params['samp count'],
                               params["dac start"],
                               params["dac min"],
                               params["long memory N std thresh"],
                               params["short memory N std thresh"],
                               params["short memory length"],
                               params["peakfind thresh"],
                               params["dac step"],
                               )
        return {"dac start status": int(resp[0]),
                "dac min status": int(resp[1]),
                "dac step status": int(resp[2]),
                "dac abs min": int(resp[3]),
                "dac abs max": int(resp[4]),
                "dac abs max step": int(resp[5]),
                }
    except Exception as e:
        print(f"Arduino Parameter Refresh Failed: {e}")
        raise HTTPException(status_code=500, detail=f"Arduino Parameter Refresh Failed: {e}")


@app.get("/initialize_arduino_stats")
def initialize_arduino_stats():
    try:
        init_height, init_std = ard_mon.init_stats() # refreshes the ard_mon object's knowledge of all arduino onboard params
        return {"init height": init_height,
                "init std": init_std}
    except Exception as e:
        print(f'Arduino init stats failed: {e}')
        raise HTTPException(status_code=500, detail=f'Arduino init stats failed: {e}')
    



### ------------------------------------------- Shared Structures ------------------------------------ ###


dui_blue = None # janky global variable fix for startup_event function variable scope issue
dui_green = None
ard_mon = None


def get_state_push_dash():
    try:
        #ard peaks and dac query
        ard_mon.get_ard_log_info()
        
        with push_2dash_lock:
            requests.post(f"{DASH_URL}/api/master_pi_state_report", json=GLOB_DICT, timeout=1) # don't need a dict lock here cause reads are safe, also avoids potential deadlock bug
        
        print(f'pushed to dash!')
    except Exception as e:
        print(f'frik >:(  {e}' )
        

@app.on_event("startup")
def startup_event():
    global dui_blue, dui_green, ard_mon
    # Create the DUIMonitor instances once at startup
    dui_blue = DUIMonitor('blue', "192.168.10.3", 60001, F_SAMP, WINDOW_LEN_B, RMS_THRESH_B)
    dui_green = DUIMonitor('green', "192.168.10.3", 60002, F_SAMP, WINDOW_LEN_G, RMS_THRESH_G)
    try:
        ard_mon = ArduinoMonitor('/dev/ttyACM0', 250000, 8,
                                 MOD_EN_PIN, LOCK_STATE_PIN,
                                 MOD_ACTIVE_PIN, MOD_FAILURE_PIN, PEAKS_LOST_PIN)
    except Exception as e1:
        try:
            print(f"Failed to connect to Arduino using port ttyACM0: {e1}")
            print()
            print("Trying ttyACM1...")
            ard_mon = ArduinoMonitor('/dev/ttyACM1', 250000, 8,
                                 MOD_EN_PIN, LOCK_STATE_PIN,
                                 MOD_ACTIVE_PIN, MOD_FAILURE_PIN, PEAKS_LOST_PIN)
        except Exception as e2:
            
            print()
            print(f"Failed to connect to Arduino using port ttyACM1: {e2}")
    
    # Start their background monitor threads
    threading.Thread(target=dui_blue.simple_monitor_loop, daemon=True).start()
    threading.Thread(target=dui_green.simple_monitor_loop, daemon=True).start()
    threading.Thread(target=ard_mon.gpio_mon_loop, daemon=True).start()
    time.sleep(5)
    get_state_push_dash()


@app.post("/trigger_state_report")
def trigger_state_report():
    try:
        get_state_push_dash()
    except Exception as e:
        print(f'Trigger_state_report failed: {e}')
        raise HTTPException(status_code=500, detail=f'Trigger_state_report failed: {e}')
    



if __name__ == '__main__':
    
    dui_blue = DUIMonitor('Blue', "192.168.10.3", 60001, F_SAMP, WINDOW_LEN_B, RMS_THRESH_B)
    dui_green = DUIMonitor('Green', "192.168.10.3", 60002, F_SAMP, WINDOW_LEN_G, RMS_THRESH_G)
    ard_mon = ArduinoMonitor('/dev/ttyACM0', 250000,
                             MOD_EN_PIN, LOCK_STATE_PIN,
                             MOD_ACTIVE_PIN, MOD_FAILURE_PIN, PEAKS_LOST_PIN) 
    
    print(dui_blue.dui.query_numeric('scope:ch2:rms'))
    print(dui_green.dui.query_numeric('scope:ch2:rms'))
    
    thread_blue = threading.Thread(target=dui_blue.simple_monitor_loop, daemon=True)
    thread_green = threading.Thread(target=dui_green.simple_monitor_loop, daemon=True)

    thread_green.start()
    thread_blue.start()
    
    trace = ard_mon.get_trace()
    print(len(trace))
    

