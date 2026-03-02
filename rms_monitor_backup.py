from digilock_remote import Digilock_UI
from fastapi import FastAPI, HTTPException, Body, Query
from fastapi.responses import ORJSONResponse, JSONResponse
import threading
import numpy as np
import matplotlib.pyplot as plt
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)


"""
Two parallel threads monitor the two lasers for signs of unlocking or if they are fully unlocked.
Each thread modifies global variables stored in the GLOB_* dictionaries.
These global variables can then be read out to the dashboard server using a third thread.

~~Coming soon~~
When unlocking behavior is sensed trigger an autocorrect loop that monitors rms and bumps the current until rms goes away.

"""

needs2werk = ['scope:ch2:rms', 'pid2:lock:state', 'pid1:input', 'pid1:output', 'pid1:lock:enable', 'pid1:setpoint', 'pid1:sign', 'pid1:slope']
types = [           'num',     		'bool', 		'enum (str)',  'enum (str)',       'bool',             'num',		'bool',   	'bool']
able =  [            'q',            'q',                'q,s',      'q,s',             'q,s',           'q,s', 		'q,s', 		'q,s']
wouldBcool = ['scope:graph']

RMS_THRESH_G = 10
RMS_THRESH_B = 80
WINDOW_LEN_G = 20
WINDOW_LEN_B = 20
F_SAMP = 2 #Hz

# State pins
MOD_FAILURE_PIN = 29
MOD_ACTIVE_PIN = 31
MOD_EN_PIN = 33
LOCK_STATE_PIN = 35


class DUIMonitor:
    def __init__(self, name, ip, port, f_samp, window_len, rms_thresh, fill_frac_thresh, trace_downsamp_factor=10):
        self.name = name
        print(name)
        self.dui = Digilock_UI(ip, port)
        print()
        
        self.locked = False
        self.lock_unhappy = False
        
        self.window_len = window_len
        self.rms_thresh = rms_thresh
        self.sum_thresh = int(window_len*fill_frac_thresh)
        self.running_window = np.zeros(window_len, dtype=bool) # for dynamic size edit outside of here
        
        self.f_samp = f_samp
        self.cur_ctrl_en = False
        self.cur_ctrl_active = False
        
        self.ch1 = None
        self.ch2 = None
        self.downsamp = int(trace_downsamp_factor)
        
        self.thread_lock = threading.Lock()
        
    def simple_monitor_loop(self):
        while True:
            try:              
                ti=time.perf_counter()
                rms = self.dui.query_numeric('scope:ch2:rms')
                with self.thread_lock:
                    self.running_window[1:] = self.running_window[:-1] #bitshift
                    self.running_window[0] = rms > self.rms_thresh # add new bit to front of sliding window 
                    self.lock_unhappy = np.sum(self.running_window) >= self.sum_thresh
                
                self.locked = self.dui.query_bool('pid2:lock:state')
                if self.lock_unhappy and self.locked and self.cur_ctrl_en:  # set up for current bump 
                    self.trigger_current_bump()
                
                ch1, ch2 = self.dui.query_graph('scope:graph') # this happens outside the lock so we dont block off large continuous section of time
                with self.thread_lock:
                    self.ch1, self.ch2 = ch1[::self.downsamp].tolist(), ch2[::self.downsamp].tolist() # this happens inside lock cause the self.attributes may be read out and need to be correlated
                
                tf=time.perf_counter()
                wait = (1/self.f_samp) - (tf-ti)
                if wait > 0:
                    time.sleep(wait)
                else:
                    print(f"LOOP LAGGING DESIRED F_SAMP by: {-1*wait} ms")
                
                tff = time.perf_counter()
                #print(f'ops: {tf-ti} s')
                #print(f'wait: {tff-tf} s')
                #print(f'tot: {tff-ti} s')
                #print(f'{self.name} rms_thresh: {self.sum_thresh}')
                
                
            except Exception as e:
                print(f"[{self.name} monitor error] {e}")
                time.sleep(2)
                
    
    def trigger_current_bump(self):
        # self.locked = self.dui.query_bool('pid2:lock:state') MAKE SURE WERE LOCKED BEFORE TRYING THIS CURRENT BUMP
        # safe loop inc = 0 
        #while safe loop inc < #:
            #bump current by (X)
            #loop to check if rms decreasing
                # roll and update running window
                # wait
            # sum window to see if below thresh
            # if not:
                #if we're still within output range
                    # set (X=bump inc) (loop and bump)
                # if not
                    # return current bump failed (deal with how to handle this in the main loop)       
            # if yes:
                # set current bump (X=0) (loop but dont bump next time)
                # set safe_loop_inc += 1
        print('current ctrl on')
        self.cur_ctrl_active = True
    
        
    def refresh_params(self, rms_thresh: float, window_len: int, fill_thresh: int):
        try:
            with self.thread_lock:
                self.rms_thresh = rms_thresh
                self.window_len = window_len
                self.sum_thresh = fill_thresh
                self.running_window = np.zeros(window_len, dtype=bool)
            #return [self.rms_thresh, self.window_len, fill_frac_thresh]
        except Exception as e:
            raise RuntimeError('Failed to Set Params') from e
        
app = FastAPI()

dui_blue = None # janky global variable fix for startup_event function variable scope issue
dui_green = None



@app.on_event("startup")
def startup_event():
    global dui_blue, dui_green
    # Create the DUIMonitor instances once at startup
    dui_blue = DUIMonitor('Blue', "192.168.10.3", 60001, F_SAMP, WINDOW_LEN_B, RMS_THRESH_B, fill_frac_thresh=0.75)
    dui_green = DUIMonitor('Green', "192.168.10.3", 60002, F_SAMP, WINDOW_LEN_G, RMS_THRESH_G, fill_frac_thresh=0.75)

    # Start their background monitor threads
    threading.Thread(target=dui_blue.simple_monitor_loop, daemon=True).start()
    threading.Thread(target=dui_green.simple_monitor_loop, daemon=True).start()
    
    #INIT pins proceedure
    GPIO.setup([MOD_EN_PIN, LOCK_STATE_PIN], GPIO.OUT, initial=GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output([MOD_EN_PIN, LOCK_STATE_PIN], GPIO.LOW)
    time.sleep(0.5)
    GPIO.output([MOD_EN_PIN, LOCK_STATE_PIN], GPIO.HIGH)   

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


@app.get("/scope_traces", response_class=ORJSONResponse)
def get_scopes():
    try:
        with dui_blue.thread_lock:
            b_ch1 = dui_blue.ch1
            b_ch2 = dui_blue.ch2

        with dui_green.thread_lock:
            g_ch1 = dui_green.ch1
            g_ch2 = dui_green.ch2
        return {
            "b_ch1": b_ch1,
            "b_ch2": b_ch2,
            "g_ch1": g_ch1,
            'g_ch2': g_ch2
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Scope Retrieval Failed: {e}")


@app.get("/monitor_states", response_class=ORJSONResponse)
def get_lock_states():
    try:
        return {
            "b_locked": bool(dui_blue.locked),
            "b_lock_unhappy": bool(dui_blue.lock_unhappy),
            "g_locked": bool(dui_green.locked),
            "g_lock_unhappy": bool(dui_green.lock_unhappy),
            "b_cur_ctrl_active": bool(dui_blue.cur_ctrl_active),
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Monitor State Retrieval Failed: {e}")


@app.get('/monitor_params')
def get_monitor_params(laser: str = Query(...)):
    try:
        if laser == 'green':
            device = dui_green
        elif laser=='blue':
            device = dui_blue
        else:
            raise ValueError('laser name invalid')
        rms_thresh = device.rms_thresh
        window_len = device.window_len
        sum_thresh = device.sum_thresh
        return {
            "rms threshold": rms_thresh,
            "window length": window_len,
            "sum threshold": sum_thresh
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f'Monitor param retrieval failed: {e}')

@app.post("/refresh_params")
def set_lock_params(params: dict = Body(...)):
    ''' params format: { name: str(green / blue)
                         window length: int
                         rms threshold: float
                         fill fraction threshold: float
                        }
    '''
    
    try:
        device = dui_green if params['name'] == "green" else dui_blue
        device.refresh_params(params['rms threshold'],
                              params['window length'],
                              params['fill fraction threshold'])
        
        #return {"status": "ok"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Parameter Refresh Failed: {e}")
   

if __name__ == '__main__':
    
    dui_blue = DUIMonitor('Blue', "192.168.10.3", 60001, F_SAMP, WINDOW_LEN_B, RMS_THRESH_B, fill_frac_thresh=0.75)
    dui_green = DUIMonitor('Green', "192.168.10.3", 60002, F_SAMP, WINDOW_LEN_G, RMS_THRESH_G, fill_frac_thresh=0.75)

    print(dui_blue.dui.query_numeric('scope:ch2:rms'))
    print(dui_green.dui.query_numeric('scope:ch2:rms'))
    
    thread_blue = threading.Thread(target=dui_blue.simple_monitor_loop, daemon=True)
    thread_green = threading.Thread(target=dui_green.simple_monitor_loop, daemon=True)

    thread_green.start()
    thread_blue.start()

