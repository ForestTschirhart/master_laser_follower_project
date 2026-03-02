/*

 Fabry-Perot Scan Asynchronous Scope Data Acquisition using peripherals and timer counter module
 + Feedback Control + Communication with RaspberryPi and by extension Dash server 

 By Forest Tschirhart 2/1/2026, 
 foresttschirhart@gmail.com
 :3

*/

#include <math.h>
#include <ctype.h>
#include <Arduino.h>

//GPIO Pin IDs
#define ANALOG_PIN A0      // Fabry Perot input
#define TRIGGER_PIN 13     // Trigger input
#define MOD_DAC DAC0       // Current modulation out 

#define DIGILOCK_STATUS 30  // Digital lock status
#define MOD_ENABLE 28       // Feedback enable
#define MOD_ACTIVE 26       // Feedback active status
#define MOD_FAILURE 24      // Feedback failure status
#define PEAKS_LOST 32       // for logging purposes

// Other constants
#define N 500                       // Samples per scan
#define NUM_INIT_PEAKS 400          // 5s average 40 Hz * 2 peaks per scan
#define MAX_INIT_SCANS 700          // tries for stats initialization procedure 
#define PEAKS_PER_SCAN 2   
#define RUNNING_BUFFER_SIZE 800     // at 40Hz and 2 samples per scan this is 10 seconds of data
#define ABS_MIN_DAC_LVL 0           // hard limits on the adjustable vals
#define ABS_MAX_DAC_LVL 4000
#define ABS_MAX_DAC_STEP 100
#define MAX_CMD_ARGS 15             // may change in the future as more params become adjustable

// State variables with corresponding status pins // 
bool digilock_status_flag = false;
bool mod_enable_flag = false;
bool mod_failure_flag = false;
bool mod_active_flag = false;
bool peakfinder_success_flag = false;     // this one goes into peakslost pin
bool longterm_peakslost_flag = false;  

// Data buffers and vars related to the asynch scope function // 
static volatile uint16_t pdc_buf0[N];     // just for PDC writing
static volatile uint16_t pdc_buf1[N];     // alternating buffers so never busy waiting for PDC
static volatile uint16_t pdc_buf2[N];     // necessary to avoid serial delay failure case
volatile int pdc_last_buf = 1;            // which buffer was last filled by PDC
volatile bool fresh_data = false;         // flag set by ISR when new data ready

// Misc variables related to the async scope operation //
volatile int trig_delay = 20;     // Number of sample timesteps to delay by. Samp count has to be half period, 
                                  // so this needs to be multiplied by 2 every time used
volatile int adc_samp_ct = 52;    // this sets the sample timestep

// Data buffers and vars used by the main loop (Dont think volatile necessary but not hurting anthing rn) //
volatile uint16_t buffer[N];                    // raw data buffer accessible in the main loop 
volatile uint16_t tempPeaks[PEAKS_PER_SCAN];    // stores peaks from a single scan
volatile int tempPeakNo = 0;                    // used in a bunch of different funcs, keeps track of no. peaks found in a scan
int peakfind_thresh = 1000;                     // for the peakfinding algorithm

// Initialization variables //
int foundInitPeakNo = 0;
uint16_t initPeaks[NUM_INIT_PEAKS];
bool initialized = false;

// Feedback parameters //
int shortMem = 40;                  // no traces between each feedback iteration, this default is ~ a second
int countdown = 0;                  // gets set and decremented, controls feedback algorithm flow
float long_mem_n_stdev = 1.0;       // no stdevs below init height to trigger peak lost
float short_mem_n_stdev = 1.0;      // no stdevs below init height to trigger differnt feedback actions
int bump_count = 0;                 // keeps track of no. dac bump attempts
int max_bumps = 100;                // max dac bump attempts before giving up 
int min_daclvl = 10;                // minimum acceptable dac lvl before stopping feedback
int dac_reset_lvl = 2000;           // inital and reset dac lvl, can be changed later from dash
int daclvl = dac_reset_lvl;         // current dac level
int dac_step = 100;                 // step size for dac lvl bumps


// ---------- Peak statistics structure ----------
struct PeakStats {
 float meanHeight;
 float stdHeight;
};

// Global stats //
PeakStats initPeakStats;
float initHeight;
float initStd;


// ------------ Running Buffer struct works ~ as integrator for feedback control ------------- //
struct RunningBuffer {
  uint16_t* data;    // Pointer to dynamically allocated buffer
  int size;          // Max number of samples
  int head;          // Next index to write
  int count;         // Current number of elements
  float mean;        // Running mean
  float M2;          // Sum of squared differences

  RunningBuffer(int n) : size(n), head(0), count(0), mean(0.0f), M2(0.0f) {
    data = new uint16_t[n];
  }

  ~RunningBuffer() {
    delete[] data;
  }

  void push(uint16_t value) {     // when fresh peaks are found, can push them into the buffer and update stats
    data[head] = value;
    head = (head + 1) % size;
    if (count < size) count++;

    // Recompute mean and M2
    mean = 0.0f;
    for (int i = 0; i < count; i++) mean += get(i);
    mean /= count;

    M2 = 0.0f;
    for (int i = 0; i < count; i++) {
      float delta = (float)get(i) - mean;
      M2 += delta * delta;
    }
  }

  uint16_t get(int i) const {
    if (i >= count) return 0;
    int index = (head - count + i + size) % size;
    return data[index];
  }

  uint16_t latest() const {
    if (count == 0) return 0;
    int idx = (head - 1 + size) % size;
    return data[idx];
  }

  float getMean() const {
    return mean;
  }

  float getStd() const {
    if (count < 2) return 0.0f;
    return sqrt(M2 / (count - 1));
  }

  float getShortMean(int n) const {     // get the mean for a desired length of running buffer memory
    if (n > count) n = count;
    float shortMean = 0.0f;
    for (int i = count - n; i < count; i++) {
      shortMean += get(i);
    }
    shortMean /= n;
    return shortMean;
  }
};

RunningBuffer runningBuffer(RUNNING_BUFFER_SIZE); // init


// ----------------------- SETUP and LOOP ------------------------- //
void setup() {
  SerialUSB.begin(250000);              // Use the Programming port (SerialUSB) so the device matches /dev/ttyACM1 or ACM0
  pinModeSetup();
  scopeSetup();
  analogWriteResolution(12);
  analogWrite(MOD_DAC, dac_reset_lvl); 

  while (!SerialUSB);                   // wait for pi connection before moving to LOOP
  SerialUSB.println("=== Due USB Connected, Loop Starting ===");  
}


void loop() {
  // Check that parallel DAQ loop has updated, only process data, update state, run feedback if so //
  if (fresh_data) { 
    // Process data //                    
    acquireScan();
    findPeaks();
    if (peakfinder_success_flag){       // make sure findPeaks() found peaks
      for (int i = 0; i < tempPeakNo; i++) {
        runningBuffer.push(tempPeaks[i]); 
      }
    }

    // State update and Feedback //
    if (statusWrapper()) {              // checks blocking pins and diable conditions, also updates peak lost info 
      feedbackWrapper();  
    }
  }
  
  // HANDLE SerialUSB COMMANDS //
  char cmd = '\0';
  float args[MAX_CMD_ARGS];
  if (readCommandLetterNumbers(cmd, args)) { 

    if (cmd == 'T') {                                             // T for Trace
      SerialUSB.write((uint8_t*)buffer, N * sizeof(uint16_t));
      SerialUSB.flush();

    } else if (cmd == 'R') {                                      // R for Refresh params
      // scope params // 
      trig_delay = (int)args[0];
      adc_samp_ct = (int)args[1];
      TC_SetRC(TC0, 0, adc_samp_ct);                              // direct manip of TC reg for ADC sample clocking
      TC_SetRC(TC0, 2, 2 * trig_delay * adc_samp_ct);             // same for TC reg for trace delay
      
      // DAC lvl boundaries // 
      bool start_status = (ABS_MIN_DAC_LVL <= (int)args[2]) && ((int)args[2] <= ABS_MAX_DAC_LVL);
      if (start_status) {dac_reset_lvl = (int)args[2];}
      bool min_status = (ABS_MIN_DAC_LVL <= (int)args[3]) && ((int)args[3] <= ABS_MAX_DAC_LVL);
      if (min_status) {min_daclvl = (int)args[3];}

      // Feedback Params //
      long_mem_n_stdev = (float)args[4];
      short_mem_n_stdev = (float)args[5];
      shortMem = (int)args[6];
      peakfind_thresh = (int)args[7];

      bool step_status = ((int)args[8] <= ABS_MAX_DAC_STEP);
      if (step_status) {dac_step = (int)args[8];}

      // Respond to Pi // 
      SerialUSB.print(start_status);
      SerialUSB.print(",");
      SerialUSB.print(min_status);
      SerialUSB.print(",");
      SerialUSB.print(step_status);
      SerialUSB.print(",");
      SerialUSB.print(ABS_MIN_DAC_LVL);
      SerialUSB.print(",");
      SerialUSB.print(ABS_MAX_DAC_LVL);
      SerialUSB.print(",");
      SerialUSB.print(ABS_MAX_DAC_STEP);
      SerialUSB.println();

    } else if (cmd == 'P') {                        // P for get Params
      SerialUSB.print(trig_delay);
      SerialUSB.print(",");
      SerialUSB.print(adc_samp_ct);
      SerialUSB.print(",");
      SerialUSB.print(dac_reset_lvl);
      SerialUSB.print(",");
      SerialUSB.print(min_daclvl);
      SerialUSB.print(",");
      SerialUSB.print(long_mem_n_stdev);
      SerialUSB.print(",");
      SerialUSB.print(short_mem_n_stdev);
      SerialUSB.print(",");
      SerialUSB.print(shortMem);
      SerialUSB.print(",");
      SerialUSB.print(peakfind_thresh);
      SerialUSB.print(",");
      SerialUSB.print(dac_step);
      SerialUSB.println();  

    } else if (cmd == 'I') {                         // I for Initialize
      if (initialize_peak_vals_locations()) {
        SerialUSB.print(initHeight);                 // Respond to Pi with init stats info 
        SerialUSB.print(",");
        SerialUSB.println(initStd);
      } else {
        SerialUSB.print(0.0f);                       // if the initialization failed because no peaks found,
        SerialUSB.print(",");                        // then send over two zeros to get dash users attention
        SerialUSB.println(0.0f);
      }
      
    } else if (cmd == 'L') {                         // L for Logging info
      SerialUSB.print(runningBuffer.getMean(), 2);   // send current stats
      SerialUSB.print(",");
      SerialUSB.print(runningBuffer.getStd(), 2);
      SerialUSB.print(",");
      SerialUSB.println(daclvl);                     // send current dac lvl

    } else if (cmd == 'F') {                         // F for feedback reset 
      mod_enable_flag = false;                       // make sure dash and pi get the memo too
      mod_failure_flag = false; 
      mod_active_flag = false;
      fastDigWrite(MOD_ACTIVE, mod_active_flag);     // pi will see these and auto push state update to dash
      fastDigWrite(MOD_FAILURE, mod_failure_flag);

      if ((int)args[0]) {                            // if 0 then don't change output lvl, if 1 then reset the output
        gentleDacRamp(500, dac_reset_lvl);           
      }
      
      bump_count = 0; 
    }
  }
}



// ----------------------- SETUP HELPER FUNCS ----------------------- //

void pinModeSetup(){
  pinMode(ANALOG_PIN, INPUT);
  pinMode(TRIGGER_PIN, INPUT);
  pinMode(DIGILOCK_STATUS, INPUT);
  pinMode(MOD_ENABLE, INPUT);
  pinMode(MOD_ACTIVE, OUTPUT);
  pinMode(MOD_FAILURE, OUTPUT);
  pinMode(PEAKS_LOST, OUTPUT);

  digitalWrite(MOD_ACTIVE, LOW);
  digitalWrite(MOD_FAILURE, LOW);
  digitalWrite(PEAKS_LOST, LOW);
}


void scopeSetup() {
  // Set up simple trigger interrupt //
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), onTriggerRise, RISING); 

  // Clokcs need to be enabled for config //
  pmc_enable_periph_clk(ID_TC2);            // TC0 ch2 will be the trigger delay timer
  pmc_enable_periph_clk(ID_TC0);            // TC0 ch0 will be the ADC sampling timer
  pmc_enable_periph_clk(ID_ADC);            // enable ADC peripheral clock


  // Setup timer0 ch2 for trigger delay //
  TC_Configure(TC0, 2,                      // Use channel 2 of TC module 0
               TC_CMR_TCCLKS_TIMER_CLOCK2 | // Select clock 1-4 (MCK/2,8,32,128 or Slow Clock), MCK = 84MHz
               TC_CMR_WAVE |                // Waveform mode (not input mode)
               TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset
               TC_CMR_ACPC_TOGGLE |         // toggle TIOA on RC compare
               TC_CMR_CPCSTOP               // Counter stops on RC compare just in case high priority interrupt 
                                            // cuts off clean up behavior. idk if neccessary
              ); 

  TC_SetRC(TC0, 2, 2 * trig_delay * adc_samp_ct);      // set default trigger holdoff

  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;      // enable interrupt flag on counter reaching RC value
  NVIC_SetPriority(TC2_IRQn, 3);                // set priority of that^ interrupt. TC2_IRQn handles timer2 channel 0
  NVIC_EnableIRQ(TC2_IRQn);                     // enable interrupt using NVIC (will require handler function)


  // Within the timer0 block, route the output TIOA from ch2 to the trigger input of ch0 through the XC0 channel //
  TC0->TC_BMR &= ~TC_BMR_TC0XC0S_Msk;
  TC0->TC_BMR |= TC_BMR_TC0XC0S_TIOA2; // route TIOA2 to XC0


  // Setup timer0 ch0 for ADC sampling timing //
  TC_Configure(TC0, 0,                      
               TC_CMR_TCCLKS_TIMER_CLOCK2 |   // set to same as ADC clock for easier timing
               TC_CMR_WAVE |                
               TC_CMR_WAVSEL_UP_RC |  
               TC_CMR_EEVTEDG_EDGE |          // trigger count start on either edge
               TC_CMR_EEVT_XC0 |              // trigger event from TIOA through the XC0 channel (timer0 ch1 toggles TIOA)
               TC_CMR_ENETRG |                // enable external event trigger
               TC_CMR_ACPC_TOGGLE             // clock TIOA0 
              );

  TC_SetRC(TC0, 0, adc_samp_ct);              // set the sample timestep/rate
  

  // Setup ADC to trigger on timer0 ch0 TIOA output //
  ADC->ADC_CR = ADC_CR_SWRST;                            // reset ADC
  ADC->ADC_CHDR = 0xFFFFFFFF;                            // disable all channels
  ADC->ADC_CHER = ADC_CHER_CH7;                          // enable just channel 7 (A0)

  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_TRGSEL_Msk) | ADC_MR_TRGSEL_ADC_TRIG1; // trig1 is TIOA0 from TC0 ch0, set as trigger
  ADC->ADC_MR |= ADC_MR_TRGEN;                           // enable hardware trigger
  ADC->ADC_MR &= ~ADC_MR_SLEEP_SLEEP;                    // disable sleep mode
  ADC->ADC_MR = (ADC->ADC_MR & ~ADC_MR_STARTUP_Msk) | ADC_MR_STARTUP_SUT0;    // set startup time to 0
  ADC->ADC_MR &= ~ADC_MR_FREERUN_ON;                     // disable free run mode
  ADC->ADC_MR &= 0xFFFFFFEF;                             // set 12 bit resolution
  ADC->ADC_MR &= 0xFFCFFFFF;                             // set settling time to 3 ADC clocks
  ADC->ADC_MR &= 0xF0FFFFFF;                             // Tracking Time = (TRACKTIM + 1) * ADCClock periods: set to 1 ADC clock 
  ADC->ADC_MR &= 0xCFFFFFFF;                             // Transfer Period = (TRANSFER * 2 + 3) ADCClock periods: set to 3 ADC clocks

  ADC->ADC_EMR &= ~ADC_EMR_TAG;                          // disable channel number tagging in data register 

  // CAREFUL: make sure to set the trigger period greater than 13x adc clocks

  ADC->ADC_MR |= ADC_MR_PRESCAL(3);                      // set prescaler (ADC clock = MCK/(2*(PRESCAL+1)) 10.5 MHz
  ADC->ADC_IDR = ADC_IDR_EOC7;                           // disable ADC end of conversion interrupt since using PDC
  
  // Set up PDC to recieve ADC conversions automatically //
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;      // disable PDC transfers and recieves while setting up
  ADC->ADC_RPR = (uint32_t)pdc_buf0;                      // set receive pointer to allocated PDC buffer
  ADC->ADC_RCR = N;                                       // set receive counter to correct buffer size
  ADC->ADC_RNPR = 0;                                      // clear next-pointer (no chained buffer neccessary for this, 
  ADC->ADC_RNCR = 0;                                      // traces aren't that long)

  ADC->ADC_IER = ADC_IER_RXBUFF;                          // enable interrupt when buffer is full
  NVIC_SetPriority(ADC_IRQn, 3);
  NVIC_EnableIRQ(ADC_IRQn); 

  ADC->ADC_PTCR = ADC_PTCR_RXTEN;                         // enable PDC receive transfers
}



// -------------- SCOPE RELATED INTERRUPT SERVICE ROUTINES ---------------- //

// Handler for scope trigger //
void onTriggerRise() {
  TC_Start(TC0, 2);                          // enable and start the delay timer (Ch2)
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;  // just enable the samp rate clock for now
}

// Handler for timer0 ch2 interrupt (just for profiling) (wrote this so long ago i forgor if ok to delete so will just leave in) //
extern "C" void TC2_Handler(void) {
  uint32_t sr = TC0->TC_CHANNEL[2].TC_SR;     // reads status register which autoclears flags
}

// Handler for ADC --> PDC buffer full interrupt //
extern "C" void ADC_Handler(void) {
  uint32_t sr = ADC->ADC_ISR;                 // clears status bits
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS;            // stop RX while processing / rearming
  TC_Stop(TC0, 0);                            // stop ADC timer, disable the clock
  (void)TC0->TC_CHANNEL[0].TC_SR;             // clear timer status register DONT NEED ANYMORE?
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG;   // reset the counter but dont start as it is disabled

  // Buffer cycling // 
  if (pdc_last_buf == 2) {                          // if last_buffer was buf2, we just filled buf0, 
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf1;   // so record last_buffer as 0 and rearm pdc with buf1
    pdc_last_buf = 0;
  } else if (pdc_last_buf == 1) {
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf0;   // if last was 1, just filled 2, so set last_buf to 2 and rearm with 0
    pdc_last_buf = 2;
  } else if (pdc_last_buf == 0) {
    ADC->ADC_RPR = (uint32_t)(uintptr_t)pdc_buf2;   // if last was 0, just filled 1, so set last_buf to 1 and rearm with 2
    pdc_last_buf = 1;
  }

  fresh_data = true;                                // signal main loop that new data is ready
  ADC->ADC_RCR = N;                                 // Re-arm PDC for next capture
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;                   // restart PDC RX
}


// --------------------- MISC SMALL HELPER FUNCS ----------------------- //

void fastDigWrite(uint32_t arduinoPin, bool val) {
  Pio* port = g_APinDescription[arduinoPin].pPort;
  uint32_t mask = g_APinDescription[arduinoPin].ulPin;
  if (val) port->PIO_SODR = mask;
  else      port->PIO_CODR = mask;
}

bool fastDigRead(uint32_t arduinoPin) {
  Pio* port = g_APinDescription[arduinoPin].pPort;
  uint32_t mask = g_APinDescription[arduinoPin].ulPin;
  return (port->PIO_PDSR & mask) != 0;
}


// Laser diode can be damaged by high amplitude fast rectangle signals, so this smooths out sudden DAC steps //
void gentleDacRamp(unsigned int delayus, int final) { 
  final = constrain(final, 0, 4095);                 // avoid rollovers shocking the diode current
  int step = final - daclvl;                         
  int sign;
  if (step == 0) {
    return;
  } else if (step < 0) {
    sign = -1;
  } else {
    sign = 1;
  }
  for (int i = 0; i < abs(step); i++) {
    daclvl += sign;                                  // bump current modulation by +-1
    analogWrite(MOD_DAC, daclvl);                    // this takes 3.6 us and is negligible compared to ~order 500us explicit delay 
    delayMicroseconds(delayus);                 
  }
}

// ------------------------- MAIN LOOP FUNCS -------------------------- //

// Grab scans from last PDC buffer filled by the asych scope // 
void acquireScan() { 
  fresh_data = false;
  if (pdc_last_buf == 2) {                                                 // last filled buffer was buf2, so copy buf2
    memcpy((void*)buffer, (const void*)pdc_buf2, N * sizeof(uint16_t));    // copies pdc buffer to main loop buffer for processing
  } else if (pdc_last_buf == 1) {
    memcpy((void*)buffer, (const void*)pdc_buf1, N * sizeof(uint16_t));    // etc. for other cases
  } else if (pdc_last_buf == 0) {
    memcpy((void*)buffer, (const void*)pdc_buf0, N * sizeof(uint16_t));
  }
}


// Local max detection //
bool isLocalMax(int i, int window) {
 uint16_t val = buffer[i];
 for (int j = -window; j <= window; j++) {
   if (j == 0) continue;
   if (buffer[i + j] > val) return false;
 }
 return true;
}


// Find peaks in current scan //
void findPeaks() {
 int window = 5;
 int f = 0;
 tempPeakNo = 0;
 for (int i = window; i < N - window; i++) {
   uint16_t val = buffer[i];
   if (val > peakfind_thresh && isLocalMax(i, window)) {
     if (f < PEAKS_PER_SCAN) {
       tempPeaks[f++] = val; 
       tempPeakNo++;
     }
   }
 }
 if (tempPeakNo > 0) {
    peakfinder_success_flag = true;
 } else { 
    peakfinder_success_flag = false;
 }
}


// Check if peak lost //
bool isPeakLost(float maxVal, float meanHeight, float stdHeight) {
 if (maxVal < (meanHeight - stdHeight)) return true;
 return false;
}


/* Checks external GPIO flags set by the PI, checks internal state info, 
   outputs appropriate GPIO flags, returns true if all checks passed 
*/ 
bool statusWrapper() { 
  digilock_status_flag = fastDigRead(DIGILOCK_STATUS);
  mod_enable_flag = fastDigRead(MOD_ENABLE);

  if (initialized) {
    // check peakslost status, will output to GPIO and also use in feedback loop
    longterm_peakslost_flag = isPeakLost(runningBuffer.getMean(), initHeight, long_mem_n_stdev * initStd);
  }
  
  fastDigWrite(PEAKS_LOST, longterm_peakslost_flag || !peakfinder_success_flag);    // both finder fail and low peaks trigger this
  fastDigWrite(MOD_ACTIVE, mod_active_flag);                                        // these two get set in the feedback wrapper 
  fastDigWrite(MOD_FAILURE, mod_failure_flag);                                      // so don't need to change here, just output 

  // make sure all checks passed
  return digilock_status_flag && mod_enable_flag && !mod_failure_flag && initialized && peakfinder_success_flag;
}


// Does all of the feedback action, called every time main loop repeats (provided checks passed) //
void feedbackWrapper() {
  float short_term_mean = runningBuffer.getShortMean(shortMem * 2);         // * 2 cause 2 peaks per trace

  if (!mod_active_flag && longterm_peakslost_flag) {
    mod_active_flag = true;                                                 // activate feedback with some hysteresis
    countdown = 0;
  }

  if (!(mod_active_flag && countdown == 0)) {
    countdown--;                                    // exit fbk wrapper and delay to fill up short term memory
  } else {                                          // else it's time to check short term memory
    countdown = shortMem;                           // reset countdown

    if (bump_count > max_bumps || daclvl <= (min_daclvl + dac_step)) {      // fail if too many bumps or out of range
      mod_failure_flag = true;                                              // blocks entering feedback wrapper again
      mod_active_flag = false;  

    } else if (isPeakLost(short_term_mean, initHeight, short_mem_n_stdev * initStd)) { 
      gentleDacRamp(500, daclvl - dac_step);        // bump the current down
      bump_count++;
    } else {                                        // thus short term peak height is above stdev
      if (longterm_peakslost_flag) {
        countdown = shortMem;                       // short term is good, wait for long term and keep checking. INTEGRALGAIN
      } else {
        mod_active_flag = false;                    // disable feedback if long term memory is good
        bump_count = 0; 
      }
    }
  } 
}


/* Receive commands throught serial of the format b'<letter><arg1>,<arg2>,...'
   Fills preexisting vars passed as args
   Returns boolean used in flow control of main loop
*/
bool readCommandLetterNumbers(char &cmdChar, float *values) {
  static char buf[64];
  static uint8_t len = 0;

  while (SerialUSB.available()) {
    int c = SerialUSB.read();
    if (c == '\r') continue;                // skip these guys
    if (c == '\n') {                        // finished reading in the serial to the buf
      if (len == 0) { return false; }       // handle empty case: b'\n'

      buf[len] = '\0';                      // turn buf into C string by adding \0 to the spot after last buf element in memory

      // pre-fill the values array with zeros
      for (uint8_t i = 0; i < MAX_CMD_ARGS; i++) {
        values[i] = 0;
      }

      // parse the char cmd keyletter
      char *p = buf;                                      // start at first index of (now cstring) buf
      while (*p && isspace((unsigned char)*p)) p++;       // skip any whitespace
      cmdChar = *p ? *p++ : '\0';                         // neat ternary, extracts whatever is at p, advances the pointer
      
      // parse the comma separated numbers 
      while (*p && isspace((unsigned char)*p)) p++;       // these ops work on cstrings and are nice :P
      uint8_t idx = 0;
      while (*p && idx < MAX_CMD_ARGS) {
        values[idx++] = strtof(p, &p);

        // skip space comma space. If no comma this is the end and break
        while (*p && isspace((unsigned char)*p)) p++;
        if (*p == ',') {
          p++;
          while (*p && isspace((unsigned char)*p)) p++;
        } else {
          break;
        }
      }

      len = 0;
      return true;
    }

    if (len < sizeof(buf) - 1)
      buf[len++] = (char)c;
    else
      len = 0;
  }

  return false;
}


// Compute Peak stats (just for the initialization proceedure) //
PeakStats computePeakStats(uint16_t *heights, int count) {
 PeakStats s = {0, 0};
 if (count <= 0) return s;
 float sumH = 0;
 for (int i = 0; i < count; i++) sumH += heights[i];
 s.meanHeight = sumH / count;
 float varH = 0;
 for (int i = 0; i < count; i++) varH += pow(heights[i] - s.meanHeight, 2);
 if (count > 1) s.stdHeight = sqrt(varH / (count - 1));
 return s;
}


// Initialization Proceedure fills up initPeaks, computes stats, sets initialized flag var //
bool initialize_peak_vals_locations() {
 foundInitPeakNo = 0;
 int scanCount = 0;

 while (scanCount < MAX_INIT_SCANS && foundInitPeakNo < NUM_INIT_PEAKS) {
   while (!fresh_data) {}                       // busy wait for new data
   acquireScan();
   findPeaks();
   scanCount++;
   if (peakfinder_success_flag) {               // make sure that it's actually grabbing peaks 
    for (int i = 0; i < tempPeakNo; i++) {
        if (foundInitPeakNo < NUM_INIT_PEAKS) {
          initPeaks[foundInitPeakNo++] = tempPeaks[i];        
        }
    }
   }
 }

 if (foundInitPeakNo > 0) {
  initPeakStats = computePeakStats(initPeaks, foundInitPeakNo);
  initHeight = initPeakStats.meanHeight;
  initStd = initPeakStats.stdHeight;
  initialized = true;
  return true;
 } else {
  initialized = false;
  return false;
 }
}

// -------------- Old funcs which may be useful again --------------- // 

// void profilingHelper(uint32_t arduinoPin, int no_flash){
//   fastDigWrite(arduinoPin, false); 
//   for (int i = 0; i < no_flash; i++) {
//     fastDigWrite(arduinoPin, true);
//     delayMicroseconds(500);
//     fastDigWrite(arduinoPin, false);
//     delayMicroseconds(500);
//   }
// }
