## Description 
Repo for saving the Raspberry Pi script and Arduino sketch used in Gupta lab B055 to monitor and stabilize the blue master laser.
Arduino sketch just included for context. If you actually wanted to use it you would need to put it inside a folder of the same name, and also maybe an overarching Arduino folder. 
## Dependencies
To run the Pi script you need to install several packages using pip as follows: 

```
pip install <see below>
----------------------------
uvicorn
telnetlib3
numpy
pyserial
"fastapi[standard]"
orjson
RPi.GPIO
requests
```

You also need to install the digilock_remote package found here: https://github.com/ForestTschirhart/digilock_remote_v2

With the desired python env activated:
```
git clone https://github.com/ForestTschirhart/digilock_remote_v2.git
cd digilock_remote_v2
pip install -e .
```

## Arduino Stuff

Script intended to be used with the arduino script included in this repo. For more about the arduino project see:
https://github.com/ForestTschirhart/gupta_lab_arduino_proj
