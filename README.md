## Description 
Repo for saving the Raspberry Pi script and Arduino sketch used in Gupta lab B055 to monitor and stabilize the blue master laser.

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

You also need to install the digilock_remote package found here:
https://github.com/ForestTschirhart/digilock_remote_v2
