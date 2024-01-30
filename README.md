# Tools for [TTT4280 - Sensorer og instrumentering](https://www.ntnu.no/studier/emner/TTT4280#tab=omEmnet)

Use [remote_adc_server.c](adc_server/remote_adc_server.c) on your pi to create a server that allows a client to request and retrieve adc data remotely.  
Build and run with `sudo make run`

Run [remoteADC.py](py/remoteADC.py) on your computer to get the adc data.  
[live_plotter.py](py/live_plotter.py) gives you a live (but laggy) oscilloscope-esque view of the adc data.