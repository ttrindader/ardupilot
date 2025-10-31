#!/bin/bash
sim_vehicle.py  -D -f rover-skid --model gazebo-rover --out=udpout:127.0.0.1:14550 --out=udpout:127.0.0.1:14555 -L SAE --console --add-param-file gzsitl.params
