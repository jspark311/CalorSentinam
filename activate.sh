#!/bin/bash
source /home/ian/.espressif/python_env/idf4.2_py3.8_env/bin/activate
export PATH=$PATH:/opt/esp/tools/
idf.py build size
idf.py flash monitor
