#!/bin/bash
#
# This script is meant to go fetch the most recent versions of various libraries that
#   this program has been written against.
mkdir -p lib

# ManuvrDrivers
rm -rf lib/ManuvrDrivers
git clone https://github.com/jspark311/ManuvrDrivers.git lib/ManuvrDrivers

# CppPotpourri
rm -rf lib/CppPotpourri
git clone https://github.com/jspark311/CppPotpourri.git lib/CppPotpourri

# Teensyduino and support libraries...
rm -rf lib/teensyduino
rm -rf lib/teensy4_i2c
rm -rf lib/ManuvrPlatforms
git clone https://github.com/PaulStoffregen/cores.git lib/teensyduino
git clone https://github.com/Richard-Gemmell/teensy4_i2c.git lib/teensy4_i2c
git clone https://github.com/jspark311/ManuvrPlatforms.git lib/ManuvrPlatforms
