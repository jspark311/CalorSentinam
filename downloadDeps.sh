#!/bin/bash
#
# This script is meant to go fetch the most recent versions of various libraries that
#   this program has been written against.
mkdir -p components

# ManuvrDrivers
rm -rf components/ManuvrDrivers
git clone https://github.com/jspark311/ManuvrDrivers.git components/ManuvrDrivers

# CppPotpourri
rm -rf components/CppPotpourri
git clone https://github.com/jspark311/CppPotpourri.git components/CppPotpourri

# Support libraries...
rm -rf components/ManuvrPlatforms
git clone https://github.com/jspark311/ManuvrPlatforms.git components/ManuvrPlatforms
ln -s components/ManuvrPlatforms/ESP32 components/ManuvrPlatform
