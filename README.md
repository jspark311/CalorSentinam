# Calor Sentinam (Heat Pump)

This is an instrumented heat pump based on thermoelectric elements and an array of TMP102 sensors.

Operation is entirely single-threaded with co-operative scheduling, as best as I could manage it without explicitly building a faculty to do it better. Might use my scheduler class in the future.

----------------------

## Teensy4 support

This project depends on [ManuvrPlatforms](https://github.com/jspark311/ManuvrPlatforms) to provide the fill-in for AbstractPlatform as appropriate for the Teensy4. It must be pulled into the `lib` folder.

This project depends on [ManuvrDrivers](https://github.com/jspark311/ManuvrDrivers) to support most of the hardware. It must be pulled into the `lib` folder.

----------------------

## Calorimetry features

The originally purpose of this machine was to be a water chiller for closed-loop
heat-exchange in the context of a chemistry lab. IE, pumping heat out of condensers
and reactor jackets. As such, the machine needs to be able to pump nearly a kilowatt
of waste heat. Now... moving heat is usually very extravagant of energy, and a machine
that has a pumping capacity of 1 kilowatt, will end up producing _at least_ several hundred
Watts of waste heat of its own (which it must also remove). *Weight adds weight.*

A machine that draws 1500W out of the wall should probably not be left unattended for as
long as it might take a chemical reaction to complete. The power supply for this machine
will happily turn the heat exchangers to lava and start a fire if some critical part fails.
It also needs to not turn the cold side of the pump to ice when the thermal flux falls.

So it needs to monitor heat flows through the machine, and possibly take actions
when some user-defined set of homeostatic parameters is violated.

### Monitor points

Coolant flow through the following thermal elements is monitored for temperature (both in and out):

    * The radiator
    * The cold-side heat exchanger
    * The hot-side heat exchanger(s)

The ambient air temperature, and speed of the radiator fans is also monitored. This data both informs the thermal model, and ensures machine safety.

Current flows through the following electrical elements is monitored, and used to constrain the thermal model:

    * The power cord
    * The TEC elements
    * The pumps and fans (as a single group)

### Configuring the firmware for alternate pump configurations

    * *Low-differential, high efficiency:* Both banks of TECs are cooling the center exchanger, which is the pump's cold side.
    * *High differential, low efficiency:* One TEC bank is cooling the hot side of the other. This allows operation down to the minimum temperature of -55C, at the cost of a rough quartering of the pump's capacity.

### Using the firmware to measure the thermal load imposed by a connected system


----------------------

#### License

Original code is Apache 2.0.

Code adapted from others' work inherits their license terms, which were preserved in the commentary where it applies.

The `assets` directory contains the source images for icons used in the program. I am under the impression that this is all free clipart.
