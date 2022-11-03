# PMTR_001 V0.21

# LIB INCLUDES / CREDITS / MAIN CONTRIBUTORS

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <ModbusRTUServer.h>
#include <ModbusServer.h>

// Modbus library used for PLC client/servers communication 
// ArduinoModbus depends on the ArduinoRS485 library https://github.com/arduino-libraries/ArduinoRS485
//https://github.com/arduino-libraries/ArduinoModbus
//// by sandeepmistry


#include <PZEM004Tv30.h>
// specific modbus conditioning and parsing library for the power monitoring modules
//https://github.com/mandulaj/PZEM-004T-v30
//// by mandulaj

#include <TimeLib.h>
/// library for scheduling recurrent tasks 
//https://github.com/PaulStoffregen/Time/blob/master/TimeLib.h
//// by PaulStroffregen

#include <expr.h>
//// library for parsing math and logical expressions (for relay logic)
//https://github.com/zserge/expr/blob/master/expr.h
//// by zserge



PMTR is a 3 phase power meter for 110V 230V 380V networks. it measures the following power parameters :
voltage (RMS) up to 427V with 0.2 V precision
current through a current transformer up to 100A
active power VA
frequency with 0.1 Hz precision
power factor (leading or lagging info is not provided)

the metering is done through a modified PZEM-004t v3 for up to European phase to phase voltage tolerance.

The following protection layer has been added :

- 20mm x 5mm 0.1 A ceramic fuses, fast blow 250V
this for each phase L1 L2 L3 and neutral N
-14D481K varistors between each phase and neutral, and for protection of the PLC module.


----
AC/DC conversion :


Power is provided by YHT4S12V/10W modules AC380V tolerant.
the three modules are connnected line to neutral and should be tolerant to neutral fault and up to 2 phase faults.

-----
DC Bus :

Matched Schottky diodes are used to improve power sharing betweeen the modules at 11.5V
a single LM2596 dc step down module provides 5V to the DC bus

-----
PLC :

The main controller is an Arduino mega 2560 it is on a separate mezzanine board than the HV parts / PZEM Modules.

The following IC add functionallity.

ADG333 to route a single hardware serial port to the three PZEM modules. Querying is continuous and sequential (1,2,3,1,2...)
ES-1642NC uses a separate hardware serial port for modbus communication over one phase.
the serial line speed is 9600 bps. the PLC speed is between 2.5 to 4.5 kbps.


A real time clock (RTC) 3231 mini module is used to keep track of time and the synchronization is done over PLC / Modbus.

Modbus address encoding of the server is done throught a 8 bit DIP switch encoder.

Most of the single phase functionality is POC- tested.
three phase hardware testing and harderning is still in progress.
Software is still in POC phase. Consider the code to be in alpha stage.
