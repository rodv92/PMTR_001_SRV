# PMTR_001_SRV_001 V0.22


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



PMTR_001_SRV is a smart 3 phase power meter with relay action for low voltage (110V 240V 400V) consumer,commercial,industrial electrical networks.
PMTR_001_SRV IS NOT INTEDED FOR DISTRIBUTION / POWER UTILITY NETWORKS !



PMTR_001_SRV Should be installed on the consumer side, After the main-circuit breaker.
PMTR_001_SRV Contains a PLC module. No Line filter should be use between the server and the client, the client would be unable to communicate with the server !
L1, L2, L3 current coils should be placed on the current carrying wires of the metered installation, not on the wires supplying energy to the meter !!
PMTR_001_SRV can operate over IT, TT, TN-C TN-CS earthing schemes.
Being a non metallic chassis, PMTR_001_SRV does not require to be grounded.
PMTR_001_SRV contains ceramic fuse protection and varistor overvoltage protection.


Modified PZEM-004t v3.0
The original PZEM-004t v3.0 can measure up to 267V RMS.


Here, The metering is done through a modified PZEM-004t v3 for up to European phase to phase voltage tolerance :
up to 427V RMS with 0.2V resolution. Compared to the non-modified version, This ALSO allows phase to phase metering, and phase to neutral in case of neutral fault.
Power, Energy, power factor resolution are doubled due to the fact that their computing involve instantaneous voltage measurement which is affected by the PZEM modification.

Otherwise, the floors, ceilings of the remaining parameters conforms to the PZEM-004T v3.0 datasheet of this day.
It is expected that frequency measurement is not affected by the PZEM-004t v3.0 HV modification.
For power factor, leading or lagging info is not provided.

PMTR_001_SRV performs instantaneous measurement as fast as the 9600 bps serial interface allows, which is a little less than 100 per second (all parameters read)
When performing 3 phase metering the PMTR_001_SRV performs serial port switching (using an ADG333A Quad SPDT switch) So it shares a single serial port on the Arduino for 3 PZEM-004T modules.
this lowers the read rate to less than 33 per second.

an average value over 1 second of these 33 samples is stored as the "1 second average"
On the basis of this 1 second average, then a moving average from 2 to maximum of 60 seconds is saved.

The device has 5 memory slots for Moving Average information of all parameters (except energy)
For instance, We could store the Moving Average over 5, 10, 20, 30, 60 seconds.

The incentive behind moving average calculation is to apply low pass filtering and filter any spurious spike that would make an inappropriate relay action at the time.

An 'relay action' is a set of the following parameters :

-An expression : using tokens representing PMTR_001_SRV parameters, and a set of inequalities and logic.
-A moving average to use for expression evaluation (except energy which is never averaged)
-A dry run, which does not perform the relay action, but does log all action behaviour
-A recovery period during which the moving average must evaluate negatively the expression before reverting relay action
A HIGH or LOW state to command the relay on expression true. (on false evaluation the state commanded will the be opposite ) nb  : This evaluation from memory is the first action to be executed at boot up.
A list of pins on which DIGITALWRITE Will be performed.
It is recommended that each action activates a separate set of pins, otherwise the last evaluated actions will have precedence if not dryruns.

There are 5 actions slots.


Examples of valid expressions :

(L1U > 253) || (L2U > 253)) || (L3U > 253)
LII > 16
LIp > 1200
L2e > 10
(L1f < 49.1) && (L2f < 49.1) (L3f < 49.1)

Ex : L1U is phase to neutral L1 RMS voltage, L1I is phase to neutral RMS current, L2f is L2 frequency.
Units use are SI, excepts for energy which is in kWh

First case would be the detection of a voltage swell over 253V over L1,L2 and L3 at the same time.
The second a current of more than 16A on L1
Next an active power of more than 1200W on L1
Next, A total energy consumed of more than 10kW over L2
Finally a frequency sag of 49.1 Hz over any of the phases



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

The following IC : ADG333 allows routing a single hardware serial port to the three PZEM modules. Querying is continuous and sequential (all parameters of module 1,2,3,1,2...)

ES-1642NC uses a separate hardware serial port for modbus communication over one phase.
the serial line speed is 9600 bps. the PLC speed is between 2.5 to 4.5 kbps.


A real time clock (RTC) 3231 mini module is used to keep track of time and the synchronization is done over PLC / Modbus.

Modbus address encoding of the server is done throught a 8 bit DIP switch encoder.

----
SETUP :

0) Disconnect power to the facility or to the VFD for safe installation of the current coils.

1) Setup Modbus Address using the 8 position DIP switch to any value between 1 and 254. 0 is a broadcast address. 255 is a bridge address. Do not use these on the PMTR_001_SRV
2) Set the 3 jumpers to LN metering - default - (line to neutral) or LL metering (line to line) Usually LN is preferred if the metering is on a standard european consumer network. LL metering is usually reserved for Variable Frequency Drive metering. In that case, skip step 4.

2) After each phase is disconnected from the terminals, insert the current coil over the wire and reattach to the input and output terminals. repeat for each phase.
3) Connect CT- and CT+ wires to the appropriate PMTR-001-SRV terminals. Repeat for each phase
Then,
4) Connect neutral wire first to the PMTR_001_SRV 
5) You can re-establish power inside the facility now or power the VFD

6) then connect L1 L2 L3. Beware, lines are powered on, use an electrician screwdriver !!!!
When connecting L1, the unit should power up, unless on a VFD where you need at least two phases. You can test that all DC power supplies are providing backup power by connecting them individually. PMTR_001_SRV should work with either L1, L2 or L3 connected in LN metering.

7) if a backup / battery supply is available you can connect it to AUX12VDC to the positive terminal and GND to the negative terminal

8) Tie the SEND terminal to any L1 L2 or L3 phase, now the Rasperry Pi (PMTR_001_CLI) can be connected to the same phase and neutral as the PMTR_001_SRV and it should set the time up

repeat the same actions 1 to 8 on any other PMTR_001_SRV to install on the network. use the same phase line for all the PMTR_001_SRV that you install

PMTR_001_SRV can supply up to 0.8A at 12V over the 12VDC_OUT terminal, electromechanical relays can be powered by 12V IF the 5V relay action pin is recognized as HIGH by the relay.
anyway, PMTR_001_SRV can supply up to 1.7A at 5V over the 5VDC_OUT terminal, for electromechanical relays. 
nb: The relay action pins cannot supply more than 40mA current, use is limited only to SSR relays !!

-----
Proceed to PMTR_001_CLI Readme to finish setup.



Most of the single phase functionality is POC- tested.
three phase hardware testing and harderning is still in progress.
Software is still in POC phase. Consider the code to be in alpha stage.
