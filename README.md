# PMTR_001_SRV_001 V0.22




Description
----
PMTR (Power Meter, Telemetry & Relay) is an open source project that allows precise monitoring of mains parameters (voltage, current, active power, energy, power factor and frequency) and allows to act on external relays based on formula triggers (ex: overcurrent or overvoltage)
It is a client/server project with a Web interface for data display, logging and device configuration.
It uses power line communication and the industrial proved Modbus protocol to transfer data between the client and servers; no need to have ethernet or wifi at your main power panel.

It is modular and robust in approach, using off the shelf components available on Aliexpress or other vendor sites to ease replacement of parts if need arise, and thus lower TCO for many years.

The server(s), (PMTR_001_SRV) is an Arduino based device that performs the power measurements and relay actions. Its configuration is pushed by a client.

The client(s) (PMTR_001_CLI) is a Raspberry Pi device that grabs power telemetry and pushes configuration and time synchronisation to the server(s) device. As stated above, it does not rely on an Ethernet connection, just a Power line communication based on the ES-1642 NC module (also available on Aliexpress)

Relay actions are used mainly to protect your equipment (for industrial settings) and control energy costs (ex: hotel industry, power charging for EV), It will also allow to cut/enable power at specific time of the day.

This project, using the modbus protocol, will allow several servers. thus you can monitor individual circuit branchings on power subpanels.

Two flavour of servers are being developped : A single phase meter, and a 3 phase meter. they are rugged using fuses and varistors for overvoltage protection.
Metering is based on modified PZEM-004t v3.0 modules that can measure line to line voltages up to 437V AC.

For more information on the client, check https://www.github.com/rodv92/PMTR_001_CLI

----
Current State of the project.

A single phase proof of concept with a raspberry pi client and a single phase power meter was done and shows good results. The server code integrate formula based actions, and time synchronization. A basic web chart of power parameters and logging into a MariaDB database is part of this POC.

What needs to be done in priority as of November 2022:

- Finish the 3 phase prototype server, currently designed with Easy-EDA pro.
    - Add ADG333 switching of serial line to query each PZEM-004t V3 modules.
- Add code for DS3231 time keeping.
- Harden and test formula actions where conflicting pins are used with different actions.
- Make the raspberry pi (wifi, serial) configuration seamless
- Expand the raspberry pi client to query several servers with auto-detection on the PLC bus 
- Add a MQTT layer on the raspberry pi for upstream telemetry reporting
- keep in touch with PeaceFair, the maker of the PZEM-004t V3.0 to make a hardened 400V module (there are still some issues regarding with high voltage tolerance)

----
projected INSTALLATION and SETUP of the 3 phase prototype :

PMTR_001_SRV IS NOT INTEDED FOR DISTRIBUTION / POWER UTILITY NETWORKS !

typical use case for 3 phase delta-wired measurement:

![alt text](https://github.com/rodv92/PMTR_001_SRV/blob/master/hw/pmtr_001_use_case.png?raw=true)

This use case shows the installation of the PMTR_001_SRV for monitoring voltage and current drawn by a 3 phase induction motor, and all remaining power parameters.
Note that since the motor is a 3 phase delta wired device, PMTR_001_SRV has to be setup in the following way :
- Hardware method : By placing measurement jumpers inside the device for line voltage measurement (L1,L2),(L2,L3)(L3,L1)
OR
- Software method : By specifying a multiplicative correction factor : sqrt(3) for power and energy measurements inside the PMTR web interface. this factor will be uploaded to the PMTR_001_SRV device

The setup contains a single digital pin operating a 5V relay (NO) that provides in turn power to a contactor (NO) that power ups (or down) the induction motor.

It is then trivial to use logic to specify a relay action that evaluates and power ups or down the motor upon specific power conditions. More on relay actions below.

Also note that the PMTR_001_SRV and PMTR_001_CLI are connected on the same line (L1) for PLC communication to occur.

----

PMTR_001_SRV Contains a ES-1642 NC PLC modem (Power Line Communication). No Line filter should be present between the server and the client, the client would be unable to communicate with the server !
L1, L2, L3 current coils should be placed on the current carrying wires of the metered installation, not on the wires supplying energy to the meter !!
PMTR_001_SRV can operate over IT, TT, TN-C TN-CS earthing schemes.
Being a non metallic chassis, PMTR_001_SRV does not require to be grounded.
PMTR_001_SRV contains ceramic fuse protection and varistor overvoltage protection.
PMTR_001_SRV will be packaged inside a standar DIN Rail plastic enclosure for ease of installation inside a power distribution main panel.

Check the HW folder of this project for the terminal pin references.

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



----
Modified PZEM-004t v3.0 information

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

- A 'relay action' is a set of the following parameters :

- An expression : using tokens representing PMTR_001_SRV parameters, and a set of inequalities and logic.
- A moving average to use for expression evaluation (except energy which is never averaged)
- A dry run bit (Yes/No), which tells whether to perform or not the relay action, while still logging all action behaviour
- A recovery period during which the expression must keep evaluating to false before reverting relay action. This hysteresis is implemented for protecting devices which should not be turned on/off too frequently.
- A HIGH or LOW state to command the relay when expression evaluates to true. (on false evaluation the state commanded will the be opposite ) nb  : This evaluation from memory is the first action to be executed at boot up.
- A list of pins on which DIGITALWRITE Will be performed.
It is recommended that each action activates a separate set of pins, otherwise the last evaluated actions will have precedence and overwrite the status of pins.

There are 5 actions slots.


Examples of valid expressions :

- (L1U > 253) || (L2U > 253)) || (L3U > 253)
- LII > 16
- LIp > 1200
- L2e > 10
- (L1f < 49.1) && (L2f < 49.1) && (L3f < 49.1)

Ex : L1U is phase to neutral L1 RMS voltage, L1I is phase to neutral RMS current, L2f is L2 frequency.
Units use are SI, excepts for energy which is in kWh

First case would be the detection of a voltage swell over 253V over L1,L2 and L3 at the same time.
The second a current of more than 16A on L1
Next an active power of more than 1200W on L1
Next, A total energy consumed of more than 10kW over L2
Finally a frequency sag of 49.1 Hz over any of the phases



The following protection layer has been added :

- 20mm x 5mm 0.1 A ceramic fuses, fast blow 250V, this for each phase L1 L2 L3 and neutral N
- 14D481K varistors between each phase and neutral, and for protection of the PLC module.


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

It could be technically feasible to have all PZEM modules on the same RX/TX serial TTL pair, but that would need to ensure that the optocouplers do not overload the RX/TX current sourcing capabilities, and also would need a factory preset of the PZEM-004t V3.0 addresses. Note that PZEM-004t V3.0 itself is a modbus device.

The server itself encapsulate telemetry queries and present the 3 PZEM-004t V3.0 as a single modbus server 
device to the Raspberry Pi client.

Thus, it seems more adequate to use the same Modbus address on each module, and switch them using an IC switch like the ADG333.

ES-1642NC uses a separate hardware serial port for modbus communication over one phase.
the serial line speed is 9600 bps. the PLC speed is between 2.5 to 4.5 kbps.


A real time clock (RTC) 3231 mini module is used to keep track of time and the synchronization is done over PLC / Modbus.

Modbus address encoding of the server is done throught a 8 bit DIP switch encoder.


-----
Proceed to PMTR_001_CLI Readme to finish setup.



Most of the single phase functionality is POC tested.
three phase hardware testing and harderning is still in progress.
Software is still in POC phase. Consider the code to be in alpha stage.


----
Hardware Schematics (check the /hw folder)

![alt text](https://github.com/rodv92/PMTR_001_SRV/blob/master/hw/SCH_Logic_Board.pdf?raw=true)

This is the logic mezzanine board.


![alt text](https://github.com/rodv92/blob/master/SCH_Power_Board.pdf?raw=true)

This is the high voltage board.

# LIB INCLUDES / CREDITS / MAIN CONTRIBUTORS


Modbus library used for PLC client/servers communication 
ArduinoModbus depends on the ArduinoRS485 library https://github.com/arduino-libraries/ArduinoRS485
https://github.com/arduino-libraries/ArduinoModbus
by sandeepmistry


specific modbus conditioning and parsing library for the power monitoring modules
https://github.com/mandulaj/PZEM-004T-v30
by mandulaj

library for timekeeping 
https://github.com/PaulStoffregen/Time/blob/master/TimeLib.h
by PaulStroffregen


library for scheduling task by timer ISRs
https://github.com/khoih-prog/TimerInterrupt
by khoih-prog

library for parsing math and logical expressions (for relay logic)
https://github.com/zserge/expr/blob/master/expr.h
by zserge
