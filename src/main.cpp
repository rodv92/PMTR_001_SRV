//CREDITS / INCLUDES

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <ModbusRTUServer.h>
#include <ModbusServer.h>
#include <PZEM004Tv30.h>
#include <TimeLib.h>
#include <expr.h>
#include <Wire.h>
#include <DS3231.h>
#include <time.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#define USE_TIMER_1     false
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     true
#include <TimerInterrupt.h>
#include "ISR_Timer.h"
#include <SimpleTimer.h>              // https://github.com/schinken/SimpleTimer


uint8_t DebugLevel = 0;
bool Anyerror = false;
uint8_t maxPZEMRetries = 3;
uint16_t PZEMRetries = 0;
uint16_t PZEMReads = 0;
uint8_t DeviceAddress;
uint16_t LogEndMarkerAddr = 0; // eventcode is never written at index 0, so it means that it is uninitialized and we must find
// it with GetLastEventAddrFromEEPROM(). If it returns 0, it means no events are logged.
uint16_t globalEventCounter = 0; 

ISR_Timer ISR_timer;
DS3231 myRTC;

// The following Arduino pins are ued to switch serial2 signals to each of the 3 PZEMs using the ADG333A
// Quad SPDT Switch
#define ADG333A_IN1_PIN 32
#define ADG333A_IN2_PIN 33
#define ADG333A_IN3_PIN 34
#define ADG333A_IN4_PIN 35

PZEM004Tv30 pzem(&Serial2);
time_t sync_time;
uint16_t sync_offset = 6;

bool linetest = false;
const uint16_t timeoutSyncMillis = 20000;
const int ledPin = LED_BUILTIN;
uint16_t VoltageModbusRegister = 0;
uint16_t CurrentModbusRegister = 0;
uint16_t PowerModbusRegister = 0;
uint16_t EnergyModbusRegister = 0;
uint16_t FrequencyModbusRegister = 0;
uint16_t PowerFactorModbusRegister = 0;

uint16_t AvgVoltageModbusRegister = 0;
uint16_t AvgCurrentModbusRegister = 0;
uint16_t AvgPowerModbusRegister = 0;
uint16_t AvgFrequencyModbusRegister = 0;
uint16_t AvgPowerFactorModbusRegister = 0;

const uint16_t sampling_intervals_millis = 1000; // period of PZEM sampling by the ISR
const uint8_t nbvalues = 20; // number of PZEM samples to retain in memory.

uint8_t WindowLengths[5] = {5,10,0,0,0};

/*
struct MovingAveragesStruct
{
  uint16_t NbSamples; // number of samples to retain for moving average calculation
  double NowValues[6]; // Now values to provide for Moving Average Calculation
  double MovingAverageValues[5]; // output values resulting from Moving Average calculation
};
*/

uint16_t NowValues[18]; // contains most recent values read from PZEM. used as storage since PZEM cannot be queried inside ISR because it uses Serial.

struct MovingAveragesStructV2
{
  uint8_t WindowLength;
  uint32_t MovingAverageValues[15];

};


// 5 moving averages slots to store computed MA value for different windows for each metric except Energy.
volatile MovingAveragesStructV2 myMovingAveragesStructV2_0;
volatile MovingAveragesStructV2 myMovingAveragesStructV2_1;
volatile MovingAveragesStructV2 myMovingAveragesStructV2_2;
volatile MovingAveragesStructV2 myMovingAveragesStructV2_3;
volatile MovingAveragesStructV2 myMovingAveragesStructV2_4;


volatile MovingAveragesStructV2 *MovingAveragesStructV2ptr[5] = {&myMovingAveragesStructV2_0,&myMovingAveragesStructV2_1,&myMovingAveragesStructV2_2,&myMovingAveragesStructV2_3,&myMovingAveragesStructV2_4};



struct CircularBufferValuesStruct
{
  uint8_t FillNbValues; // used during ring buffer initial fill, capped at nbvalues. used for moving average initialization
  uint8_t NextWriteIndex; // index for next write;
  uint16_t PZEMValues[nbvalues][15]; //struct storing values history for each metric except energy
};

// total history buffer window in seconds = sampling_interval_millis * nbvalues; typically 60 seconds (60 values with ISR firing each second)
// MA window cannot be larger than nbvalues !

volatile CircularBufferValuesStruct myCircularBufferValuesStruct;


//volatile MovingAveragesStruct myMovingAveragesStruct = { 10, {0.0,0.0,0.0,0.0,0.0,0.0}, {0.0,0.0,0.0,0.0,0.0} };
int MovingAveragesTimerNumber;

struct TripFormulaData
{
  bool HighOrLowOnFormulaTrue = HIGH;
  bool FormulaDryRun = true;
  bool IsTripped = false;
  bool isTripRecoveryStrategyAuto = false; //false = Manual, true = Auto
  uint8_t MAWindowSeconds = 0; //0 marks trip slot as unused
  uint8_t DigitalPinsList[8];
  uint32_t TripRecoveryDurationSeconds; // after how many seconds after condition is cleared we can recover
  uint32_t CurrentTripRecoveryDurationSeconds; // how many seconds have passed since condition is cleared
  char FormulaChars[64];
};

volatile TripFormulaData myTripFormulaData_0;
volatile TripFormulaData myTripFormulaData_1;
volatile TripFormulaData myTripFormulaData_2;
volatile TripFormulaData myTripFormulaData_3;
volatile TripFormulaData myTripFormulaData_4;

volatile TripFormulaData *TripFormulaDataStructptr[5] = {&myTripFormulaData_0,&myTripFormulaData_1,&myTripFormulaData_2,&myTripFormulaData_3,&myTripFormulaData_4};


const int numCoils = 12;

// coil 0 : time synchronisation requested by client
// coil 1 : formula update requested by client
// coil 2 : formula digitalwrite HIGH or LOW on formula evaluation returning true
// coil 3 : formula DRY RUN : does not effect pin status, only reports it (see pin statuses coils)
// coil 4 : trip recovery strategy, MANUAL or AUTO on condition clear


  

const int numDiscreteInputs = 12;
//const int numHoldingRegisters = 28;
const int numHoldingRegisters = 109;


// HOLDING REGISTERS FOR TELEMETRY :
// holding register 0 to 35 : most recent telemetry
// holding register 36 to 65 : Moving average telemetry for processing by client purposes

// HOLDING REGISTERS FOR TIME SYNCHRONISATION
// holding registers 66,67,68,69 : epoch seconds in 64 bit format (Y2K38 compliant)
// holding register 70 : milliseconds to add to epoch seconds. valid integer values 0 to 999

// HOLDING REGISTERS FOR TRIP FORMULAS UPDATE
// holding register 71 LSB : MA Window in seconds to use for formula evaluation of variables. all variables in the same formula
// are evaluated using the same MA Windows
// holding register 71 MSB : Formula Slot that is being updated
// holding register 72 to 79 : list of pins to trip. each register holds 2 list items (8 bit pin ID). max number of pins : 8
// holding register 80 : trip recovery hysteresis time : duration in seconds condition must remain clear before
// pin(s) reactivation if AUTO MODE.
// holding registers 81 to 144 : formula string


const int numInputRegisters = 2;

void(* resetFunc) (void) = 0; 

String GetStringFromId(uint16_t stringId)
{
  switch (stringId)
  {
    case 0:
      return String(F("Unit started"));
    case 1:
      return String(F("Unit configuration OK"));
    case 2:
      return String(F("Initiating reset (from local serial command)"));
    case 3:
      return String(F("Initiating reset (from Modbus client)"));
    case 4:
      return String(F("Unexpected reset (watchdog initiated,reset pressed, or power loss)"));
    case 5:
      return String(F("reset after firmware update"));
    case 6:
      return String(F("Pausing unit operation"));
    case 7:
      return String(F("Resuming unit operation"));
    case 8:
      return String(F("Time synchronization OK"));
    case 9:
      return String(F("Time set from local serial command"));
    case 10:
      return String(F("Loss of power on all three phases. imminent shutdown"));
    case 11:
      return String(F("12V DC Bus low voltage"));
    case 12:
      return String(F("12V DC Bus critical low voltage. imminent shutdown"));
    case 13:
      return String(F("5V DC Bus low voltage"));
    case 14:
      return String(F("5V DC Bus critical low voltage. imminent shutdown"));
    case 17:
      return String(F("EEPROM cleared"));
    
    default:
      return String(F("Unknown event code"));
       
  }
}

size_t print64(uint64_t number, int base)
{
    size_t n = 0;
    unsigned char buf[64];
    uint8_t i = 0;

    if (number == 0)
    {
        n += Serial.print((char)'0');
        return n;
    }

    if (base < 2) base = 2;
    else if (base > 16) base = 16;

    while (number > 0)
    {
        uint64_t q = number/base;
        buf[i++] = number - q*base;
        number = q;
    }

    for (; i > 0; i--)
    n += Serial.print((char) (buf[i - 1] < 10 ?
    '0' + buf[i - 1] :
    'A' + buf[i - 1] - 10));

    return n;
}

inline void DebugPrint(String DebugStr, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {
    Serial.print(DebugStr);
    Serial.flush();
  }
}

inline void DebugPrint(uint8_t DebugNum, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {
    Serial.print(DebugNum);
    Serial.flush();
  }
}


inline void DebugPrint(int8_t DebugNum, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {
    Serial.print(DebugNum);
    Serial.flush();
  }
}

inline void DebugPrint(int16_t DebugNum, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {
    Serial.print(DebugNum);
    Serial.flush();
  }
}



inline void DebugPrint(uint64_t DebugNum, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {
    print64(DebugNum,10);
    Serial.flush();
  }
}


const byte numChars = 40;
char receivedChars[numChars]; // an array to store the received data
bool newData = false;

void recvWithEndMarker(char endChar) 
{
  static uint8_t ndx = 0;
  char rc;
 

  if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();
      //delay(10);
      if (rc != endChar) {
        receivedChars[ndx] = rc;
        ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
      }
    }
  }
}


uint8_t myPowofTwo (uint8_t p) 
{
  int i = 2;
  for (int j = 1; j <= p; j++)  i *= i;
  return i;
}

uint8_t GetDeviceAddress(uint8_t *pins, uint8_t supplypin)  // get device adress by reading voltage on 8 pins, array from LSB to MSB.
{

  // supplying 5V to address bus
  pinMode(supplypin,OUTPUT);
  digitalWrite(supplypin,HIGH);

  uint8_t k;
  bool digital_val;
  uint8_t devaddr = 0;
  for (k=0;k<8;k++)
  {
    pinMode(pins[k],INPUT);
    digital_val = digitalRead(pins[k]);
    devaddr += digital_val << k;
  } 


  // stop supplying 5V to address bus
  digitalWrite(supplypin,LOW);
  pinMode(supplypin,INPUT);
  

  return devaddr;
}

void TimerHandler()
{
  ISR_timer.run();
}



void EvalLogicalExpression(uint8_t MAWindowSeconds, char Expression[64], float &result)
{

  uint8_t indexptr;

  for(indexptr = 0; indexptr<5; indexptr++)
  {

    DebugPrint(F("EvalLogicalExpression: SEARCHING FOR EXISTING WINDOW STRUCT: indexptr windowslength requestedwindowslength\t"),5);
    DebugPrint(String(indexptr),5);
    DebugPrint(F("\t"),5);

    DebugPrint(String(MovingAveragesStructV2ptr[indexptr]->WindowLength),5);
    DebugPrint(F("\t"),5);
    
    DebugPrint(String(MAWindowSeconds),5);
    DebugPrint(F("\n"),5);
    

    if(MovingAveragesStructV2ptr[indexptr]->WindowLength == MAWindowSeconds)
    {
      // found the struct we will use
      DebugPrint(F("EvalLogicalExpression: STRUCT MATCH!: indexptr\t"),5);
      DebugPrint(String(indexptr),5);
      DebugPrint(F("\n"),5);
      break;
    }

  }
  if(indexptr == 5)
  {
      // struct not found
      DebugPrint(F("EvalLogicalExpression: ERR: NO MATCHING STRUCT!\n"),5);
      return;
    
  }

  struct expr_var_list vars = {0};
  
  // voltage
  struct expr_var *L1U = expr_var(&vars, "L1U", 3);
  L1U->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[0]);
  struct expr_var *L2U = expr_var(&vars, "L2U", 3);
  L2U->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[1]);
  struct expr_var *L3U = expr_var(&vars, "L3U", 3);
  L3U->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[2]);

  // current
  struct expr_var *L1I = expr_var(&vars, "L1I", 3);
  L1I->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[3]);
  struct expr_var *L2I = expr_var(&vars, "L2I", 3);
  L2I->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[4]);
  struct expr_var *L3I = expr_var(&vars, "L3I", 3);
  L3I->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[5]);

  // power
  struct expr_var *L1P = expr_var(&vars, "L1P", 3);
  L1P->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[6]);
  struct expr_var *L2P = expr_var(&vars, "L2P", 3);
  L2P->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[7]);
  struct expr_var *L3P = expr_var(&vars, "L3P", 3);
  L3P->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[8]);

  // energy. always use NowValues
  struct expr_var *L1e = expr_var(&vars, "L1e", 3);
  L1e->value = static_cast <float> (NowValues[15]);
  struct expr_var *L2e = expr_var(&vars, "L2e", 3);
  L2e->value = static_cast <float> (NowValues[16]);
  struct expr_var *L3e = expr_var(&vars, "L3e", 3);
  L3e->value = static_cast <float> (NowValues[17]);

  // frequency
  struct expr_var *L1f = expr_var(&vars, "L1f", 3);
  L1f->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[9]);
  struct expr_var *L2f = expr_var(&vars, "L2f", 3);
  L2f->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[10]);
  struct expr_var *L3f = expr_var(&vars, "L3f", 3);
  L3f->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[11]);

  // power factor
  struct expr_var *L1pf = expr_var(&vars, "L1pf", 4);
  L1pf->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[12]);
  struct expr_var *L2pf = expr_var(&vars, "L2pf", 4);
  L2pf->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[13]);
  struct expr_var *L3pf = expr_var(&vars, "L3pf", 4);
  L3pf->value = static_cast <float> (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[14]);


  //Serial.println(a->value);
  //Serial.println(b->value);
  //Serial.println(c->value);
  

  static struct expr_func user_funcs[] = {
    {NULL, NULL, NULL, 0},
};

  const char *s = Expression;

  DebugPrint(F("EvalLogicalExpression: Expression\t"),5);
  DebugPrint(String(s),5);
  DebugPrint(F("\n"),5);
    
  struct expr *e = expr_create(s, strlen(s), &vars, NULL);
    if (e == NULL) {
    DebugPrint(F("EvalLogicalExpression: EXPRESSION SYNTAX ERROR\n"),5);
    return 1;
  }

  result = expr_eval(e);
  DebugPrint(F("EvalLogicalExpression: EXPRESSION RESULT: result\t"),5);
  DebugPrint(String(result),5);
  DebugPrint(F("\n"),5);

  expr_destroy(e, &vars);
  return 0;

}


void FillAverageValuesRegisters()
{

    int32_t ret;
    
    for(uint8_t i=0;i<3;i++)
    {
      /*
      memcpy(AvgVoltageModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[0 + i]), sizeof(AvgVoltageModbusRegister));
      memcpy(AvgCurrentModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[3 + i]), sizeof(AvgVoltageModbusRegister));
      memcpy(AvgPowerModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[6 + i]), sizeof(AvgVoltageModbusRegister));
      memcpy(AvgFrequencyModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[9 + i]), sizeof(AvgVoltageModbusRegister));
      memcpy(AvgPowerFactorModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[12 + i]), sizeof(AvgVoltageModbusRegister));
      */

      //U Avg
      ret = ModbusRTUServer.holdingRegisterWrite(116 + 2*i,myMovingAveragesStructV2_0.MovingAverageValues[0 + i]);
      
      //ret = ModbusRTUServer.holdingRegisterWrite(116 + 2*i,AvgVoltageModbusRegister[0]);
      //ret = ModbusRTUServer.holdingRegisterWrite(117 + 2*i,AvgVoltageModbusRegister[1]);
      
      //I Avg
      ret = ModbusRTUServer.holdingRegisterWrite(122 + 2*i,myMovingAveragesStructV2_0.MovingAverageValues[3 + i]);
      
      //ret = ModbusRTUServer.holdingRegisterWrite(122 + 2*i,AvgCurrentModbusRegister[0]);
      //ret = ModbusRTUServer.holdingRegisterWrite(123 + 2*i,AvgCurrentModbusRegister[1]);
    
      //P Avg
      ret = ModbusRTUServer.holdingRegisterWrite(128 + 2*i,myMovingAveragesStructV2_0.MovingAverageValues[6 + i]);
      
      //ret = ModbusRTUServer.holdingRegisterWrite(128 + 2*i,AvgPowerModbusRegister[0]);
      //ret = ModbusRTUServer.holdingRegisterWrite(129 + 2*i,AvgPowerModbusRegister[1]);
      
      //f Avg
      ret = ModbusRTUServer.holdingRegisterWrite(134 + 2*i,myMovingAveragesStructV2_0.MovingAverageValues[9 + i]);
      
      //ret = ModbusRTUServer.holdingRegisterWrite(134 + 2*i,AvgFrequencyModbusRegister[0]);
      //ret = ModbusRTUServer.holdingRegisterWrite(135 + 2*i,AvgFrequencyModbusRegister[1]);
        
      //pf Avg
      ret = ModbusRTUServer.holdingRegisterWrite(140 + 2*i,myMovingAveragesStructV2_0.MovingAverageValues[12 + i]);
      
      //ret = ModbusRTUServer.holdingRegisterWrite(140 + 2*i,AvgPowerFactorModbusRegister[0]);
      //ret = ModbusRTUServer.holdingRegisterWrite(141 + 2*i,AvgPowerFactorModbusRegister[1]);
      
    }

        

}

void WriteCircularBufferValues()
{
  
  DebugPrint(F("WriteCircularBufferValues: START\n\n"),6);
  
  for (uint8_t IndexType = 0;IndexType <15; IndexType++)
  {
    DebugPrint(F("WriteCircularBufferValues: IndexType NextWriteIndex\t"),6);
    DebugPrint(String(IndexType),6);
    DebugPrint(F("\t"),6);
    DebugPrint(String(myCircularBufferValuesStruct.NextWriteIndex),6);
    DebugPrint(F("\n"),6);

    // QUICK FIX for the MA not to be impacted by PZEM NaN (error querying the PZEM, despite retries)
    // in this case NowValues[IndexType] is 0. We can not add such a value to the Circular Buffer, or MA will decrease.
    // a decrease in MA could be interpreted as a voltage sag, which is not necessarily the case
    // so we propagate the previous circular buffer sample to NextWriteIndex
    // TODO : handle the case where the PZEM is not connected to L/N or L/L, does it report NaN ?
    // TODO : add a failure flag to inform that the MA is not to be trusted, when too many samples are 0 from a NaN event, since NaN cannot be pushed to unsigned int.

    if (NowValues[IndexType] != 0)
    {
        myCircularBufferValuesStruct.PZEMValues[myCircularBufferValuesStruct.NextWriteIndex][IndexType] = NowValues[IndexType];
    }
    else
    {
        myCircularBufferValuesStruct.PZEMValues[myCircularBufferValuesStruct.NextWriteIndex][IndexType] = myCircularBufferValuesStruct.PZEMValues[(myCircularBufferValuesStruct.NextWriteIndex - 1 + nbvalues) % nbvalues][IndexType];
    }

  }


  DebugPrint(F("WriteCircularBufferValues: END\n\n"),6);
  DebugPrint(F("WriteCircularBufferValues: UPDATE INDEXES\n"),6);
  
  myCircularBufferValuesStruct.NextWriteIndex++;  
  myCircularBufferValuesStruct.NextWriteIndex %= nbvalues;
  myCircularBufferValuesStruct.FillNbValues++;
  myCircularBufferValuesStruct.FillNbValues = constrain(myCircularBufferValuesStruct.FillNbValues,1,nbvalues);

  DebugPrint(F("WriteCircularBufferValues: NextWriteIndex FillNbValues\t"),6);
  DebugPrint(String(myCircularBufferValuesStruct.NextWriteIndex),6);
  DebugPrint(F("\t"),6);
  DebugPrint(String(myCircularBufferValuesStruct.FillNbValues),6);
  DebugPrint(F("\n"),6);

}

void ComputeMovingAveragesV2Handler()
{

  // Called through ISR Timer. Never leave the code at a DebugLevel >= at the level defined in DebugPrint() calls in this function or inside 
  //WriteCircularBufferValues while in production, as Serial.print calls are blocking.

  // update circular buffer
  DebugPrint(F("ComputeMovingAveragesV2Handler: Call WriteCircularBufferValues() \n\n"),6);
  WriteCircularBufferValues();
  DebugPrint(F("ComputeMovingAveragesV2Handler: End Call WriteCircularBufferValues() \n\n"),6);
  
  // stop processing WindowLengths[5] at first 0 valued WindowLength requested
  uint8_t FirstZeroindexWindows = 0;

  for(uint8_t indexWindows = 0; indexWindows<5; indexWindows++)
    {
      if ((WindowLengths[indexWindows] == 0) && (indexWindows == 0)) {return;} // no moving average to process}
      else if (WindowLengths[indexWindows] == 0)
      {FirstZeroindexWindows = indexWindows; break;}
    }
  // next, find if there are any unused moving averages structs and free them.

  DebugPrint(F("ComputeMovingAveragesV2Handler: FREEING UNUSED STRUCTS\n\n"),6);
    
  for(uint8_t indexptr = 0; indexptr<5; indexptr++)
  {
    DebugPrint(F("ComputeMovingAveragesV2Handler: FREE UNUSED STRUCT?: indexptr\t"),6);
    DebugPrint(String(indexptr),6);
    DebugPrint(F("\n"),6);

    bool match = false;
    for(uint8_t indexWindows = 0; indexWindows<FirstZeroindexWindows; indexWindows++)
    {
      if(MovingAveragesStructV2ptr[indexptr]->WindowLength == WindowLengths[indexWindows]) {match=true;break;}
    }
    if (!match)
    {
      // this moving average struct is not needed anymore, free it by setting WindowLength to 0 and clear moving average values
      MovingAveragesStructV2ptr[indexptr]->WindowLength = 0;
      for (uint8_t indextype = 0; indextype < 15; indextype++)
      {
        MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = 0;
      }
      DebugPrint(F("ComputeMovingAveragesV2Handler: FREED!: indexptr\t"),6);
      DebugPrint(String(indexptr),6);
      DebugPrint(F("\n"),6);

    }
  }

  DebugPrint(F("ComputeMovingAveragesV2Handler: COMPUTING MOVING AVERAGES\n\n"),6);
    
  for (uint8_t indexWindows = 0; indexWindows < FirstZeroindexWindows; indexWindows++) 
  {

    DebugPrint(F("ComputeMovingAveragesV2Handler: COMPUTE MA FOR indexWindows:\t"),6);
    DebugPrint(String(indexWindows),6);
    DebugPrint(F("\n\n"),6);
    bool FoundStruct = false;

    for(uint8_t indexptr = 0; indexptr<5; indexptr++)
    {

      DebugPrint(F("ComputeMovingAveragesV2Handler: SEARCHING FOR EXISTING WINDOW STRUCT indexptr:\t"),6);
      DebugPrint(String(indexptr),6);
      DebugPrint(F("\n"),6);

      DebugPrint(F("ComputeMovingAveragesV2Handler: STRUCT WindowLength RequestedWindowLength:\t"),6);
      DebugPrint(String(MovingAveragesStructV2ptr[indexptr]->WindowLength),6);
      DebugPrint(F("\t"),6);
      
      DebugPrint(String(WindowLengths[indexWindows]),6);
      DebugPrint(F("\n"),6);
      

      if(MovingAveragesStructV2ptr[indexptr]->WindowLength == WindowLengths[indexWindows])
      {
        // found the struct we will use
        FoundStruct = true;
        DebugPrint(F("ComputeMovingAveragesV2Handler: FOUND STRUCT indexptr:\t"),6);
        DebugPrint(String(indexptr),6);
        DebugPrint(F("\n"),6);

        if (myCircularBufferValuesStruct.FillNbValues > WindowLengths[indexWindows])
        {
          //Perform MovingAverageCalculation : subtract oldest sample, add new one and divide by k;
          
          DebugPrint(F("ComputeMovingAveragesV2Handler: USING MA CALCULATION MODE indexptr:\t"),6);
          DebugPrint(String(indexptr),6);
          DebugPrint(F("\n\n"),6);

          
          for (uint8_t indextype = 0; indextype < 15; indextype++)
          {
              // update using moving average update formula

              DebugPrint(F("ComputeMovingAveragesV2Handler: CALCULATING FOR indextype:\t"),6);
              DebugPrint(String(indextype),6);
              DebugPrint(F("\n"),6);

              int8_t OldestSampleToDropIndex = myCircularBufferValuesStruct.NextWriteIndex - WindowLengths[indexWindows] -1;
              int8_t SampleToAddIndex = myCircularBufferValuesStruct.NextWriteIndex - 1;
              if (SampleToAddIndex < 0)  {SampleToAddIndex += nbvalues;}
              if (OldestSampleToDropIndex < 0) {OldestSampleToDropIndex += nbvalues;}


              DebugPrint(F("ComputeMovingAveragesV2Handler: DROPPING/ADDING VALUE OldestSampleToDropIndex SampletoAddIndex:\t"),6);
              DebugPrint(String(OldestSampleToDropIndex),6);
              DebugPrint(F("\t"),6);

              DebugPrint(String(SampleToAddIndex),6);
              DebugPrint(F("\n"),6);
              
              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = static_cast <uint16_t> (floor(MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] + ( (float) myCircularBufferValuesStruct.PZEMValues[SampleToAddIndex][indextype] - (float) myCircularBufferValuesStruct.PZEMValues[OldestSampleToDropIndex][indextype]) / (float) WindowLengths[indexWindows]));
              
          }


        }
        else
        {


          DebugPrint(F("ComputeMovingAveragesV2Handler: USING CA CALCULATION MODE indexptr:\t"),6);
          DebugPrint(String(indexptr),6);
          DebugPrint(F("\n\n"),6);


          for (uint8_t indextype = 0; indextype < 15; indextype++)
          {

              DebugPrint(F("ComputeMovingAveragesV2Handler: CALCULATING FOR indextype (FillNbValues-1):\t"),6);
              DebugPrint(String(indextype),6);
              DebugPrint(F("\t"),6);
              DebugPrint(String(myCircularBufferValuesStruct.FillNbValues -1),6);
              DebugPrint(F("\n"),6);


              // update using cumulative average update formula
              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype]*(myCircularBufferValuesStruct.FillNbValues -1) + myCircularBufferValuesStruct.PZEMValues[myCircularBufferValuesStruct.FillNbValues - 1][indextype]) / myCircularBufferValuesStruct.FillNbValues;
          }
        }

        DebugPrint(F("ComputeMovingAveragesV2Handler: BREAK\n"),6);
        break;  
      }
    }


    DebugPrint(F("ComputeMovingAveragesV2Handler: STRUCT NOT FOUND for windowLength\n\n"),6);
      
    
    // We couldn't find an existing struct for this windowLength
    // is there any struct that is free ? (WindowLength == 0)
    if (!FoundStruct)
    {
      for(uint8_t indexptr = 0; indexptr<5; indexptr++)
      {

          DebugPrint(F("ComputeMovingAveragesV2Handler: CHECK IF STRUCT IS FREE indexptr:\t"),6);
          DebugPrint(String(indexptr),6);
          DebugPrint(F("\n"),6);



        if(MovingAveragesStructV2ptr[indexptr]->WindowLength == 0)
        {

          DebugPrint(F("ComputeMovingAveragesV2Handler: FOUND FREE STRUCT indexptr:\t"),6);
          DebugPrint(String(indexptr),6);
          DebugPrint(F("\n"),6);

          // use the first free one, mark as used by writing WindowLength
          MovingAveragesStructV2ptr[indexptr]->WindowLength = WindowLengths[indexWindows];

          // find how many samples we can process
          uint8_t NbSamplesToProcess = min(myCircularBufferValuesStruct.FillNbValues,WindowLengths[indexWindows]);
          // find the oldest sample index


          DebugPrint(F("ComputeMovingAveragesV2Handler: NbSamplesToProcess OldestSampleIndex:\t"),6);
          DebugPrint(String(NbSamplesToProcess),6);
          DebugPrint(F("\t"),6);


          int8_t OldestSampleIndex = myCircularBufferValuesStruct.NextWriteIndex - NbSamplesToProcess;

          if (OldestSampleIndex < 0) 
          {
            OldestSampleIndex += nbvalues;
          }


          DebugPrint(String(OldestSampleIndex),6);
          DebugPrint(F("\n\n"),6);


          // work from here upwards
          for (uint8_t indexval = OldestSampleIndex; indexval < OldestSampleIndex + NbSamplesToProcess; indexval++)
          {
            for (uint8_t indextype = 0; indextype < 15; indextype++)
            {

              DebugPrint(F("ComputeMovingAveragesV2Handler: PROCESSING INDEXTYPE (ADD) indextype valueaddress\t:"),6);
              DebugPrint(String(indextype),6);
              DebugPrint(F("\t"),6);

              DebugPrint(String(indexval%nbvalues),6);
              DebugPrint(F("\n"),6);



              //Serial.println("WILL ADD THIS:");
              //Serial.println(myCircularBufferValuesStruct.PZEMValues[indexval%nbvalues][indextype]);


              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] += myCircularBufferValuesStruct.PZEMValues[indexval%nbvalues][indextype];
              //Serial.println("RESULT OF ADD:");
              //Serial.println(MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype]);
              //Serial.flush();
            }
  
          }

          for (uint8_t indextype = 0; indextype < 15; indextype++)
            {
              DebugPrint(F("ComputeMovingAveragesV2Handler: PROCESSING INDEXTYPE (DIVIDE BY NBSAMPLESTOPROCESS): indextype NbSamplesToProcess"),6);
              DebugPrint(String(indextype),6);
              DebugPrint(F("\t"),6);
              DebugPrint(String(NbSamplesToProcess),6);
              DebugPrint(F("\n"),6);

              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] /= NbSamplesToProcess;

/*
              Serial.println("DIVIDING:");
              Serial.println(MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype]);
              Serial.println("BY:");
              Serial.println(NbSamplesToProcess);
  */            
            }
          
          break;  
      
        }
      }
    }
      DebugPrint(F("\nComputeMovingAveragesV2Handler: EXIT INDEXWINDOWS\n"),6);
  }

    DebugPrint(F("\nComputeMovingAveragesV2Handler: EXIT call\n"),6);
  
}

void SelectPZEM(uint8_t PZEMID)
{

  pinMode(ADG333A_IN1_PIN,OUTPUT);
  pinMode(ADG333A_IN4_PIN,OUTPUT);

  switch (PZEMID)
  {
    case 0:

// switching TX line
    digitalWrite(ADG333A_IN1_PIN,HIGH);
// switching RX line
    digitalWrite(ADG333A_IN4_PIN,HIGH);

    break;

    case 1:

// switching TX line
    digitalWrite(ADG333A_IN1_PIN,LOW);
// switching RX line
    digitalWrite(ADG333A_IN4_PIN,LOW);
    break;
    
  }



}
// final code for 3 units (for now we are testing two pzems only)
void SelectPZEM_v2(uint8_t PZEMID)
{

  //ISR_timer.disable(MovingAveragesTimerNumber);
  
  switch (PZEMID)
  {
    case 0:

// switching TX line
    digitalWrite(ADG333A_IN1_PIN,HIGH);
// switching RX line
    digitalWrite(ADG333A_IN3_PIN,HIGH);

    break;
    
    case 1:

// switching TX line
    digitalWrite(ADG333A_IN1_PIN,LOW);
    digitalWrite(ADG333A_IN2_PIN,HIGH);
// switching RX line
    digitalWrite(ADG333A_IN3_PIN,LOW);
    digitalWrite(ADG333A_IN4_PIN,HIGH);
    break;

    case 2:

// switching TX line
    digitalWrite(ADG333A_IN1_PIN,LOW);
    digitalWrite(ADG333A_IN2_PIN,LOW);
// switching RX line
    digitalWrite(ADG333A_IN3_PIN,LOW);
    digitalWrite(ADG333A_IN4_PIN,LOW);
    break;

  }
  
  //ISR_timer.enable(MovingAveragesTimerNumber);

}

void PZEM_resetEnergy(uint8_t PZEMresetflag)
{
  for(uint8_t i=0; i<3; i++)
  {
    if (PZEMresetflag & (1 << i)) 
    {
      SelectPZEM_v2(i);
      pzem.resetEnergy();

    }
  }
}

uint32_t GetProcessingDelayMicros()
{
  static uint32_t prevMicros = 0;
  uint32_t ProcessingDelayMicros =  micros() - prevMicros;
  prevMicros = micros();
  return ProcessingDelayMicros;
}


uint32_t GetProcessingDelay()
{
  static uint32_t prevMillis = 0;
  uint32_t nowMillis = millis();
  uint32_t ProcessingDelay =  nowMillis - prevMillis;
  prevMillis = nowMillis;
  DebugPrint(String(nowMillis),7);
  DebugPrint("\t",7);
  return ProcessingDelay;
}


void FillNowValuesAndRegisters()
{

  long ret = 0;
  
  uint32_t ProcessingCompensatedDelayMicros = 0;
  
  
  uint32_t ProcessingCompensatedDelayMillis = 0;
  uint32_t ProcessingCompensationRemainderDelayMicros = 0;

  for (uint8_t i = 0; i < 3; i++)
  {
    
    DebugPrint(F("FillNowValuesAndRegisters: PZEM:\t"),1);
    DebugPrint(i,1);
    DebugPrint(F("\n"),1);

    /*
    ProcessingCompensatedDelayMicros = GetProcessingDelayMicros();
    ldiv_t result = ldiv(ProcessingCompensatedDelayMicros,1000);
    ProcessingCompensatedDelayMillis = result.quot;
    ProcessingCompensationRemainderDelayMicros = result.rem;
    

    DebugPrint(String(i),0);
    DebugPrint("\t",0);
    DebugPrint(String(ProcessingCompensatedDelayMicros),0);
    DebugPrint("\t",0);
    DebugPrint(String(ProcessingCompensatedDelayMillis),0);
    DebugPrint("\t",0);
    DebugPrint(String(ProcessingCompensationRemainderDelayMicros),0);
    DebugPrint("\n",0);
    */

    ProcessingCompensatedDelayMillis = GetProcessingDelay();
    
    
    DebugPrint(String(i),7);
    DebugPrint("\t",7);
    DebugPrint(String(ProcessingCompensatedDelayMillis),7);
    DebugPrint("\t",7);
    

    delay(333);
    //delay(333 - constrain(ProcessingCompensatedDelayMillis,0,332) - int(bool(ProcessingCompensatedDelayMicros)));
    //delay(333 - constrain(ProcessingCompensatedDelayMillis,0,333));
    
    ProcessingCompensatedDelayMillis = GetProcessingDelay();
    
    DebugPrint(String(ProcessingCompensatedDelayMillis),7);
    DebugPrint("\n",7);
    
    SelectPZEM_v2(i);
    
    float tmpval = 0.f;

    uint16_t voltage = 0;
    uint16_t current = 0;
    uint16_t power = 0;
    uint16_t frequency = 0;
    uint16_t pf = 0;
    uint16_t energy = 0;

    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.voltage();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      voltage = static_cast <uint16_t> (floor(10.f * tmpval)); // value in decivolts, max_value 65535 deciVolts, or 6553.5 Volts 
      break;
    }

    if (isnan(tmpval)) {voltage = 0;} // NaN despite retries.

    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.current();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      current = static_cast <uint16_t> (floor(100.f * tmpval)); // value in centiAmperes, max value 65535 deciAmperes, or 655.35 Amperes
      break;
    }

    if (isnan(tmpval)) {current = 0;} // NaN despite retries.

    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.power();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      power = static_cast <uint16_t> (floor(0.5f + tmpval)); // value in W, max value 65535 W. note we reduce resolution from 0.1W to 1W in order to store on 16 bits
      // TODO : add power_decimal support to store residual of power (so we don't lose resolution from 0.1W to 1W) or add another uint16_t and econde over two uint16_t.
      break;
    }

    if (isnan(tmpval)) {power = 0;} // NaN despite retries.
    
    
    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.frequency();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      frequency = static_cast <uint16_t> (floor(10.f * tmpval)); //  value in deciHertz, max value 65535 deciHertz, or 6553.5 Hertz
      break;
    }

    if (isnan(tmpval)) {frequency = 0;} // NaN despite retries.
    
    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.pf();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      pf = static_cast <uint16_t> (floor(100.f * tmpval)); // value pf*100. pf = 1.0 -> 100.
      break;
    }

    if (isnan(tmpval)) {pf = 0;} // NaN despite retries.
    
    for(uint8_t retry = 0; retry < maxPZEMRetries; retry++)
    {
      tmpval = pzem.energy();
      PZEMReads++;
      if(isnan(tmpval)) {PZEMRetries++; continue;}
      energy = static_cast <uint16_t> (floor(10.f * tmpval)); // value in hectoWh. max value = 65536 hWh = 6553.5 kWh
      break;
    }

    if (isnan(tmpval)) {energy = 0;} // NaN despite retries.
    

    // TODO : overflow management. (like boost narrow-cast) or using pow(2,sizeof(uint_16t)*8) and <template>
    // TODO : overflow flags for each metric
    // TODO : store overflow residual in case of energy reset initiated locally due to uint16_t energy overflow.
    //      : will be grabbed by backend to restore proper energy counter value, and residual will be cleared at this time. 
    // TODO : other metrics should never overflow, as they are above hardware electrical upper limits of PZEM 004t v3.0

    // NOTE :
    // values above do not take into account PZEM limitations (inherent to current/voltage sensing admissible values and PZEM internal data types limitations, which may be substnatially lower than the theoretical values above)
    // check PZEM datasheet to know more.

    // TODO : isochronous sampling. compensate processing delay and remove it from PZEM polling delay.
     

    if(!isnan(voltage)){
        //Serial.print("Voltage: "); Serial.print(voltage); Serial.println("V");
        NowValues[0 + i] = voltage;
    } else {
        //Serial.println("Error reading voltage");
        NowValues[0 + i] = 0;
        Anyerror = true;
    }
    //memcpy(VoltageModbusRegister, &(NowValues[0 + i]), sizeof(VoltageModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(80 + 2*i,NowValues[0 + i]);
    
    //ret = ModbusRTUServer.holdingRegisterWrite(80 + 2*i,VoltageModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(81 + 2*i,VoltageModbusRegister[1]);

      
    //NowValues[3] = pzem.current();
    if(!isnan(current)){
        //Serial.print("Current: "); Serial.print(current); Serial.println("A");
        NowValues[3 + i] = current;
        
    } else {
        //Serial.println("Error reading current");
        NowValues[3 + i] = 0;
        Anyerror = true;
    }
    //memcpy(CurrentModbusRegister, &(NowValues[3 + i]), sizeof(CurrentModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(86 + 2*i,NowValues[3 + i]);

    //ret = ModbusRTUServer.holdingRegisterWrite(86 + 2*i,CurrentModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(87 + 2*i,CurrentModbusRegister[1]);



    //NowValues[6] = pzem.power();
    if(!isnan(power)){
        //Serial.print("Power: "); Serial.print(power); Serial.println("W");
        NowValues[6 + i] = power;
    
    } else {
        //Serial.println("Error reading power");
        NowValues[6 + i] = 0;
        Anyerror = true;
    }
    //memcpy(PowerModbusRegister, &(NowValues[6 + i]), sizeof(PowerModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(92 + 2*i,NowValues[6 +i]);
    
    //ret = ModbusRTUServer.holdingRegisterWrite(92 + 2*i,PowerModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(93 + 2*i,PowerModbusRegister[1]);  
  

    //NowValues[9] = pzem.frequency();
    if(!isnan(frequency)){
        //Serial.print("Frequency: "); Serial.print(frequency, 1); Serial.println("Hz");
        NowValues[9 + i] = frequency;
    
    } else {
        //Serial.println("Error reading frequency");
        NowValues[9 + i] = 0;
        Anyerror = true;
    }
    //memcpy(FrequencyModbusRegister, &(NowValues[9 + i]), sizeof(FrequencyModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(104 + 2*i,NowValues[9 + i]);

    //ret = ModbusRTUServer.holdingRegisterWrite(104 + 2*i,FrequencyModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(105 + 2*i,FrequencyModbusRegister[1]);
   

    //NowValues[12] = pzem.pf();
    if(!isnan(pf)){
        //Serial.print("PF: "); Serial.println(pf);
        NowValues[12 + i] = pf;
    } else {
        //Serial.println("Error reading power factor");
        NowValues[12 + i] = 0;
        Anyerror = true;
    }
    //memcpy(PowerFactorModbusRegister, &(NowValues[12 + i]), sizeof(PowerFactorModbusRegister));
    
    ret = ModbusRTUServer.holdingRegisterWrite(110 + 2*i,NowValues[12 + i]);
    
    //ret = ModbusRTUServer.holdingRegisterWrite(110 + 2*i,PowerFactorModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(111 + 2*i,PowerFactorModbusRegister[1]);


    //NowValues[15] = pzem.energy();
    if(!isnan(energy)){
        //Serial.print("Energy: "); Serial.print(energy,3); Serial.println("kWh");
        NowValues[15 + i] = energy;  
    } else {
        //Serial.println("Error reading energy");
        NowValues[15 + i] = 0;
        Anyerror = true;
    }
    //memcpy(EnergyModbusRegister, &(NowValues[15 + i]), sizeof(EnergyModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(98 + 2*i,NowValues[15 + i]);
    
    //ret = ModbusRTUServer.holdingRegisterWrite(98 + 2*i,EnergyModbusRegister[0]);
    //ret = ModbusRTUServer.holdingRegisterWrite(99 + 2*i,EnergyModbusRegister[1]);
    
  }
    
    
}

void ProcessFormulas()
{

  // COILS FOR TRIP FORMULAS UPDATE

  // coil 1 : formula update requested by client
  // coil 2 : formula digitalwrite HIGH or LOW on formula evaluation returning true
  // coil 3 : formula DRY RUN : does not effect pin status, only reports it (see pin statuses coils)
  // coil 4 : trip recovery strategy, MANUAL or AUTO on condition clear


// HOLDING REGISTERS FOR TELEMETRY :
// holding register 80 to 115 : most recent telemetry
// holding register 116 to 145 : Moving average telemetry for processing by client purposes

// HOLDING REGISTERS FOR TIME SYNCHRONISATION
// holding registers 146,147,148,149 : epoch seconds in 64 bit format (Y2K38 compliant)
// holding register 150 : milliseconds to add to epoch seconds. valid integer values 0 to 999

// HOLDING REGISTERS FOR TRIP FORMULAS UPDATE
// holding register 151 LSB : MA Window in seconds to use for formula evaluation of variables. all variables in the same formula
// are evaluated using the same MA Windows
// holding register 151 MSB : Formula Slot that is being updated
// holding register 152 to 155 : list of pins to trip. each register holds 2 list items (8 bit pin ID). max number of pins : 8
// holding register 156 : trip recovery hysteresis time : duration in seconds condition must remain clear before
// pin(s) reactivation if AUTO MODE.
// holding registers 157 to 188 : formula string, 32 register to hold 64 chars max, including \0
  // convert register content to temporary char array
  // compile and evaluate : if ok
  // write it to flash, use 4K for each formula

  // if no formula update : read sequentially each formula from flash, compile and evaluate.
  // generate action on pins.

  if(ModbusRTUServer.coilRead(1) == 1)
  {
    uint16_t FormulaDataBaseAddress = 151;
    DebugPrint(F("ProcessFormulas: FORMULA UPDATE: indexptr:\t"),5);
    uint16_t MAWindowAndFormulaSlot;
    uint8_t MAWindowAndFormulaSlotArray[2];
    uint8_t indexptr;
    ISR_timer.disable(MovingAveragesTimerNumber);
    MAWindowAndFormulaSlot = ModbusRTUServer.holdingRegisterRead(FormulaDataBaseAddress);
    memcpy(&MAWindowAndFormulaSlotArray,&MAWindowAndFormulaSlot,sizeof(MAWindowAndFormulaSlot));
    indexptr = MAWindowAndFormulaSlotArray[0];
    
    DebugPrint(String(indexptr),5);
    DebugPrint(F("\n"),5);

    if (indexptr < 5)
    {

      TripFormulaDataStructptr[indexptr]->MAWindowSeconds = constrain(MAWindowAndFormulaSlotArray[1],1,nbvalues);
      TripFormulaDataStructptr[indexptr]->HighOrLowOnFormulaTrue = ModbusRTUServer.coilRead(2);
      DebugPrint(F("ProcessFormulas: MAWindowSeconds HighOrLowOnFormulaTrue FormulaChars DryRun isTripRecoveryStrategyAuto TripRecoveryDuration PinsList:\t"),5);
      DebugPrint(String(TripFormulaDataStructptr[indexptr]->MAWindowSeconds),5);
      DebugPrint(F("\t"),5);
      
      DebugPrint(String(TripFormulaDataStructptr[indexptr]->HighOrLowOnFormulaTrue),5);
      DebugPrint(F("\t"),5);
      

      char FormulaChars[64];
      uint16_t FormulaCharsRegisters[32];
      for(uint8_t indexcharregister = 0; indexcharregister < 32; indexcharregister++)
      {
        FormulaCharsRegisters[indexcharregister] = ModbusRTUServer.holdingRegisterRead(FormulaDataBaseAddress + 6 + indexcharregister);
      }

      memcpy(&FormulaChars,&FormulaCharsRegisters,sizeof(FormulaChars));
      
      for(uint8_t charindex = 0; charindex < 64; charindex += 2)
      {
        char tmpchar;
        tmpchar = FormulaChars[charindex];
        FormulaChars[charindex] = FormulaChars[charindex+1];
        FormulaChars[charindex+1] = tmpchar;
      }

      memcpy(&(TripFormulaDataStructptr[indexptr]->FormulaChars),&FormulaChars,sizeof(FormulaChars));
      //TripFormulaDataStructptr[indexptr]->FormulaChars = FormulaChars;


      DebugPrint(String(FormulaChars),5);
      DebugPrint(F("\t"),5);
    


      TripFormulaDataStructptr[indexptr]->FormulaDryRun = ModbusRTUServer.coilRead(3);
      TripFormulaDataStructptr[indexptr]->isTripRecoveryStrategyAuto = ModbusRTUServer.coilRead(4);
      TripFormulaDataStructptr[indexptr]->TripRecoveryDurationSeconds = ModbusRTUServer.holdingRegisterRead(FormulaDataBaseAddress + 5);
      
      DebugPrint(String(TripFormulaDataStructptr[indexptr]->FormulaDryRun),5);
      DebugPrint(F("\t"),5);
    
      DebugPrint(String(TripFormulaDataStructptr[indexptr]->isTripRecoveryStrategyAuto),5);
      DebugPrint(F("\t"),5);

      DebugPrint(String(TripFormulaDataStructptr[indexptr]->TripRecoveryDurationSeconds),5);
      DebugPrint(F("\t"),5);
    



      uint8_t DigitalPinsList[8];
      uint16_t DigitalPinsListRegisters[4];

      for(uint8_t indexpinsregister = 0; indexpinsregister < 4; indexpinsregister++)
      {
        DigitalPinsListRegisters[indexpinsregister] = ModbusRTUServer.holdingRegisterRead(FormulaDataBaseAddress + 1 + indexpinsregister);
      }
      
      memcpy(&DigitalPinsList,&DigitalPinsListRegisters,sizeof(DigitalPinsListRegisters));
    
      for(uint8_t pinindex = 0; pinindex < 8; pinindex += 2)
      {
        uint8_t tmppin;
        tmppin = DigitalPinsList[pinindex];
        DigitalPinsList[pinindex] = DigitalPinsList[pinindex+1];
        DigitalPinsList[pinindex+1] = tmppin;
      }

      memcpy(&(TripFormulaDataStructptr[indexptr]->DigitalPinsList), &DigitalPinsList, sizeof(DigitalPinsList));
  

      
      for(uint8_t indexpins = 0; indexpins < 8; indexpins++)
      {

        DebugPrint(String(TripFormulaDataStructptr[indexptr]->DigitalPinsList[indexpins]),5);
        DebugPrint(F(" "),5);
    
      }

      DebugPrint(F("\n"),5);
    
    }
    else
    {
      // not a valid formula slot
      DebugPrint(F("ProcessFormulas: NOT A VALID FORMULA SLOT\n"),5);
    
    }

    ModbusRTUServer.coilWrite(1,0);
    ISR_timer.enable(MovingAveragesTimerNumber);
    
  }
    
  DebugPrint(F("ProcessFormulas: WILL PARSE FORMULAS\n\n"),5);
    
  // parse_formulas
  for(uint8_t indexptr = 0; indexptr<5; indexptr++)
  {
    // check if formula slot is used
    DebugPrint(F("ProcessFormulas: FORMULA indexptr:\t"),5);
    DebugPrint(indexptr,5);
    DebugPrint(F("\n"),5);    
    
    if(TripFormulaDataStructptr[indexptr]->MAWindowSeconds != 0)
    {
      float result;
      bool bool_result;
      DebugPrint(F("ProcessFormulas: THIS FORMULA SLOT IS USED\n"),5);
      
      EvalLogicalExpression(TripFormulaDataStructptr[indexptr]->MAWindowSeconds,TripFormulaDataStructptr[indexptr]->FormulaChars,result);
      bool_result = bool(result);

      if ((bool_result) && !(TripFormulaDataStructptr[indexptr]->FormulaDryRun))
      {
        // this is not a drill. toggle pins
        DebugPrint(F("ProcessFormulas: NOT A DRILL, TOGGLE PINS:\t"),5);
        TripFormulaDataStructptr[indexptr]->IsTripped = true;
        // reset current trip recovery duration
        TripFormulaDataStructptr[indexptr]->CurrentTripRecoveryDurationSeconds = 0;
          
        for(uint8_t digitalPinsListIndex = 0; digitalPinsListIndex < 8; digitalPinsListIndex++)
        {
          digitalWrite(TripFormulaDataStructptr[indexptr]->DigitalPinsList[digitalPinsListIndex],TripFormulaDataStructptr[indexptr]->HighOrLowOnFormulaTrue);
          DebugPrint(TripFormulaDataStructptr[indexptr]->DigitalPinsList[digitalPinsListIndex],5);
          DebugPrint(F(" "),5);
          DebugPrint(F("\n"),5);
        }
        
        DebugPrint(F("\n"),5);
       
      }
      else if (!bool_result)
      {
        // check if pin has either recovered from trip or was not tripped previously and if we can recover Automatically
        if ((TripFormulaDataStructptr[indexptr]->IsTripped) && (TripFormulaDataStructptr[indexptr]->isTripRecoveryStrategyAuto))
        {

          DebugPrint(F("ProcessFormulas: CAN WE RECOVER?\t"),5);
            
          // since ProcessFormulas() is triggered by an ISR every second, we can increment this way.
          if (++(TripFormulaDataStructptr[indexptr]->CurrentTripRecoveryDurationSeconds) >= (TripFormulaDataStructptr[indexptr]->TripRecoveryDurationSeconds))
          {
            // we can recover from trip, no need to recover if formula is dry run
            if (!TripFormulaDataStructptr[indexptr]->FormulaDryRun)
            {
              DebugPrint(F("YES.\tTOGGLING PINS:\t"),5);
              for(uint8_t digitalPinsListIndex = 0; digitalPinsListIndex < 8; digitalPinsListIndex++)
              {
                digitalWrite(TripFormulaDataStructptr[indexptr]->DigitalPinsList[digitalPinsListIndex],!(TripFormulaDataStructptr[indexptr]->HighOrLowOnFormulaTrue));
                DebugPrint(TripFormulaDataStructptr[indexptr]->DigitalPinsList[digitalPinsListIndex],5);
                DebugPrint(F(" "),5);
                
              }
              DebugPrint(F("\n"),5);

            }
            else
            {
              DebugPrint(F("NO.\tRECOVER NOT NEEDED, DRY RUN\n"),5);
            }
              
          }
          else
          {
            DebugPrint(F("NO.\tRECOVERY WINDOW NOT ELASPED\n"),5);
          }
        
        }
        else
        {
          DebugPrint(F("ProcessFormulas: EITHER NOT TRIPPED, OR MANUAL RECOVERY\n"),5);
        }
            
      }
    }

  }
  
}

void CPL_line_test(uint8_t test_seconds)
{
  for(uint8_t i=0;i<test_seconds;i++)
  {
    Serial1.print(F("1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz\n"));
    Serial1.flush();
    delay(1000);
  }
}

long readVcc() 
{
 
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  //result = 1126400L / result; // Back-calculate AVcc in mV
  // after calibration

  result = 1129400L / result; // Back-calculate AVcc in mV
  result -= 255; 
  return result;

}

uint16_t GetLastEventAddrFromEEPROM()
{

    for (uint16_t i = 12 ; i < EEPROM.length() ; i+=16) 
    {
      if (EEPROM.read(i) == MAX_COUNT_8BIT) 
      {
        return i;
      }
    }
    return 0; // returns 0 if no event is logged.

}

int8_t WriteFrameToEEPROM(uint16_t addr, uint8_t frame[16])
{
  // addr is frame base address (address of first byte of frame)
  if ((addr % 16) != 0) {return -1;} // wrong address. frame addr must be aligned.
  if(frame[12] != MAX_COUNT_8BIT) {return -2;} // wrong endMarker data
  
  uint8_t epoch_is_zero[8] = {0,0,0,0,0,0,0,0}; 
  if(!memcmp(frame,epoch_is_zero,sizeof(epoch_is_zero))) {return -3;} // epoch must be > 0.

  for(uint16_t i=0; i < 16; i++)
  {
    EEPROM.update(addr + i,frame[i]);
  }

  EEPROM.update((addr - 4 + EEPROM.length()) % EEPROM.length(),0); // resets previous last event marker.
  return 0;

}

void logEventToEEPROM(uint16_t eventCode, uint16_t eventData)
{
  uint16_t writeAddr = 0; // frame base address we will write to.

  if(LogEndMarkerAddr == 0)
  {
    //LogEndMarkerAddr initialized at 0 (recent reset?). we have to search for last event in EEPROM and also restore globalEventCounter.
    DebugPrint("LogEndMarkerAddr initialized at 0. Getting last event address from EEPROM\n",0);
    
    LogEndMarkerAddr = GetLastEventAddrFromEEPROM();
    if (LogEndMarkerAddr == 0)
    {
      DebugPrint("Log is empty\n",0);
      globalEventCounter = 0;
    }
    else
    {
      EEPROM.get(LogEndMarkerAddr +1,globalEventCounter);
      DebugPrint("LogEndMarkerAddr\tglobalEventCounter:\n",0);
      DebugPrint(String(LogEndMarkerAddr),0);
      DebugPrint("\t",0);
      DebugPrint(String(globalEventCounter),0);
      DebugPrint("\n",0);   
    }
  }

  globalEventCounter++; // increment before write to EEPROM

  DateTime nowtime = RTClib::now();
  uint32_t epoch = nowtime.unixtime();

  // EEPROM event log frame is 16 bytes, which means 128 events can be logged
  
  // EEPROM event log frame template
  uint8_t frame[16] = {0,0,0,0,0,0,0,0,0,0,0,0,MAX_COUNT_8BIT,0,0,0};
  
  // byte [0] to [7] : Y2K38 compliant unix epoch
  // byte [8] and [9] : eventcode. byte [8] is eventcode MSB, [9] is LSB.
  // byte [10] and [11] : eventdata. byte [10] is eventdata MSB, [11] is LSB.
  // byte [12] : last event marker. set to 255 if part of the last event frame.
  // byte [13] to [14] : global event counter. used by backend to fetch missing event and ensure proper sorting/indexing, as two event may have the same timestamp in epoch 
  // byte [15] : padding byte set to 0 / reserved for future use
  DebugPrint("epoch:\n",0);
  DebugPrint(String(epoch),0);
  DebugPrint("\n",0);

  memcpy(&(frame[0]),&epoch,sizeof(epoch));
  memcpy(&(frame[8]),&eventCode,sizeof(eventCode));
  memcpy(&(frame[10]),&eventData,sizeof(eventData));
  memcpy(&(frame[13]),&globalEventCounter,sizeof(globalEventCounter));
  
  DebugPrint("frame:\n",0);
  for (uint8_t k=0;k<16;k++)
  {
    DebugPrint(String(frame[k]),0);
    DebugPrint("\t",0);
  }
  DebugPrint("\n",0);

  // write strategy : circular logging for wear levelling.
  
  if(LogEndMarkerAddr !=0) 
  {
    writeAddr = (LogEndMarkerAddr+4)%EEPROM.length();
  }
  else
  {
    writeAddr = 0;
  } 

  int8_t ret = WriteFrameToEEPROM(writeAddr,frame);
  if(!ret) 
  {
    LogEndMarkerAddr = (LogEndMarkerAddr+16)%EEPROM.length();
  } // Update End of log marker if write successful.
  else
  {
    // rollback globalEventCounter
    DebugPrint(String(ret),0);
    globalEventCounter--;
  }

}

uint16_t clearEEPROM(bool HardErase)
{
    // HardErase also reset global event counter (if data on EEPROM is not trusted / or need to reset the counter)
    // SoftErase does not resets the globalEventCounter, and logs that EEPROM was erased to the EEPROM as the first event.
    if(!HardErase)
    {
      uint16_t LogEndMarkerAddr = GetLastEventAddrFromEEPROM();
      if(LogEndMarkerAddr != 0)
      {
        EEPROM.get(LogEndMarkerAddr + 1,globalEventCounter);
      }
      else
      {
        globalEventCounter = 0;
      }
      logEventToEEPROM(17,0);
    }
    else
    {
      globalEventCounter = 0;
    }
    

    // usually called by backend or from serial debug to clear all events stored in the EEPROM
    for (uint16_t i = 0 ; i < EEPROM.length() ; i++) 
    {
      EEPROM.write(i, 0);
    }
    LogEndMarkerAddr = EEPROM.length() - 4; // ensures next log write is at base address 0.

}

void DumpEEPROM()
{
    DebugPrint("\n",0);
    for (uint16_t i = 0 ; i < EEPROM.length() ; i+=16) 
    {
      for (uint16_t j = 0 ; j < 16 ; j++) 
      {
        DebugPrint(String(EEPROM.read(i+j)), 0);
        DebugPrint("\t",0);

      }
      DebugPrint("\n",0);
    }
}

int16_t DumpEventLog()
{

  DebugPrint("\n",0);
  bool HasLogCycled = true;

  if(LogEndMarkerAddr == 0)
  {
    LogEndMarkerAddr = GetLastEventAddrFromEEPROM();
    if(LogEndMarkerAddr == 0) {DebugPrint(F("Log is empty.\n"),0);return 0;}  
  }

  DebugPrint("DumpEventLog: LogEndMarkerAddr baseAddr\n",0);
  DebugPrint(String(LogEndMarkerAddr),0);
  DebugPrint("\t",0);

  
  uint16_t baseAddr = (LogEndMarkerAddr + 4) % EEPROM.length();
  
  DebugPrint(String(baseAddr),0);
  DebugPrint("\n",0);
  
  for(uint16_t i=baseAddr;i<baseAddr+EEPROM.length();i+=16)
  {
    uint16_t realindex = i % EEPROM.length();
    uint32_t epoch = 0;
    uint16_t eventcode = MAX_COUNT_16BIT;
    uint16_t eventdata = MAX_COUNT_16BIT;
    
    //DebugPrint(String(realindex),0);
    //DebugPrint("\n",0);
   
    EEPROM.get(realindex,epoch);
    if(epoch == 0)
    {
      if(realindex == baseAddr) 
      {
        DebugPrint("Log has not cycled yet.\n",0);
        HasLogCycled = false;
        break;
      }
      else 
      {
        DebugPrint("Unexpected event log end at address:\t",0);
        DebugPrint(String(realindex),0);
        return 1;
      }
    }
    EEPROM.get(realindex += sizeof(uint64_t),eventcode);
    EEPROM.get(realindex += sizeof(eventcode),eventdata);  
    DebugPrint(F("Event:\t"),0);
    DebugPrint(String(epoch),0); // TODO : Format as human readable datetime
    DebugPrint(F("\t"),0);
    DebugPrint(GetStringFromId(eventcode),0); // TODO : Format as human readable datetime
    DebugPrint(F("\t"),0);
    DebugPrint(String(eventdata),0);
    DebugPrint(F("\n"),0);
  }

  if(!HasLogCycled)
  {
    for(uint16_t i=0;i<baseAddr;i+=16)
    {

      uint32_t epoch = 0;
      uint16_t eventcode = MAX_COUNT_16BIT;
      uint16_t eventdata = MAX_COUNT_16BIT;
    
      //DebugPrint(String(i),0);
      //DebugPrint("\n",0);
      uint16_t readAddr = i;

      EEPROM.get(readAddr,epoch);

      if(epoch == 0)
      {
        DebugPrint("Unexpected event log end at address:\t",0);
        DebugPrint(String(i),0);
        return 1;
      }
      
      EEPROM.get(readAddr += sizeof(uint64_t),eventcode);
      EEPROM.get(readAddr += sizeof(eventcode),eventdata);  
      DebugPrint(F("Event:\t"),0);
      DebugPrint(String(epoch),0); // TODO : Format as human readable datetime
      DebugPrint(F("\t"),0);
      DebugPrint(GetStringFromId(eventcode),0); // TODO : Format as human readable datetime
      DebugPrint(F("\t"),0);
      DebugPrint(String(eventdata),0);
      DebugPrint(F("\n"),0);
    }

  }

  
  DebugPrint("Log end.\n",0);
  return 0;
}


void setup() 
{
  
  wdt_disable();
  //clearEEPROM(true);
  Serial.begin(57600);
  Serial1.begin(9600);
  Wire.begin();
  logEventToEEPROM(0,0); // logging unit start-up
  
  pinMode(ADG333A_IN1_PIN,OUTPUT);
  pinMode(ADG333A_IN2_PIN,OUTPUT);
  pinMode(ADG333A_IN3_PIN,OUTPUT);
  pinMode(ADG333A_IN4_PIN,OUTPUT);

  
  delay(1000);
  DebugPrint(F("setup: Modbus RTU Server Init\n"),0);
  
  //EvalLogicalExpression(5);

  // Tell whether the time is (likely to be) valid
  /*
	if (myRTC.oscillatorCheck()) {
		Serial.print(" O+\n");
	} else {
		Serial.print(" O-\n");
	}

  	// then the date
  bool century = false;
  bool h12Flag;
  bool pmFlag;

	Serial.println(myRTC.getYear(), DEC);
  Serial.println(myRTC.getMonth(century), DEC);
  Serial.println(myRTC.getDate(), DEC);

  Serial.println(myRTC.getHour(h12Flag,pmFlag), DEC);
  Serial.println(myRTC.getMinute(), DEC);
  Serial.println(myRTC.getSecond(), DEC);

	Serial.print("\n");
  */


if (linetest)

{
  CPL_line_test(10);
}

  // GetDeviceAddress first argument is array of pin from lsb to msb, second argument is the pin supplying digital high voltage
  uint8_t DeviceAddressPins[8] = {37,38,39,40,41,42,43,44};
  DeviceAddress = GetDeviceAddress(DeviceAddressPins,36);
  DebugPrint(F("setup: DeviceAddress:\t"),0);
  DebugPrint(String(DeviceAddress),0);
  DebugPrint(F("\n"),0);
  
  
  // start the Modbus RTU server, with device id based on DIP switch encoded address
  if (!ModbusRTUServer.begin(DeviceAddress, 9600)) 
  {
    DebugPrint(F("setup:Failed to start Modbus RTU Server!\n"),0);
    while (1);
  }


  myCircularBufferValuesStruct.FillNbValues = 0;
  myCircularBufferValuesStruct.NextWriteIndex = 0;

  myMovingAveragesStructV2_0.WindowLength = 0;
  myMovingAveragesStructV2_1.WindowLength = 0;
  myMovingAveragesStructV2_2.WindowLength = 0;
  myMovingAveragesStructV2_3.WindowLength = 0;
  myMovingAveragesStructV2_4.WindowLength = 0;

  
  // configure the LED
  //pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, LOW);
  long ret;
  // configure a single coil at address 0x00
  ret = ModbusRTUServer.configureCoils(0, numCoils);
  DebugPrint(F("setup: config coils:\t"),1);
  DebugPrint(String(ret),1);
  DebugPrint(F("\n"),1);
  
  // configure discrete inputs at address 0x40
  ret = ModbusRTUServer.configureDiscreteInputs(40, numDiscreteInputs);
  DebugPrint(F("setup: config discrete inputs:\t "),1);
  DebugPrint(String(ret),1);  
  DebugPrint(F("\n"),1);
  
  // configure holding registers at address 0x80
  ret = ModbusRTUServer.configureHoldingRegisters(80, numHoldingRegisters);
  DebugPrint(F("setup: config holding registers:\t"),1);
  DebugPrint(String(ret),1);
  DebugPrint(F("\n"),1);
  

  // configure input registers at address 0x00
  ret = ModbusRTUServer.configureInputRegisters(200, numInputRegisters);
  DebugPrint(F("setup: config input registers:\t"),1);
  DebugPrint(String(ret),1);
  DebugPrint(F("\n"),1);
  
  FillNowValuesAndRegisters(); // making sure that values are filled before timer5 is started

  ITimer5.init();

  if (ITimer5.attachInterruptInterval(10, TimerHandler))
  {
    DebugPrint(F("setup: Starting  ITimer5 OK, millis() =\t"),1); 
    DebugPrint(String(millis()),1);
    DebugPrint(F("\n"),1);
  }
  else
    DebugPrint(F("setup: Can't set ITimer5. Select another freq. or timer"),1);


  MovingAveragesTimerNumber = ISR_timer.setInterval(1000, ComputeMovingAveragesV2Handler);

  DebugPrint(F("setup: ISR_timer set for Moving averages. Timer number=\t"),1); 
  DebugPrint(String(MovingAveragesTimerNumber),1);
  DebugPrint(F("\n"),1);

  wdt_enable(WDTO_4S);
  logEventToEEPROM(1,0); // logging unit Configure OK. 

}

String FormatIntToDecimal(String value, uint8_t DecimalPlaces)
{
    
    uint8_t len = value.length();
    int8_t lendiff = len - DecimalPlaces;
    String formatted;
    
    if (lendiff > 0)
    {
      formatted.reserve(len + 1);
      formatted = value.substring(0,len - DecimalPlaces) + "." + value.substring(len - DecimalPlaces);
    }
    else
    {
      formatted.reserve(2 + abs(lendiff) + len);
      formatted = "0.";
      while (lendiff < 0) { formatted.concat("0"); lendiff++;}
      formatted.concat(value); 
    }
    return formatted;


}

void PrintNowValues(uint8_t DebugLevel)
{

  DebugPrint(F("\n"),DebugLevel);
  DebugPrint(F("***LAST KNOWN PZEM VALUES***\n\n"),DebugLevel);

  DebugPrint(F("PZEM voltage L1 L2 L3:\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[0]),1),DebugLevel);
  DebugPrint(F(" V\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[1]),1),DebugLevel);
  DebugPrint(F(" V\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[2]),1),DebugLevel);
  DebugPrint(F(" V\n"),DebugLevel);

  
  DebugPrint(F("PZEM current L1 L2 L3:\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[3]),2),DebugLevel);
  DebugPrint(F(" A\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[4]),2),DebugLevel);
  DebugPrint(F(" A\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[5]),2),DebugLevel);
  DebugPrint(F(" A\n"),DebugLevel);

  DebugPrint(F("PZEM power L1 L2 L3:\t"),DebugLevel);
  
  DebugPrint(String(NowValues[6]),DebugLevel);
  DebugPrint(F(" W\t"),DebugLevel);

  DebugPrint(String(NowValues[7]),DebugLevel);
  DebugPrint(F(" W\t"),DebugLevel);

  DebugPrint(String(NowValues[8]),DebugLevel);
  DebugPrint(F(" W\n"),DebugLevel);


  DebugPrint(F("PZEM frequency L1 L2 L3:\t"),DebugLevel);
  
  DebugPrint(FormatIntToDecimal(String(NowValues[9]),1),DebugLevel);
  DebugPrint(F(" Hz\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[10]),1),DebugLevel);
  DebugPrint(F(" Hz\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[11]),1),DebugLevel);
  DebugPrint(F(" Hz\n"),DebugLevel);

  DebugPrint(F("PZEM power factor L1 L2 L3:\t"),DebugLevel);
  
  DebugPrint(FormatIntToDecimal(String(NowValues[12]),2),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[13]),2),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[14]),2),DebugLevel);
  DebugPrint(F("\n"),DebugLevel);

  DebugPrint(F("PZEM energy L1 L2 L3:\t"),DebugLevel);

  
  DebugPrint(FormatIntToDecimal(String(NowValues[15]),1),DebugLevel);
  DebugPrint(F(" kWh\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[16]),1),DebugLevel);
  DebugPrint(F(" kWh\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(NowValues[17]),1),DebugLevel);
  DebugPrint(F(" kWh\n"),DebugLevel);

  DebugPrint(F("5V BUS Voltage:\t"),DebugLevel);

  DebugPrint(String(readVcc()),DebugLevel);
  DebugPrint(F(" mV\n"),DebugLevel);

  
  DebugPrint(F("\nRetries counter: Retries TotalReads PercentErrorRate\t"),DebugLevel);

  DebugPrint(String(PZEMRetries),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(String(PZEMReads),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(String((float) 100.f*PZEMRetries/PZEMReads),DebugLevel);
  DebugPrint(F("\n"),DebugLevel);

}

void PrintAvgValues(uint8_t DebugLevel)
{

  DebugPrint(F("\n"),DebugLevel);
  DebugPrint(F("***AVERAGE PZEM VALUES***\n\n"),DebugLevel);
 
  DebugPrint(F("PZEM voltage L1 L2 L3:\t"),DebugLevel);


  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[0]),1),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[1]),1),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[2]),1),DebugLevel);
  DebugPrint(F("\n"),DebugLevel);


  
  DebugPrint(F("PrintAvgValues: PZEM current L1 L2 L3:\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[3]),2),DebugLevel);
  DebugPrint(F(" A\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[4]),2),DebugLevel);
  DebugPrint(F(" A\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[5]),2),DebugLevel);
  DebugPrint(F(" A\n"),DebugLevel);

  DebugPrint(F("PrintAvgValues: PZEM power L1 L2 L3:\t"),DebugLevel);
  
  DebugPrint(String(myMovingAveragesStructV2_0.MovingAverageValues[6]),DebugLevel);
  DebugPrint(F(" W\t"),DebugLevel);

  DebugPrint(String(myMovingAveragesStructV2_0.MovingAverageValues[7]),DebugLevel);
  DebugPrint(F(" W\t"),DebugLevel);

  DebugPrint(String(myMovingAveragesStructV2_0.MovingAverageValues[8]),DebugLevel);
  DebugPrint(F(" W\n"),DebugLevel);


  DebugPrint(F("PrintAvgValues: PZEM frequency L1 L2 L3:\t"),DebugLevel);
  
  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[9]),1),DebugLevel);
  DebugPrint(F(" Hz\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[10]),1),DebugLevel);
  DebugPrint(F(" Hz\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[11]),1),DebugLevel);
  DebugPrint(F(" Hz\n"),DebugLevel);

  DebugPrint(F("PrintAvgValues: PZEM power factor L1 L2 L3:\t"),DebugLevel);
  

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[12]),2),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[13]),2),DebugLevel);
  DebugPrint(F("\t"),DebugLevel);

  DebugPrint(FormatIntToDecimal(String(myMovingAveragesStructV2_0.MovingAverageValues[14]),2),DebugLevel);
  DebugPrint(F("\n"),DebugLevel);

}

void DumpCircularBuffer(uint8_t DebugLevel)
{
  DebugPrint(F("\n"),DebugLevel);
  for(uint8_t indextype = 0; indextype<15; indextype++)
  {
    for(uint8_t indexval = 0; indexval<20; indexval++)
    {
      DebugPrint(String(myCircularBufferValuesStruct.PZEMValues[indexval][indextype]),DebugLevel);
      DebugPrint(F("\t"),DebugLevel);
    }

      DebugPrint(F("\n"),DebugLevel);
  }
}

void PrintCurrentTime(uint8_t DebugLevel)
{
  DateTime nowtime = RTClib::now();
  time_t epoch = nowtime.unixtime();
  epoch -= UNIX_OFFSET; // localtime_r expects epoch since Jan1 2000 00:00, not Jan1 1970 00:00
  // TODO : check Y2K38 support !! (uint64_t based time_t)
  struct tm* nowtm = localtime(&epoch);
  
  char buffer[64];
  strftime(buffer, 64, "%d %m %Y %H:%M:%S UTC", nowtm);   
  
  DebugPrint("\n",DebugLevel);
  DebugPrint(String(buffer),DebugLevel);
  DebugPrint("\n",DebugLevel);
  
  DebugPrint("epoch (2000 based):\t",DebugLevel);  
  DebugPrint(String(epoch),DebugLevel);
  DebugPrint("\n",0);
  
  DebugPrint("years since 1900:\t",DebugLevel);  
  DebugPrint(nowtm->tm_year,DebugLevel);
  DebugPrint("\n",DebugLevel);
  
  DebugPrint("months since january:\t",DebugLevel);  
  DebugPrint(nowtm->tm_mon,DebugLevel);
  DebugPrint("\n",DebugLevel);

  DebugPrint("day of month:\t",DebugLevel);  
  DebugPrint(nowtm->tm_mday,DebugLevel);
  DebugPrint("\n",DebugLevel);

  DebugPrint("hours:\t",DebugLevel);  
  DebugPrint(nowtm->tm_hour,DebugLevel);
  DebugPrint("\n",DebugLevel);

  DebugPrint("minutes:\t",DebugLevel);  
  DebugPrint(nowtm->tm_min,DebugLevel);
  DebugPrint("\n",DebugLevel);

  DebugPrint("seconds:\t",DebugLevel);  
  DebugPrint(nowtm->tm_sec,DebugLevel);
  DebugPrint("\n",DebugLevel); 
  
}

void processCommand()
{
  
  // get now values
  String strreceivedchars =  String(receivedChars);
  // gnv = get now values, print currently stored PZEM values for all phases, that were fetched by FillNowValuesAndRegisters()
  if (!(strcmp(receivedChars , "gnv")))
  {
    
    PrintNowValues(0);
  }
  else if (!(strcmp(receivedChars , "gav")))
  {
    
    PrintAvgValues(0);
  }
  else if (!(strcmp(receivedChars , "dcb")))
  {
    DumpCircularBuffer(0);
  }
  else if (!(strcmp(receivedChars , "dl")))
  {
    int16_t ret = 0;
    ret = DumpEventLog();
  }
  else if (!(strcmp(receivedChars , "dee")))
  {
    int16_t ret = 0;
    DumpEEPROM();
  }
  
  
  // resets PZEM energy counters for the specified phases as a binary flag representation. little endian. 
  // ex: rste 001 : resets L1 energy counter, rste 110 : resets L2 and L3 energy counters.
  else if(strreceivedchars.startsWith("rste") && strreceivedchars.length() == 8)
  {
    String strPZEMbits = strreceivedchars.substring(5);
    uint8_t PZEMbits = (uint8_t) strtoul(strPZEMbits.c_str(),nullptr,2);
    PZEM_resetEnergy(PZEMbits);
  }
  else if(strreceivedchars.startsWith("time"))
  {
    if(strreceivedchars.length() == 4) {PrintCurrentTime(0);}
    else
    {
      myRTC.setClockMode(false); // use 24h mode
      String strEpoch = strreceivedchars.substring(5);
      time_t epoch = strtoul(strEpoch.c_str(),nullptr,10);
      myRTC.setEpoch(epoch);
      PrintCurrentTime(0);
    }
     
  }
  // CPL line test
  else if (!(strcmp(receivedChars , "lnt")))
  {
    DebugPrint(F("starting CPL Line test."),0);
    //pinMode(22,OUTPUT);
    //digitalWrite(22,HIGH);
    CPL_line_test(30);
    //digitalWrite(22,LOW);
    DebugPrint(F("end CPL Line test."),0);
    
  }
  else
  {
    DebugPrint(F("Invalid command"),0);
  }
  

}


void loop() 
{

  wdt_reset(); //watchdog reset


  // DEBUG DELAY COMPENSATION
  //ISR_timer.disable(MovingAveragesTimerNumber);
    
  
  //delay(1000);

  // poll for serial command
  recvWithEndMarker(':');

  if(newData)
  {
    newData = false;
    processCommand();    
  }
  

  // poll for Modbus RTU requests
  static uint32_t millisAtSyncedEpoch;
  static uint64_t epochSeconds;
  static uint16_t epochMillis;

  ModbusRTUServer.poll();
  //delay(100);
  //Serial.println(millis());
  
  //Serial.println(ModbusRTUServer.coilRead(0));
  if(ModbusRTUServer.coilRead(0) == 1)
  {
    // client just put the unit into time sync mode
    ISR_timer.disable(MovingAveragesTimerNumber);
    uint32_t timeoutStartMillis;
    uint16_t epochSecondsRegisters[4]; // 4 words store the epoch seconds (2038 bug compliant)
    uint16_t millisAtSyncedEpochRegisters[2]; // store the internal millis clock when time sync was performed
  

    DebugPrint(F("loop: Ready for clock sync"),1);
    DebugPrint(F("\n"),1);
    timeoutStartMillis = millis();
    while(true)
    {
      if ((millis() - timeoutStartMillis) > timeoutSyncMillis) 
      {
        DebugPrint(F("loop: time sync timeout"),1);
        DebugPrint(F("\n"),1);
        break;
      }
      
      wdt_reset();
      ModbusRTUServer.poll();
      uint8_t addr;
      if(ModbusRTUServer.coilRead(0) == 0)
      {
        DebugPrint(F("loop: time received"),1);
        DebugPrint(F("\n"),1);
        
        uint8_t k;
        for(k=0;k<4;k++)
        {
          addr = 146 + k;
          epochSecondsRegisters[k] = ModbusRTUServer.holdingRegisterRead(addr);
        }

        addr++;
        epochMillis = ModbusRTUServer.holdingRegisterRead(addr);
        memcpy(&epochSeconds,epochSecondsRegisters,sizeof(epochSeconds));
        epochSeconds += sync_offset;
        sync_time = (unsigned long) epochSeconds;
        
        myRTC.setClockMode(false); // use 24h mode 
        myRTC.setEpoch(sync_time);
        PrintCurrentTime(5);
        

        DebugPrint(F("loop: epochregisters:\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[0]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[1]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[2]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[3]),5);
        DebugPrint(F("\n"),5);
        
        DebugPrint(F("loop: epochseconds (1970 based):\t"),0);
        DebugPrint(String(long(epochSeconds)),0);
        DebugPrint(F("\n"),0);
        
        
        DebugPrint(F("loop: year month day hour minute second d:\t"),1);

        DebugPrint(String(year(sync_time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(month(sync_time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(day(sync_time)),1);     
        DebugPrint(F("\t"),1);

        DebugPrint(String(hour(sync_time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(minute(sync_time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(second(sync_time)),1);
        DebugPrint(F("\n"),1);

        // time sync OK.
        logEventToEEPROM(8,0); // logging successful time sync.

        DebugPrint(F("loop: Enabling Moving Averages Timer"),1);
        DebugPrint(F("\n"),1);

        ISR_timer.enable(MovingAveragesTimerNumber);
    
        break;
        
      }
      
      DebugPrint(F("loop: Writing time sync stamp to Modbus Registers"),1);
      DebugPrint(F("\n"),1);
                
      memcpy(millisAtSyncedEpochRegisters,&millisAtSyncedEpoch,sizeof(millisAtSyncedEpochRegisters));
      ModbusRTUServer.inputRegisterWrite(200,millisAtSyncedEpochRegisters[0]);
      ModbusRTUServer.inputRegisterWrite(201,millisAtSyncedEpochRegisters[1]);
      
    }
  }

  FillNowValuesAndRegisters();
  FillAverageValuesRegisters();
  ProcessFormulas();
  PrintNowValues(2);
  
  Anyerror = false;

}


