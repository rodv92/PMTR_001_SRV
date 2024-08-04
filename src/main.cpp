//CREDITS / INCLUDES

#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <ModbusRTUServer.h>
#include <ModbusServer.h>
#include <PZEM004Tv30.h>
#include <TimeLib.h>
#include <expr.h>

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#define USE_TIMER_1     false
#define USE_TIMER_2     false
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     true
#include <TimerInterrupt.h>
#include "ISR_Timer.h"
#include <SimpleTimer.h>              // https://github.com/schinken/SimpleTimer

ISR_Timer ISR_timer;

// The following Arduino pins are ued to switch serial2 signals to each of the 3 PZEMs using the ADG333A
// Quad SPDT Switch
#define ADG333A_IN1_PIN 32
#define ADG333A_IN2_PIN 33
#define ADG333A_IN3_PIN 34
#define ADG333A_IN4_PIN 35

PZEM004Tv30 pzem(&Serial2);
time_t time;

bool linetest = false;
const uint16_t timeoutSyncMillis = 20000;
const int ledPin = LED_BUILTIN;
uint16_t VoltageModbusRegister[2] = {0};
uint16_t CurrentModbusRegister[2] = {0};
uint16_t PowerModbusRegister[2] = {0};
uint16_t EnergyModbusRegister[2] = {0};
uint16_t FrequencyModbusRegister[2] = {0};
uint16_t PowerFactorModbusRegister[2] = {0};

uint16_t AvgVoltageModbusRegister[2] = {0};
uint16_t AvgCurrentModbusRegister[2] = {0};
uint16_t AvgPowerModbusRegister[2] = {0};
uint16_t AvgFrequencyModbusRegister[2] = {0};
uint16_t AvgPowerFactorModbusRegister[2] = {0};

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

double NowValues[18]; // contains most recent values read from PZEM. used as storage since PZEM cannot be queried inside ISR because it uses Serial.

struct MovingAveragesStructV2
{
  uint8_t WindowLength;
  double MovingAverageValues[15];

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
  double PZEMValues[nbvalues][15]; //struct storing values history for each metric except energy
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

bool Anyerror = false;
uint8_t DeviceAddress;

uint8_t DebugLevel = 2;

void DebugPrint(String DebugStr, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {Serial.print(DebugStr);
  Serial.flush();}
}

void DebugPrint(uint8_t DebugNum, uint8_t myDebugLevel)
{
  if (myDebugLevel <= DebugLevel)
  {Serial.print(DebugNum);
  Serial.flush();}
}

uint8_t myPowofTwo (uint8_t p) {
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



void EvalLogicalExpression(uint8_t MAWindowSeconds, char Expression[64], double &result)
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
  struct expr_var_list vars = {0};
  
  // voltage
  struct expr_var *L1U = expr_var(&vars, "L1U", 3);
  L1U->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[0];
  struct expr_var *L2U = expr_var(&vars, "L2U", 3);
  L2U->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[5];
  struct expr_var *L3U = expr_var(&vars, "L3U", 3);
  L3U->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[10];

  // current
  struct expr_var *L1I = expr_var(&vars, "L1I", 3);
  L1I->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[1];
  struct expr_var *L2I = expr_var(&vars, "L2I", 3);
  L2I->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[6];
  struct expr_var *L3I = expr_var(&vars, "L3I", 3);
  L3I->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[11];

  // power
  struct expr_var *L1P = expr_var(&vars, "L1P", 3);
  L1P->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[2];
  struct expr_var *L2P = expr_var(&vars, "L2P", 3);
  L2P->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[7];
  struct expr_var *L3P = expr_var(&vars, "L3P", 3);
  L3P->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[12];

  // energy. always use NowValues
  struct expr_var *L1e = expr_var(&vars, "L1e", 3);
  L1e->value = NowValues[5];
  struct expr_var *L2e = expr_var(&vars, "L2e", 3);
  L2e->value = NowValues[11];
  struct expr_var *L3e = expr_var(&vars, "L3e", 3);
  L3e->value = NowValues[17];

  // frequency
  struct expr_var *L1f = expr_var(&vars, "L1f", 3);
  L1f->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[3];
  struct expr_var *L2f = expr_var(&vars, "L2f", 3);
  L2f->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[8];
  struct expr_var *L3f = expr_var(&vars, "L3f", 3);
  L3f->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[13];

  // power factor
  struct expr_var *L1pf = expr_var(&vars, "L1pf", 4);
  L1pf->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[4];
  struct expr_var *L2pf = expr_var(&vars, "L2pf", 4);
  L2pf->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[9];
  struct expr_var *L3pf = expr_var(&vars, "L3pf", 4);
  L3pf->value = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[14];


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

    long ret;
    memcpy(AvgVoltageModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[0]), sizeof(AvgVoltageModbusRegister));
    memcpy(AvgCurrentModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[1]), sizeof(AvgVoltageModbusRegister));
    memcpy(AvgPowerModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[2]), sizeof(AvgVoltageModbusRegister));
    memcpy(AvgFrequencyModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[3]), sizeof(AvgVoltageModbusRegister));
    memcpy(AvgPowerFactorModbusRegister, &(myMovingAveragesStructV2_0.MovingAverageValues[4]), sizeof(AvgVoltageModbusRegister));
    
    //L1U Avg
    ret = ModbusRTUServer.holdingRegisterWrite(116,AvgVoltageModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(117,AvgVoltageModbusRegister[1]);
    //L2U Avg
    ret = ModbusRTUServer.holdingRegisterWrite(118,0);
    ret = ModbusRTUServer.holdingRegisterWrite(119,0);
    //L3U Avg
    ret = ModbusRTUServer.holdingRegisterWrite(120,0);
    ret = ModbusRTUServer.holdingRegisterWrite(121,0);

    //L1I Avg
    ret = ModbusRTUServer.holdingRegisterWrite(122,AvgCurrentModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(123,AvgCurrentModbusRegister[1]);
    //L2I Avg
    ret = ModbusRTUServer.holdingRegisterWrite(124,0);
    ret = ModbusRTUServer.holdingRegisterWrite(125,0);
    //L3I Avg
    ret = ModbusRTUServer.holdingRegisterWrite(126,0);
    ret = ModbusRTUServer.holdingRegisterWrite(127,0);
  
    //L1P Avg
    ret = ModbusRTUServer.holdingRegisterWrite(128,AvgPowerModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(129,AvgPowerModbusRegister[1]);
    //L2P Avg
    ret = ModbusRTUServer.holdingRegisterWrite(130,0);
    ret = ModbusRTUServer.holdingRegisterWrite(131,0);
    //L3P Avg
    ret = ModbusRTUServer.holdingRegisterWrite(132,0);
    ret = ModbusRTUServer.holdingRegisterWrite(133,0);
  

    //L1f Avg
    ret = ModbusRTUServer.holdingRegisterWrite(134,AvgFrequencyModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(135,AvgFrequencyModbusRegister[1]);
    //L2f Avg
    ret = ModbusRTUServer.holdingRegisterWrite(136,0);
    ret = ModbusRTUServer.holdingRegisterWrite(137,0);
    //L3f Avg
    ret = ModbusRTUServer.holdingRegisterWrite(138,0);
    ret = ModbusRTUServer.holdingRegisterWrite(139,0);
    
    //L1pf Avg
    ret = ModbusRTUServer.holdingRegisterWrite(140,AvgPowerFactorModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(141,AvgPowerFactorModbusRegister[1]);
    //L2pf Avg
    ret = ModbusRTUServer.holdingRegisterWrite(142,0);
    ret = ModbusRTUServer.holdingRegisterWrite(143,0);
    //L3pf Avg
    ret = ModbusRTUServer.holdingRegisterWrite(144,0);
    ret = ModbusRTUServer.holdingRegisterWrite(145,0);
    

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

  
    myCircularBufferValuesStruct.PZEMValues[myCircularBufferValuesStruct.NextWriteIndex][IndexType] = NowValues[IndexType];

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
        MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = 0.0;
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
              

              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] + (myCircularBufferValuesStruct.PZEMValues[SampleToAddIndex][indextype] - myCircularBufferValuesStruct.PZEMValues[OldestSampleToDropIndex][indextype])/WindowLengths[indexWindows];
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
              MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype] = (MovingAveragesStructV2ptr[indexptr]->MovingAverageValues[indextype]*(myCircularBufferValuesStruct.FillNbValues -1) + myCircularBufferValuesStruct.PZEMValues[myCircularBufferValuesStruct.FillNbValues - 1][indextype])/myCircularBufferValuesStruct.FillNbValues;
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

  pinMode(ADG333A_IN1_PIN,OUTPUT);
  pinMode(ADG333A_IN2_PIN,OUTPUT);
  pinMode(ADG333A_IN3_PIN,OUTPUT);
  pinMode(ADG333A_IN4_PIN,OUTPUT);


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
}

void FillNowValuesAndRegisters()
{

  long ret = 0;
  for (uint8_t i = 0; i < 3; i++)
  {
    DebugPrint(F("FillNowValuesAndRegisters: PZEM:\t"),1);
    DebugPrint(i,1);
    DebugPrint(F("\n"),1);

    SelectPZEM_v2(i);
    delay(1000);
    float voltage = pzem.voltage(); 
    float current = pzem.current();
    float power = pzem.power();
    float frequency = pzem.frequency();
    float pf = pzem.pf();
    float energy = pzem.energy();
    

    if(!isnan(voltage)){
        //Serial.print("Voltage: "); Serial.print(voltage); Serial.println("V");
        NowValues[0 + i] = voltage;
    } else {
        //Serial.println("Error reading voltage");
        NowValues[0 + i] = 0.0;

        Anyerror = true;
    }
    memcpy(VoltageModbusRegister, &(NowValues[0 + i]), sizeof(VoltageModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(80 + 2*i,VoltageModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(81 + 2*i,VoltageModbusRegister[1]);

      
    //NowValues[3] = pzem.current();
    if(!isnan(current)){
        //Serial.print("Current: "); Serial.print(current); Serial.println("A");
        NowValues[3 + i] = current;
        
    } else {
        //Serial.println("Error reading current");
        NowValues[3 + i] = 0.0;
        Anyerror = true;
    }
    memcpy(CurrentModbusRegister, &(NowValues[3 + i]), sizeof(CurrentModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(86 + 2*i,CurrentModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(87 + 2*i,CurrentModbusRegister[1]);



    //NowValues[6] = pzem.power();
    if(!isnan(power)){
        //Serial.print("Power: "); Serial.print(power); Serial.println("W");
        NowValues[6 + i] = power;
    
    } else {
        //Serial.println("Error reading power");
        NowValues[6 + i] = 0.0;
        Anyerror = true;
    }
    memcpy(PowerModbusRegister, &(NowValues[6 + i]), sizeof(PowerModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(92 + 2*i,PowerModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(93 + 2*i,PowerModbusRegister[1]);  
  

    //NowValues[9] = pzem.frequency();
    if(!isnan(frequency)){
        //Serial.print("Frequency: "); Serial.print(frequency, 1); Serial.println("Hz");
        NowValues[9 + i] = frequency;
    
    } else {
        //Serial.println("Error reading frequency");
        NowValues[9 + i] = 0.0;
        Anyerror = true;
    }
    memcpy(FrequencyModbusRegister, &(NowValues[9 + i]), sizeof(FrequencyModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(104 + 2*i,FrequencyModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(105 + 2*i,FrequencyModbusRegister[1]);
   

    //NowValues[12] = pzem.pf();
    if(!isnan(pf)){
        //Serial.print("PF: "); Serial.println(pf);
        NowValues[12 + i] = pf;
    } else {
        //Serial.println("Error reading power factor");
        NowValues[12 + i] = 0.0;
        Anyerror = true;
    }
    memcpy(PowerFactorModbusRegister, &(NowValues[12 + i]), sizeof(PowerFactorModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(110 + 2*i,PowerFactorModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(111 + 2*i,PowerFactorModbusRegister[1]);


    //NowValues[15] = pzem.energy();
    if(!isnan(energy)){
        //Serial.print("Energy: "); Serial.print(energy,3); Serial.println("kWh");
        NowValues[15 + i] = energy;  
    } else {
        //Serial.println("Error reading energy");
        NowValues[15 + i] = 0.0;
        Anyerror = true;
    }
    memcpy(EnergyModbusRegister, &(NowValues[15 + i]), sizeof(EnergyModbusRegister));

    ret = ModbusRTUServer.holdingRegisterWrite(98 +2*i,EnergyModbusRegister[0]);
    ret = ModbusRTUServer.holdingRegisterWrite(99 +2*i,EnergyModbusRegister[1]);
    

  }
    
    // L3I
    ret = ModbusRTUServer.holdingRegisterWrite(90,0);
    ret = ModbusRTUServer.holdingRegisterWrite(91,0);
    
    // L3P
    ret = ModbusRTUServer.holdingRegisterWrite(96,0);
    ret = ModbusRTUServer.holdingRegisterWrite(97,0);  
   
    // L3E
    ret = ModbusRTUServer.holdingRegisterWrite(102,0);
    ret = ModbusRTUServer.holdingRegisterWrite(103,0);
    
    // L3f
    ret = ModbusRTUServer.holdingRegisterWrite(108,0);
    ret = ModbusRTUServer.holdingRegisterWrite(109,0);
    
    // L3pf
    ret = ModbusRTUServer.holdingRegisterWrite(114,0);
    ret = ModbusRTUServer.holdingRegisterWrite(115,0);
    
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
      double result;
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


void setup() 
{
  
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  DebugPrint(F("setup: Modbus RTU Server Init\n"),0);
  
  //EvalLogicalExpression(5);


if (linetest)

{
    while(true)
    {
      DebugPrint(F("setup: 1234567890abcdefghijklmnopqrstuvwxyz\n"),0);
      delay(1000);
    }
}

  // GetDeviceAddress first argument is array of pin from lsb to msb, second argument is the pin supplying digital high voltage
  uint8_t DeviceAddressPins[8] = {37,38,39,40,41,42,43,44};
  DeviceAddress = GetDeviceAddress(DeviceAddressPins,36);
  DebugPrint(F("setup: DeviceAddress:\t"),0);
  DebugPrint(String(DeviceAddress),0);
  DebugPrint(F("\n"),1);
  
  
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


}


void loop() {
  // poll for Modbus RTU requests
  delay(1000);
//  Serial.println("before poll");

  static uint32_t millisAtSyncedEpoch;
  static uint64_t epochSeconds;
  static uint16_t epochMillis; 
  ModbusRTUServer.poll();
  //delay(100);
  //Serial.println(millis());
  
  //Serial.println(ModbusRTUServer.coilRead(0));
  if(ModbusRTUServer.coilRead(0) == 1)
  {
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
        time = epochSeconds + 1;
        
        DebugPrint(F("loop: epochregisters:\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[0]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[1]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[2]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[3]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(String(epochSecondsRegisters[4]),5);
        DebugPrint(F("\t"),5);
        
        DebugPrint(F("loop: epochseconds:\t"),2);
        Serial.println(long(epochSeconds));
        
        
        DebugPrint(F("loop: year month day hour minute second d:\t"),1);

        DebugPrint(String(year(time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(month(time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(day(time)),1);     
        DebugPrint(F("\t"),1);

        DebugPrint(String(hour(time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(minute(time)),1);
        DebugPrint(F("\t"),1);

        DebugPrint(String(second(time)),1);
        DebugPrint(F("\n"),1);

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
  ProcessFormulas();

 
  DebugPrint(F("loop: PZEM voltage L1 L2 L3:\t"),2);

  DebugPrint(String(NowValues[0]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[1]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[2]),2);
  DebugPrint(F("\n"),2);

  
  DebugPrint(F("loop: PZEM current L1 L2 L3:\t"),2);

  DebugPrint(String(NowValues[3]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[4]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[5]),2);
  DebugPrint(F("\n"),2);

  DebugPrint(F("loop: PZEM power L1 L2 L3:\t"),2);
  
  DebugPrint(String(NowValues[6]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[7]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[8]),2);
  DebugPrint(F("\n"),2);


  DebugPrint(F("loop: PZEM frequency L1 L2 L3:\t"),2);
  
  DebugPrint(String(NowValues[9]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[10]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[11]),2);
  DebugPrint(F("\n"),2);

  DebugPrint(F("loop: PZEM power factor L1 L2 L3:\t"),2);
  
  DebugPrint(String(NowValues[12]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[13]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[14]),2);
  DebugPrint(F("\n"),2);

  DebugPrint(F("loop: PZEM energy L1 L2 L3:\t"),2);

  
  DebugPrint(String(NowValues[15]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[16]),2);
  DebugPrint(F("\t"),2);

  DebugPrint(String(NowValues[17]),2);
  DebugPrint(F("\n"),2);
  


  Anyerror = false;

}


