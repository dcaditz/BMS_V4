
#include <Arduino.h>
#include <stdint.h>
#include <SoftwareSerial.h>
//#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC68042.h"
#include "Average.h"


#define TOTAL_IC  1            // Number of ICs in the isoSPI network LTC6804-2 ICs must be addressed in ascending order starting at 0.

/***** Pack and sensor characteristics *****/
const float MAX_CURRENT = 5000000000.;          // Maximum battery current(amps) before relay opens
const float MAX_TEMP  = 50. ;                   // Maximum pack temperature (deg C) before relay opens
const float LEM_RANGE =  50.;                   // Rated range of LEM hall sensor.
const float MIN_CELL_V = 2.20;                  // Minimum allowable cell voltage. Depends on battery chemistry.
const float MAX_CELL_V = 3.60;                  // Maximum allowable cell voltage. Depends on battery chemistry.
const float CELL_BALANCE_THRESHOLD_V = 3.3;     // Cell balancing occurrs when voltage is above this value

/***** Xbee serial *****/
#define xbRxD 4
#define xbTxD 5
SoftwareSerial XbeeSerial(xbRxD, xbTxD);

/******** Arduino pin definitions ********/
int chargeRelayPin = 8;               // Relay output for overcharge conditions
int dischargeRelayPin = 9;            // Relay output for undercharge conditions
int currentPin = A0;                  // LEM Input should be Vcc/2 + I * 1.667 / LEM_RANGE
int currentBiasPin = A1;              // For comparing with LEM output (currentPin) since Vcc/2 may change as aux battery discharges.
int tempPins[] = {A2};                // Array of arduino pins used for temperature sensor inpout


/********  Variables for tracking cell voltages and states ***************/
int overCharge_state = LOW;           // Over charge state. HIGH = relay on, LOW = relay off
int underCharge_state = LOW;          // Under charge state. HIGH = relay on, LOW = relay off
int overTemp_state = LOW;             // Over temperature state. HIGH = relay on, LOW = relay off
int overCurrent_state = LOW;          // Over current state. HIGH = relay on, LOW = relay off
int chargeRelay_state;
int dischargeRelay_state;
int cellMax_i;                        // Temporary variable for holding index of cell with max voltage
int cellMin_i;                        // Temporary variable for holding index of cell with min voltage
float  cellMin_V;                     // Temporary variable for holding  min measured cell voltage
float  cellMax_V;                     // Temporary variable for holding  max measured cell voltage
float minV1 ;
float maxV1 ;


/********   Current and temperature variables ***********************/
const uint16_t imax = 100;                 // Size of arrays for averaging read measurements
Average<float> lemHistory(imax);
Average<float> lemBiasHistory(imax);
float lem = 0;
float lemBias = 0;
float lemZeroCal = 0;
float current = 0;
float temp[sizeof(tempPins)];

int error = 0;
unsigned long  tstart;


/******************************************************
  Global Battery Variables received from 6804 commands
  These variables store the results from the LTC6804
  register reads and the array lengths must be based
  on the number of ICs on the stack
 ******************************************************/


uint16_t cell_codes[TOTAL_IC][12];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][6];
/*!<
  The GPIO codes will be stored in the aux_codes[][6] array in the following format:

  |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
  |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6804 configuration data that is going to be written
  to the LTC6804 ICs on the daisy chain. The LTC6804 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:

  |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
  |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
  |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6804-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

  |rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
  |---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
  |IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/



/*!**********************************************************************
  \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  pinMode(chargeRelayPin, OUTPUT);
  pinMode(dischargeRelayPin, OUTPUT);
  pinMode(currentPin, INPUT);
  pinMode(currentBiasPin, INPUT);
  for (int i = 0; i < sizeof(tempPins) / sizeof(int); i++)
  {
    pinMode(tempPins[i], INPUT);
  }
  digitalWrite(dischargeRelayPin, LOW); // turn off relays during setup
  digitalWrite(chargeRelayPin, LOW);    // turn off relays during setup

  overCharge_state = HIGH;
  underCharge_state = HIGH;

  Serial.begin(9600);
  XbeeSerial.begin(9600);

  LTC6804_initialize();  //Initialize LTC6804 hardware
  init_cfg();            //initialize the 6804 configuration array to be written
  delay(1000);
  lemZeroCal = zeroCurrentCalibrate();   // Calibrates LEM sensor at zero current
  overCurrent_state = HIGH;
  tstart = millis();
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{


  //  while (overCurrent_state == LOW) {
  //    Serial.println("RESET TO CONTINUE");
  //    delay(10000);
  //  }

  // read current:
  //overCurrent_state = HIGH;
  current = readCurrent();


  // read temperatures:
  overTemp_state = HIGH;
  for (int i = 0; i < sizeof(tempPins) / sizeof(int); i++)
  {
    temp[i] = (analogRead(tempPins[i]) * 5. / 1024 - 0.5) / 0.01;
    if (temp[i] > MAX_TEMP) {
      overTemp_state = LOW;
      Serial.println("OVER TEMPERATURE STATE DETECTED.");
    }
  }

  // read cells:
  wakeup_idle();
  LTC6804_adcv(); // do cell AD conversion and fill cell registers
  delay(10);
  wakeup_idle();
  error = LTC6804_rdcv(0, TOTAL_IC, cell_codes); // read cell voltages from registers

  if (error == -1)
  {
    Serial.println("A PEC error was detected in the received data");
  }

  // print to serial outputs:
  print_cells();

  // test for over charge/undercharge states:
  minV1 = MIN_CELL_V;
  maxV1 = MAX_CELL_V;

  if (overCharge_state == LOW) { // add hysteresis
    maxV1 = maxV1 - .2;
  }

  if (underCharge_state == LOW) { // add hysteresis
    minV1 = minV1 + .2;
  }

  // get maximum and minimum cells:
  cellMax_i = -1;
  cellMin_i = -1;
  cellMin_V = 100.;
  cellMax_V = 0.;
  for (int i = 0; i < 12; i++)
  {
    float V = cell_codes[0][i] * 0.0001;
    if (V < cellMin_V) {
      cellMin_V = V;
      cellMin_i = i;
    }
    if (V > cellMax_V) {
      cellMax_V = V;
      cellMax_i = i;
    }
  }

  underCharge_state = HIGH;
  overCharge_state = HIGH;
  overCurrent_state = HIGH;

  if (cellMin_V <= minV1)
  {
    underCharge_state = LOW;
    // Serial.println("V <= MIN_CELL_V");
  }
  if (cellMax_V >= maxV1)
  {
    overCharge_state = LOW;
    //Serial.println("V >= MAX_CELL_V");
  }
  if (abs(current) > MAX_CURRENT) {
    overCurrent_state = LOW;
  }
  // set relay states:

  chargeRelay_state = overCurrent_state &&  underCharge_state && overCharge_state &&  overTemp_state ;
  dischargeRelay_state =  overCurrent_state &&  overCharge_state && underCharge_state &&  overTemp_state;
  digitalWrite(chargeRelayPin, chargeRelay_state  );
  digitalWrite(dischargeRelayPin, dischargeRelay_state);

  //while (chargeRelay_state == LOW || dischargeRelay_state== LOW ) {
  //  Serial.println("RESET TO CONTINUE");
  //  delay(10000);
  //}

  //  if (abs(current) > MAX_CURRENT) {
  //    overCurrent_state = LOW;
  //    digitalWrite(chargeRelayPin,  overCurrent_state  );
  //    digitalWrite(dischargeRelayPin,  overCurrent_state );
  //    Serial.println("OVER CURRENT STATE DETECTED. PRESS RESET TO CONTINUE");
  //    delay(10000);
  //  } else {
  //    overCurrent_state = HIGH;
  //    //    chargeRelay_state = overCharge_state &&  overTemp_state ;
  //    //    dischargeRelay_state =  underCharge_state &&  overTemp_state;
  //    chargeRelay_state = underCharge_state && overCharge_state &&  overTemp_state ;
  //    dischargeRelay_state =  overCharge_state && underCharge_state &&  overTemp_state;
  //    digitalWrite(chargeRelayPin, chargeRelay_state  );
  //    digitalWrite(dischargeRelayPin, dischargeRelay_state);
  //  }

  //
  //  if (underCharge_state == LOW ) {
  //    Serial.println("UNDER VOLTAGE STATE DETECTED.");
  //    digitalWrite(dischargeRelayPin, dischargeRelay_state);
  //  }
  //
  //  if (overCharge_state == LOW ) {
  //    Serial.println("OVER VOLTAGE STATE DETECTED.");
  //    digitalWrite(chargeRelayPin, chargeRelay_state  );
  //  }


  // take advantage of open relay to recalibrate LEM zero current setting:
  if (underCharge_state == LOW or overCharge_state == LOW ) {
    lemZeroCal = zeroCurrentCalibrate();
  }


  //  cell balancing:
  //  Turn on switch Sx for highest cell x if voltage is above threshold
  //  Note: DCP is set to 0 in initialize() This turns off discharge when cell voltages are read.
  // set values in tx_cfg


  //cellMax_i = 5;
  if (cellMax_V >= CELL_BALANCE_THRESHOLD_V)
  {
    balance_cfg(0, cellMax_i);
    //Serial.print("Balance ");
    //Serial.println(cellMax_i);
  } else {
    balance_cfg(0, -1);
  }

  // write tx_cfg to LTC6804. This sets the LTC6804 DCCx registers which control the S pins for balancing:
  LTC6804_wrcfg( TOTAL_IC, tx_cfg);

  delay(5000);

}



/*!***********************************
  \brief Initializes the configuration array
 **************************************/
void init_cfg()
{
  for (int i = 0; i < TOTAL_IC; i++)
  {
    tx_cfg[i][0] = 0xFE;
    //tx_cfg[i][1] = 0x04 ;
    tx_cfg[i][1] = 0x4E1 ; // 2.0V
    //tx_cfg[i][2] = 0xE1 ;
    tx_cfg[i][2] = 0x8CA; // 3.6V
    tx_cfg[i][3] = 0x00 ;
    tx_cfg[i][4] = 0x00 ; // discharge switches  0->off  1-> on.  S0 = 0x01, S1 = 0x02, S2 = 0x04, 0x08, 0x10, 0x20, 0x40, 0x80
    // tx_cfg[i][5] = 0x00 ;
    tx_cfg[i][5] = 0x20 ; // sets the software timer to 1 minute
  }
}



/*!***********************************
  \brief sets  the configuration array for cell balancing
  uses CFGR4 and lowest 4 bits of CGFR5
 **************************************/
void balance_cfg(int ic, int cell)
{
  tx_cfg[ic][4] = 0x00; // clears S1-8
  tx_cfg[ic][5] = tx_cfg[ic][5]  & 0xF0; // clears S9-12 and sets software timer to 1 min
  //Serial.println(tx_cfg[ic][5] & 0xF0,BIN);
  if (cell >= 0 and cell <= 7) {
    tx_cfg[ic][4] = tx_cfg[ic][4] | 1 << cell;
  }
  if ( cell > 7) {
    tx_cfg[ic][5] = tx_cfg[ic][5] | ( 1 << (cell - 8));
  }
}


/*!************************************************************
  \brief Prints Cell Voltage Codes to the serial port
 *************************************************************/
void print_cells()
{
  unsigned long elasped = millis()  - tstart;
  float moduleV;
  serialPrint(elasped);   //ELAPSED TIME:
  
  //INDIVIDUAL CELL VOLTAGES:
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    moduleV = 0.;
    for (int i = 0; i < 12; i++)
    {
      moduleV = moduleV + cell_codes[current_ic][i] * 0.0001;
      serialPrint(cell_codes[current_ic][i] * 0.0001);     
    }
  }  
  serialPrint(moduleV);      // TOTAL MODULE VOLTAGE:
  serialPrint(current);     //MODULE CURRENT:
  
  //TEMPERATURES:
  for (int i = 0; i < sizeof(tempPins) / sizeof(int) ; i++)
  {
    serialPrint( temp[i]);
  }

  //RELAY STATES:
  serialPrint( chargeRelay_state);
  serialPrint( dischargeRelay_state);
  
  serialPrint("\r\n");

}


/*!****************************************************************************
  \brief print function overloads:
 *****************************************************************************/
void serialPrint(String val)
{
  Serial.print(val);
  Serial.print("\t");
  XbeeSerial.print(val);
  XbeeSerial.print("\t");
}

void serialPrint(unsigned long val)
{
  Serial.print(val);
  Serial.print("\t");
  XbeeSerial.print(val);
  XbeeSerial.print("\t");
}


void serialPrint(double val)
{
  Serial.print(val, 4);
  Serial.print("\t");
  XbeeSerial.print(val, 4);
  XbeeSerial.print("\t");
}

void serialPrint(int val)
{
  Serial.print(val);
  Serial.print("\t");
  XbeeSerial.print(val);
  XbeeSerial.print("\t");
}



/*!****************************************************************************
  \brief Prints GPIO Voltage Codes and Vref2 Voltage Code onto the serial port
 *****************************************************************************/
void print_aux()
{

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 5; i++)
    {
      Serial.print(" GPIO-");
      Serial.print(i + 1, DEC);
      Serial.print(":");
      Serial.print(aux_codes[current_ic][i] * 0.0001, 4);
      Serial.print(",");
    }
    Serial.print(" Vref2");
    Serial.print(":");
    Serial.print(aux_codes[current_ic][5] * 0.0001, 4);
    Serial.println();
  }
  Serial.println();
}
/*!******************************************************************************
  \brief Prints the Configuration data that is going to be written to the LTC6804
  to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;

  Serial.println("Written Configuration: ");
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(": ");
    Serial.print("0x");
    serial_print_hex(tx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][5]);
    Serial.print(", Calculated PEC: 0x");
    cfg_pec = pec15_calc(6, &tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(", 0x");
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/*!*****************************************************************
  \brief Prints the Configuration data that was read back from the
  LTC6804 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println("Received Configuration ");
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(": 0x");
    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][5]);
    Serial.print(", Received PEC: 0x");
    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println();
  }
  Serial.println();
}

void serial_print_hex(uint8_t data)
{
  if (data < 16)
  {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}


/*!***********************************
  \brief Reads current input from LEM sensor
 **************************************/
float  readCurrent() {
  for (int i = 0 ; i < imax; i++) {
    lem = lemHistory.rolling(analogRead(currentPin));
    lemBias = lemBiasHistory.rolling(analogRead(currentBiasPin));
  }
  current =  ((lem - lemBias / 2. - lemZeroCal) * 3. * 5. / 1024 *  LEM_RANGE / 1.667) * 8.9 / 8.0;// assumes lem and bias are on 10k/5k voltage divider. Calibration fudge factor added.
  if (abs(current) < .2) current = 0;
  return current;
}
/*!***********************************
  \brief Reads  LEM sensor value when current is zero. Used to calibrates to zero output for zero current
 **************************************/
float  zeroCurrentCalibrate() {
  // get initial readings for current and set the zero calibration
  int iSample = 500;
  for (int i = 0 ; i < iSample; i++) {
    lemHistory.push(analogRead(currentPin));
    lemBiasHistory.push(analogRead(currentBiasPin));
  }
  lem = lemHistory.mean();
  lemBias = lemBiasHistory.mean();
  return  lem - lemBias / 2.;
}
