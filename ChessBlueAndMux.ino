#include <SoftwareSerial.h>
#include <stdio.h>

//=================================== rfid_RDM6300 +++
const int BUFFER_SIZE = 14; // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_TAG_SIZE = 8; // 8byte tag
SoftwareSerial ssrfid = SoftwareSerial(0, 1);
uint8_t buffer[BUFFER_SIZE]; // used to store an incoming data frame 
int buffer_index = 0;
unsigned long previous_tag = 0L;
//=================================== rfid_RDM6300 ---

//=================================== Bluetooth_HC_05 +++
// name "CHESS_BOARD", pass 1234
SoftwareSerial HC_05(7, 8); // RX | TX (D8, D7)

const long baudRate = 9600; 
char c = ' ';

byte outputData;
byte inputData;
//=================================== Bluetooth_HC_05 ---

int btnTimerDelay;
int btnTimerValue;
int btnTimerDelayCounter;
int btnTimerPressedCounter;

#define blackLed (14)     // A0
#define rfidEN3 (15)      // A1
#define rfidEN4 (16)      // A2 - it is last white line and first black line.
#define rfidEN1 (17)      // A3
#define rfidEN2 (18)      // A4
#define whiteLed (19)     // A5
// ote that not all Arduino board variants bring out A6 and A7.
#define btnTimer (20)     // A6
#define batteryLevel (21) // A7


//16-Channel MUX (74HC4067) Interface
//=================================== Multiplexer +++
#define EN 6 // Multiplexer: on - 0, off - 1

int S[4] = {5, 4, 3, 2}; // пины селектора мультиплексора
int MUXtable[16][4]=
{
  {0,0,0,0}, {1,0,0,0}, {0,1,0,0}, {1,1,0,0},
  {0,0,1,0}, {1,0,1,0}, {0,1,1,0}, {1,1,1,0},
  {0,0,0,1}, {1,0,0,1}, {0,1,0,1}, {1,1,0,1},
  {0,0,1,1}, {1,0,1,1}, {0,1,1,1}, {1,1,1,1}
};

#define signalA 9
#define signalB 10
#define signalC 11
#define signalD 12

int currentPosition[64];
int mux1PositionMap[16] =
{
  8, 0, 12, 4, 10, 2, 14, 6, 9, 1, 13, 5, 11, 3, 15, 7
};

int mux2PositionMap[16] =
{
  24, 16, 28, 20, 26, 18, 30, 22, 25, 17, 29, 21, 27, 19, 31, 23
};

int mux3PositionMap[16] =
{
  40, 32, 44, 36, 42, 34, 46, 38, 41, 33, 45, 37, 43, 35, 47, 39
};

int mux4PositionMap[16] =
{
  56, 48, 60, 52, 58, 50, 62, 54, 57, 49, 61, 53, 59, 51, 63, 55
};

bool muxANewValue;
bool muxBNewValue;
bool muxCNewValue;
bool muxDNewValue;

int currentMuxChanel;
//=================================== Multiplexer ---

void setup()
 {

  for(int i=0; i<64; i++) {
    currentPosition[i] = 0;
  }

  muxANewValue = false;
  muxBNewValue = false;
  muxCNewValue = false;
  muxDNewValue = false;

  btnTimerDelay           = 0;
  btnTimerValue           = 0;
  btnTimerPressedCounter  = 0;
  btnTimerDelayCounter    = 0;
  
  pinMode(btnTimer, INPUT);
  pinMode(batteryLevel, INPUT);
  pinMode(whiteLed, OUTPUT);
  pinMode(blackLed, OUTPUT);
  pinMode(rfidEN1, OUTPUT);
  pinMode(rfidEN2, OUTPUT);
  pinMode(rfidEN3, OUTPUT);
  pinMode(rfidEN4, OUTPUT);

  pinMode(signalA, INPUT);
  pinMode(signalB, INPUT);
  pinMode(signalC, INPUT);
  pinMode(signalD, INPUT);

  Serial.begin(baudRate); // serial port PC
  HC_05.begin(baudRate);  //Default Baud for comm, it may be different for your Module. 
  Serial.println("The bluetooth is ready to pairing!");

//=================================== rfid_RDM6300 +++
 ssrfid.begin(9600);
 ssrfid.listen();
//=================================== rfid_RDM6300 ---

//=================================== multiplexar +++
  pinMode(rfidEN1, OUTPUT);
  
  pinMode(EN, OUTPUT); // пин управления вкл/выкл mux
  
  for(int i=0; i<4; i++) {
    pinMode(S[i], OUTPUT);
  }
  
  currentMuxChanel  = 0;

  // MUX: LOW - on, HIGH - off
  //digitalWrite(EN, LOW);
  digitalWrite(rfidEN1, HIGH);
  digitalWrite(rfidEN2, HIGH);
  digitalWrite(rfidEN3, LOW);
  digitalWrite(rfidEN4, HIGH); 

  digitalWrite(blackLed, LOW);
  digitalWrite(whiteLed, LOW);
  
//=================================== multiplexar ---

}

void loop()
{

// Timer btn action +++  
  btnTimerValue = analogRead(btnTimer);
  if (btnTimerValue > 255) {
    btnTimerDelayCounter = btnTimerDelayCounter >= 99 ? 11 : btnTimerDelayCounter + 1;

    // Very important to READ analog, because it is ANALOG PIN!
    if (btnTimerDelayCounter == 1) {
      Serial.print("Timer button is pressed and value is "); Serial.println(btnTimerValue);      

      currentMuxChanel = currentMuxChanel >= 15 ? 0 : currentMuxChanel + 1;
      Serial.print("Mux chanel is "); Serial.println(currentMuxChanel);
      
      btnTimerPressedCounter = btnTimerPressedCounter >= 2 ? 0 : btnTimerPressedCounter + 1; 
      digitalWrite(blackLed, btnTimerPressedCounter == 1);
      digitalWrite(whiteLed, btnTimerPressedCounter == 2);
    }
  } else {
    btnTimerDelay = btnTimerDelay >= 11 ? 0 : btnTimerDelay + 1;
    if (btnTimerDelay == 0) {
      btnTimerDelayCounter = btnTimerDelay;      
    }
  }
// Timer btn action ---

//=================================== multiplexarSteps +++
selection(currentMuxChanel);

// mux reading +++
int muxA = digitalRead(signalA);
int muxB = digitalRead(signalB);
int muxC = digitalRead(signalC);
int muxD = digitalRead(signalD);

muxANewValue = false;
muxBNewValue = false;
muxCNewValue = false;
muxDNewValue = false;

muxANewValue = (currentPosition[ mux1PositionMap[currentMuxChanel] ] != muxA);
if (muxANewValue) {  
  currentPosition[ mux1PositionMap[currentMuxChanel] ] = muxA;
  if (muxA == 1) {
    sendNewPosition(mux1PositionMap[currentMuxChanel], muxA);
  }
}


muxBNewValue = (currentPosition[ mux2PositionMap[currentMuxChanel] ] != muxB);
if (muxBNewValue) {
  currentPosition[ mux2PositionMap[currentMuxChanel] ] = muxB;
  if (muxB == 1) {
    sendNewPosition(mux2PositionMap[currentMuxChanel], muxB);
  }
}

muxCNewValue = (currentPosition[ mux3PositionMap[currentMuxChanel] ] != muxC);
if (muxCNewValue) {
  currentPosition[ mux3PositionMap[currentMuxChanel] ] = muxC;
  if (muxC == 1) {
    sendNewPosition(mux3PositionMap[currentMuxChanel], muxC);
  }
}

muxDNewValue = (currentPosition[ mux4PositionMap[currentMuxChanel] ] != muxD);
if (muxDNewValue) {
  currentPosition[ mux4PositionMap[currentMuxChanel] ] = muxD;
  if (muxD == 1) {
    sendNewPosition(mux4PositionMap[currentMuxChanel], muxD);
  }
}

// mux reading ---
  
//currentMuxChanel = currentMuxChanel >= 15 ? 0 : currentMuxChanel + 1;
//delay(1000);

//=================================== multiplexarSteps ---

//=================================== rfid_RDM6300 +++
  if (ssrfid.available() > 0) {
    bool call_extract_tag = false;

    int ssvalue = ssrfid.read(); // read 
    if (ssvalue == -1) { // no data was read
      return;
    }
    if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
      buffer_index = 0;
    } else if (ssvalue == 3) { // tag has been fully transmitted       
      call_extract_tag = true; // extract tag at the end of the function call
    }
    if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
      Serial.println("Error: Buffer overflow detected!");
      return;
    }

    buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer
    if (call_extract_tag == true) {
      if (buffer_index == BUFFER_SIZE) {
        
        unsigned tag = extract_tag();
        
        //Serial.println(tag);
        if(previous_tag == tag){
          // it is mine: I commented it
          // Serial.println("Same tag ===================");  
        } else {

          previous_tag = tag;  // it is mine
          Serial.print(currentMuxChanel); Serial.print(":  "); Serial.println(tag); // it is mine
          
         
          // something is wrong... start again looking for preamble (value: 2)
          buffer_index = 0;
          return;
         }
        }    
} else{  
  // Serial.println("NO Cards detected ++++++++++++++++++++++++++++++++++");  
  // it is mine: I commented it
  //previous_tag = 0L; // reset previous tag (if it is has no commented then reading all time else only one!)
  }
}
//=================================== rfid_RDM6300 ---

//=================================== Bluetooth +++
 
// get data from some device
if (HC_05.available()) {
    //inputData = HC_05.read();
    //Serial.write(inputData);
    
    //String inputStr = HC_05.readString();
    //inputStr.trim();

  c = HC_05.read();
  if (c == '1') { // var c - has everyone symbol from send text, example: if you send 321 then c == '1' is true!
//=================================== BatteryLevel +++
  // Vref = 4.1 * 1024 / analogRead(battery); 4.1 - V(источника)
  // Номиналы резисторов, измеренные мультиметром:
  const float R1 = 50400.0; // 50k
  const float R2 = 9880.0;  // 10k
  const float divider = R2/(R1 + R2); // Коэффициент передачи
  const float Vref = 4.15; // Опорное напряжение, Vref = V(источника) * 1024 / analogRead(batteryLevel);

  float v = (float) (analogRead(batteryLevel) * Vref / 1024) / divider;
  HC_05.print(v);
  Serial.println(v);
//=================================== BatteryLevel ---
  }
  c == ' ';
}

/*
  // Arduino send data for Bluetooth
  if (Serial.available())
  {
    outputData = Serial.read();    
    sendbyteDataForBluetooth(outputData);
  }
*/
//=================================== Bluetooth ---

}

//=================================== multiplexar +++
void selection(int j) {
  digitalWrite(EN, HIGH); // выключаем mux на время установки 
  
  digitalWrite(S[0], MUXtable[j][0]);
  digitalWrite(S[1], MUXtable[j][1]);
  digitalWrite(S[2], MUXtable[j][2]);
  digitalWrite(S[3], MUXtable[j][3]);
 
  digitalWrite(EN, LOW); // включаем mux после установки
}
//=================================== multiplexar ---

//=================================== Bluetooth +++
void sendbyteDataForBluetooth(byte _data) {
  HC_05.write(_data);
}

void sendCharsDataForBluetooth(char* _data) {
  HC_05.write(_data);
}

void sendNewPosition(int cellPosition, int cellValue) {

HC_05.print(cellValue);    // 1:true, 0:false
HC_05.print(cellPosition); // number of cell

Serial.println("cell " + String(cellPosition) + ": " + String(cellValue));
}
//=================================== Bluetooth ---

//=================================== rfid_RDM6300 +++
unsigned extract_tag() {
uint8_t msg_head = buffer[0];
uint8_t *msg_data = buffer + 1; // 10 byte => data contains 2byte version + 8byte tag
uint8_t *msg_data_version = msg_data;
uint8_t *msg_data_tag = msg_data + 2;
uint8_t *msg_checksum = buffer + 11; // 2 byte
uint8_t msg_tail = buffer[13];

long tag = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);

return tag;
}

long hexstr_to_value(char *str, unsigned int length) { // converts a     hexadecimal value (encoded as ASCII string) to a numeric value
  char* copy = malloc((sizeof(char) * length) + 1); 
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; 
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol(copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  free(copy); // clean up 
 return value;
}
//=================================== rfid_RDM6300 ---
