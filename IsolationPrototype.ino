/*
 * TesLorean
 * Prototype Code for the ISO165C voltage and isolation monitoring
 * 2019
 * Jeff Cooke
 
 Instructions
 - 'p n' where n is 0..4  // Sends the predefined frame number n immediately once
 - 's id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'r n nnnn' start repeating predefined frame n every nnnn miliseconds
 - 'r n 0' stop repeating predefined frame n
 - 'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop
 - '?' to reshow the instructions

ISO165C Testing Protocol
 - Do Once
    - c 1   // start CAN frame capture
    - p 4   // Close the HV1 negative relay
    - p 10  // Close the HV1 positive relay
 - Set up repeating request
    - r 6 500   // Request HV1 voltage every 1/2 second
    - r 7 500   // Request HV2 voltage every 1/2 second
    - r 13 500 //  Request Insulation kOhms every second
 - Close out
    - r 6 0   // Stop HV1 voltage
    - r 7 0   // Stop HV2 voltage
    - r 13 0   // Stop Insulation kOhms
    - c 0

 Coding Todos
 - 

 Coding Notes
 - The ISO165C runs CAN at 500kbps (not 250kbps as in the documentation)
 - The Negative and Positive HV1 relays must be closed before a valid voltage will be read on HV1
 - Irrespective of the frequency of requests, the HV1 and HV2 voltages only update once per second
 - When initially connected to HV power, the HV1 reading will trend up towards the real value over 2-3 seconds
 - Frame 0x037 reports insulation kOhms and fault flags every second without requests

 //check
*/

// a=target variable, b=bit number to act upon 0-n
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

struct can_frame {
    uint32_t  can_id;
    uint8_t   can_dlc;
    uint8_t   data[8];
};

// Variables for the CAN traffic
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// Timestamp for stay alone message
unsigned long timekeepalive;

//#include <mcp2515.h>
#include <mcp_can.h>
#include <SPI.h>

// CAN board Chip Select (CS) pins
#define TESLOREAN_CAN_CS 10
#define TESLOREAN_CAN_INT 2

// DEFINE
#define StoredFrames 50

// Array for preprepared CAN frames
can_frame CANFrames[StoredFrames];

// Counters tracking the add point and read point in the circular array
volatile uint8_t addPointFrames = 0;
volatile uint8_t readPointFrames = 0;
volatile uint16_t netAddReadCount = 0;   // Adds increment, Reads decrement, only Read if <> 0
volatile bool activeAccess = false;

// DEFINE
#define PreDefinedFrames 20

// Array for preprepared CAN frames
can_frame CANPkg[PreDefinedFrames];
unsigned long CANDurs[PreDefinedFrames];       // Delay duration between frame sends
unsigned long CANTimes[PreDefinedFrames];       // Last time that a frame was sent

// Flags
bool CANread = false;     // Indicates if CAN frames should be captured and output to Serial as CSV

// Define the debug variable
#define debug 1

// Create the CAN object for transmission
MCP_CAN can0(TESLOREAN_CAN_CS);      // TESLOREAN CAN - CS on digital pin 9/10

struct can_frame frame;

void setup()
{
  //// SERIAL PORT
  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  #ifdef debug 
    Serial.println("Initialized serial port (115200 baud)");
  #endif

  // init the SPI communications
  SPI.begin();

  // Prepare the interrupt pins
  pinMode(TESLOREAN_CAN_INT, INPUT);

  // Startup CAN  Battery bus
  can0.begin(CAN_500KBPS);
  #ifdef debug
    Serial.println("Test CANbus 0 initialized");
  #endif

  // Initialize the array pointers
  addPointFrames = 0;
  readPointFrames = 0;
  netAddReadCount = 0;
  
  // Output the instructions for frames
  outputInstructions();
  timekeepalive = millis();

  //// Populate a number of pre-built CAN frames

  // ISO165C Control
  buildCANframe(CANPkg[0],0x022,5,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Self Test - Overall
  buildCANframe(CANPkg[1],0x022,5,0xC8,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : IMC Reset

  buildCANframe(CANPkg[2],0x022,5,0xCA,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Unlock
  buildCANframe(CANPkg[3],0x022,5,0xCB,0x01,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Measurement - Enable
  buildCANframe(CANPkg[4],0x022,5,0xD2,0x00,0x00,0x01,0x00,0x00,0x00,0x00); // Request : Set HV Relay - Negative Relay - Close
  
  buildCANframe(CANPkg[5],0x022,5,0x37,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : IMC Status
  buildCANframe(CANPkg[6],0x022,5,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : HV1
  buildCANframe(CANPkg[7],0x022,5,0x3A,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : HV2
  buildCANframe(CANPkg[8],0x022,5,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : VIFC Status

  buildCANframe(CANPkg[9],0x022,5,0xCA,0x01,0xFF,0xFF,0x00,0x00,0x00,0x00); // Request : Lock
  buildCANframe(CANPkg[10],0x022,5,0xD2,0x01,0x00,0x01,0x00,0x00,0x00,0x00); // Request : Set HV Relay - Positive Relay - Close
  buildCANframe(CANPkg[11],0x022,5,0xD2,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Set HV Relay - Negative Relay - Open
  buildCANframe(CANPkg[12],0x022,5,0xD2,0x01,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Set HV Relay - Positive Relay - Open
  buildCANframe(CANPkg[13],0x022,5,0x35,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Request : Insulation resistance
  buildCANframe(CANPkg[14],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[15],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[16],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[17],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[18],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[19],0x000,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  
  // Initialize the timers for the pre-built frames
  for (int t=0; t < PreDefinedFrames; t++)
  {
    CANDurs[t] = 0;        // Zero denotes an inactive timer
    CANTimes[t] = 0;       // init to 0 so always triggers immediately if set
  }

  Serial.println("Ready...");
}  // end of Setup

// Print out the instructions
void outputInstructions()
{
  Serial.println("Instructions");  
  Serial.println("'p n' where n is 0..4  // Sends the predefined frame number n immediately once");
  Serial.println("'s id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN");
  Serial.println("'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN");
  Serial.println("'r n nnnn' start repeating predefined frame n every nnnn miliseconds");
  Serial.println("'r n 0' stop repeating predefined frame n");
  Serial.println("'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop");
  Serial.println("'?' to reshow the instructions");
  unsigned long time;
  time = millis();
  Serial.print("Time : ");
  Serial.println(time);
  delay(1000);  
}

// Populate the CAN frame with the data specified
// When less that 8 bytes in frame, just fill out remainder with whatever...
void buildCANframe(can_frame & frame, uint16_t cid, uint8_t bcnt, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,  uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  frame.can_id = cid;
  frame.can_dlc = bcnt;
  frame.data[0] = b0;
  frame.data[1] = b1;
  frame.data[2] = b2;
  frame.data[3] = b3;
  frame.data[4] = b4;
  frame.data[5] = b5;
  frame.data[6] = b6;
  frame.data[7] = b7;
}

// Send the frame to the Serial port
void outputCANframe(can_frame & frame)
{
  // Send frame to the Serial Port
  Serial.print("CAN frame : 0x");
  Serial.print(frame.can_id,HEX);
  Serial.print(" - ");
  for (int i=0;i<frame.can_dlc;i++)
  {
    Serial.print(" 0x");
    Serial.print(frame.data[i],HEX);
  }
  Serial.println();  
}

// For some frames (i.e. warnings) output a full text explanation
uint8_t interpretCANframe(can_frame & frame)
{
  // Temp vars
  uint8_t b0,b1,b2,b3,b4,b5,b6,b7;
  bool frameidknown;
  frameidknown = false;

  if (frame.can_dlc > 0){b0 = frame.data[0];}
  if (frame.can_dlc > 1){b1 = frame.data[1];}
  if (frame.can_dlc > 2){b2 = frame.data[2];}
  if (frame.can_dlc > 3){b3 = frame.data[3];}
  if (frame.can_dlc > 4){b4 = frame.data[4];}
  if (frame.can_dlc > 5){b5 = frame.data[5];}
  if (frame.can_dlc > 6){b6 = frame.data[6];}
  if (frame.can_dlc > 7){b7 = frame.data[7];}

  switch (frame.can_id)
  {
    case 0x037: // IMD_Info - sent out every 1s
    {
      Serial.print("IMD: Info [");
      // b0-1 D_IMC_R_ISO : 0-50,000 insulation resistance in kOhms
      uint16_t InsulKOhms = ((uint16_t)b1 << 8) + b0;
      Serial.print("Insulation ");
      Serial.print(InsulKOhms);
      Serial.print(" kOhms, ");
      // b2-3 D_IMC_STATUS : bt0 Insulation fault, bt1 Chassis fault, bt2 System failure, bt3 Calibration running, bt4 Self Testing, bt5 Insulation warning
      bool InsulFault = BIT_CHECK(b2,0);
      bool ChassisFault = BIT_CHECK(b2,1);
      bool SystemFault = BIT_CHECK(b2,2);
      bool CalibRunning = BIT_CHECK(b2,3);
      bool SelfTesting = BIT_CHECK(b2,4);
      bool InsulWarning = BIT_CHECK(b2,5);
      if (InsulFault || ChassisFault || SystemFault)
      {
        Serial.print("Faults: ");
        if (InsulFault){Serial.print("Insulation ");}
        if (ChassisFault){Serial.print("Chassis ");}
        if (SystemFault){Serial.print("System ");}
      }
      if (CalibRunning || SelfTesting)
      {
        Serial.print("Running: ");
        if (CalibRunning){Serial.print("Calibration ");}
        if (SelfTesting){Serial.print("Self Test ");}
      }
      if (InsulWarning){Serial.print("Warning: Insulation ");}
      
      // b4-5 D_VIFC_STATUS : bt0 Insulation measurement 1=disabled, bt4 VIFC Command 1=error
      bool InsulStatus = BIT_CHECK(b4,0);
      bool CommandError = BIT_CHECK(b4,4);
      if (InsulStatus){Serial.print("Disabled: Insulation measurement ");}
      if (CommandError){Serial.print("Error: Command ");}
      
      Serial.println("]");
      frameidknown = true;
      break;
    }
    case 0x023: // IMD_Response
    {
      Serial.println("IMD Response [");

      // b0 = 0xFF <- Error
      if (b0 == 0xFF)
      {
        Serial.print("Error detected ");
      }

      // b0 = 0x37
      // b1-2 D_IMC_STATUS : bt0 Insulation fault, bt1 Chassis fault, bt2 System failure, bt3 Calibration running, bt4 Self Testing, bt5 Insulation warning
      if (b0 == 0x37)
      {
        bool InsulFault = BIT_CHECK(b2,0);
        bool ChassisFault = BIT_CHECK(b2,1);
        bool SystemFault = BIT_CHECK(b2,2);
        bool CalibRunning = BIT_CHECK(b2,3);
        bool SelfTesting = BIT_CHECK(b2,4);
        bool InsulWarning = BIT_CHECK(b2,5);
        if (InsulFault || ChassisFault || SystemFault)
        {
          Serial.print("Faults: ");
          if (InsulFault){Serial.print("Insulation ");}
          if (ChassisFault){Serial.print("Chassis ");}
          if (SystemFault){Serial.print("System ");}
        }
        if (CalibRunning || SelfTesting)
        {
          Serial.print("Running: ");
          if (CalibRunning){Serial.print("Calibration ");}
          if (SelfTesting){Serial.print("Self Test ");}
        }
        if (InsulWarning){Serial.print("Warning: Insulation ");}        
      }

      //b0 = 0x35
      // b1-2 D_IMC_R_ISO : 0 to 50,000 KOmhs
      // b3 D_IMC_R_ISO_BIAS : 0 = unknown, 1 = fault on HV1_Neg, 2 = fault on HV1_Pos
      // b4 D_IMC_R_ISO_CNT : 0-255 number of measurements
      if (b0 == 0x35)
      {
        uint16_t InsulKOhms = ((uint16_t)b2 << 8) + b1;
        Serial.print("Insulation ");
        Serial.print(InsulKOhms);
        Serial.print(" kOhms, ");        
      }
      // if (b3 == 0){Serial.print("Bias: Unknown ");} // or No Bias detected
      if (b3 == 1){Serial.print("Bias: Fault on HV1 Neg ");}
      if (b3 == 2){Serial.print("Bias: Fault on HV1 Pos ");}

      //b0 = 0x36
      // b1-2 D_IMC_HV_1 : 0-600V
      if (b0 == 0x36)
      {
        uint16_t HV1voltage = ((uint16_t)b2 << 8) + b1;
        Serial.print("HV1 ");
        Serial.print(HV1voltage);
        Serial.print(" Volts ");          
      }
      
      //b0 = 0x3A
      // b1-2 D_IMC_HV_2 : 0-600V
      if (b0 == 0x3A)
      {
        uint16_t HV2voltage = ((uint16_t)b2 << 8) + b1;
        Serial.print("HV2 ");
        Serial.print(HV2voltage);
        Serial.print(" Volts ");          
      }
      
      Serial.println("]");
      frameidknown = true;
      break;
    }
    default:
    {
      // if nothing else matches, do the default
      break;
    }
   }

   if (frameidknown)
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

void sendMessage(MCP_CAN canx, can_frame & frame)
{
  canx.sendMsgBuf(frame.can_id, 0, frame.can_dlc, frame.data);
}

#define PARSE_TIMEOUT 10
int parseHexInt()
{
    int result = 0;     // accumulate the parse int result here

    // arrays to help parse and convert the text numbers to values
    char hexes[23] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','x','a','b','c','d','e','f'};
    int hexnm[23] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,10,11,12,13,14,15};
    char deces[10] = {'0','1','2','3','4','5','6','7','8','9'};
    int decnm[10] = {0,1,2,3,4,5,6,7,8,9};

    char token[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int numtok = 0;             // number of token chars
    unsigned long timesincechar = 0;
    bool timedout = false;      // timeout since last received character has occurred
    bool numchar = true;        // last char was numeric (either HEX or DEC)
    char nextchar;
    bool hextok = true;        // Does this comply with HEX notation?
    bool dectok = true;        // Does this comply with DEC notation?
    bool hexnot = false;        // Found hex 'x' notation
    bool inttok = true;        // Does this comply with DEC notation?
    bool spctok = true;         // still stripping leading spaces
    bool errtok = false;        // Indicates if an error was detected, i.e. a non-DEC or HEX value

    while (!timedout && numchar)
    {
      if (Serial.available() > 0)
      {
        nextchar = Serial.read();
        timesincechar = millis();   // Set the timer for when the character was received
        
        // If newline found, just end the token read
        if (nextchar == 10)
        {
          numchar = false;
        }
        else
        {
          // If not clearing leading spaces, and see a space, stop reading characters
          if (!spctok && nextchar == ' ')
          {
            numchar = false;
          }
          else
          {
            // If stripping spaces, and space found, just skip the character checks
            if (spctok && nextchar == ' ')
            {
              // Skipping the space by not processing
            }
            else
            {
              // If stripping leading spaces, and you find a non-space, stop ignoring leading spaces
              if (spctok && nextchar != ' ')
              {
                // Stop ignoring leading spaces
                spctok = false;
              }
    
              // test for hex compliance
              bool hexcom = false;
              for (int i=0;i<23;i++)
              {
                  if (nextchar == hexes[i]){hexcom = true;}
              }
              if (!hexcom)
              {
                hextok = false;
              }
              else
              {
                // Test if this could be a hex notation value '0xNN'
                if (numtok == 1 && nextchar == 'x')
                {
                  hexnot = true;
                }

                // Throw an error if the 'x' is in the wrong place
                if (numtok != 1 && nextchar == 'x')
                {
                  hexnot = false;
                  errtok = true;
                }
              }
                  
              // test for positive integer compliance
              bool deccom = false;
              for (int j=0;j<10;j++)
              {
                  if (nextchar == deces[j]){deccom = true;}
              }
              if (!deccom)
              {
                dectok = false;
              }
              
              // If compliant, store the character
              if (hexcom || deccom)
              {
                  // Capture the character
                  token[numtok] = nextchar;
                  numtok = numtok + 1;
              }
              else
              {
                // Flag an error as a non-valid character was located
                errtok = true;
              }
            }
          }
        }
      }
      else
      {
          // Test timeout
          if (millis() > timesincechar + PARSE_TIMEOUT)
          {timedout = true;}
      }
    } // end of While

    // Check that a valid hex notation was received
    if (hextok && !hexnot && !dectok)
    {
      // Looks like a HEX notation, but no 'x' found
      errtok = true;
    }

    if (!errtok)
    {
    
      // Process if HEX was captured in the scan, must have the 0xNN notation to be recognized as HEX
      if (hexnot && hextok)
      {
          result = 0;
          for (int k=0;k<numtok;k++)
          {
              if (token[k] != 'x')
              {
                  int d = 0;
                  for (int n=0;n<23;n++)
                  {
                      if (hexes[n] == token[k])
                      {
                          d = hexnm[n];
                      }
                  }
                  result = (result * 16) + d;
              }
          }
      }
      else
      {
        // Process if DEC was captured in the scan
        if (dectok)
        {
            result = 0;
            for (int k=0;k<numtok;k++)
            {
                int d = 0;
                for (int n=0;n<10;n++)
                {
                    if (deces[n] == token[k])
                    {
                        d = decnm[n];
                    }
                }
                result = (result * 10) + d;
            }
        }
      }
    }
    else
    {
      // Error detected (non-DEC and non-HEX compliance)
      result = -1;
    }

    return result;
}

void loop()
{

  // Set a flag to indicate if a CAN frame was just read
  bool readCANSuccess;
  readCANSuccess = false;

  // Check if there is a CAN frame to read
  if(!digitalRead(TESLOREAN_CAN_INT))   // pin TESLOREAN_CAN_INT low if data on can0
  {
    if (!activeAccess)
    {
      can0.readMsgBuf(&CANFrames[addPointFrames].can_dlc, CANFrames[addPointFrames].data);
      CANFrames[addPointFrames].can_id = can0.getCanId();
      readCANSuccess = true;
  
      // frame contains received from RXB0 message
      addPointFrames = (addPointFrames + 1) % StoredFrames;
      netAddReadCount++;    
    }
  }

  // Receive CAN frame check
  if (CANread)
  {
    // Output a captured frame
    if (netAddReadCount > 0)
    {
      if (readPointFrames != addPointFrames)
      {
        // Copy data into temp store
        activeAccess = true;
        can_frame frame0;
        frame0.can_id = CANFrames[readPointFrames].can_id;
        frame0.can_dlc = CANFrames[readPointFrames].can_dlc;
        for (int i = 0; i < 8; i++)
        {
          frame0.data[i] = CANFrames[readPointFrames].data[i];
        }
        readPointFrames = (readPointFrames + 1) % StoredFrames;
        if(netAddReadCount > 0){netAddReadCount--;}
        activeAccess = false;
        
        // Interpret the frame
        uint8_t foundid;
        foundid = interpretCANframe(frame0);
        // Output the CAN data if the frame was not recognized
        if (foundid==0){outputCANframe(frame0);}

      }
    }
  }    // end of message capture

  // No CAN to catch so just look for Serial commands
  if(!readCANSuccess)
  {   
    // Define CAN Frame variable structure
    struct can_frame outgoing;
    struct can_frame incoming;
  
    // Check serial port for instructions
    char receivedChar;
    int receivedNum;
    if (Serial.available() > 0)
    {
      receivedChar = Serial.read();
  
      // Check for preprepared frame
      if (receivedChar == '?')
      {
        // Output the instructions for frames
        outputInstructions();
      } // end of '?'
      
      // Check for preprepared frame
      if (receivedChar == 'p')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          if ((receivedNum >= 0) && (receivedNum < PreDefinedFrames))
          {
            // Send the selected prepackaged CAN frame
            sendMessage(can0, CANPkg[receivedNum]);
  
            // Send frame to the Serial Port
            Serial.println("Preprepared CAN Frame sent...");
            outputCANframe(CANPkg[receivedNum]);
          }
          else
          {
            Serial.print("Error: Predefined frame number not within 0..");
            Serial.print(PreDefinedFrames,DEC);
            Serial.println(" range.");
          }
        }
        else
        {
          Serial.println("Error: Need to specify which predefined frame to send.");
        }
      }    // end of 'p'
  
      // Start or Stop the CAN frame capture
      if (receivedChar == 'c')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          if (receivedNum == 0)
          {
            CANread = false;
            Serial.println("Stop catching received CAN frames...");
          }
          if (receivedNum == 1)
          {
            CANread = true;
            Serial.println("Start catching received CAN frames...");
          }
        }
        else
        {
          Serial.println("Error: Need to specify 0 - stop or 1 - start frame capture.");
        }
      }    // end of 'c'

      // Check for repeat start/stop requests
      if (receivedChar == 'r')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          if ((receivedNum >= 0) && (receivedNum < PreDefinedFrames))
          {
  
            // Fetch the millisecond delay duration
            unsigned long fdur = 0;
            delay(1);
            if (Serial.available() > 0)
            {
              fdur = Serial.parseInt();
              
              // Update the delay duration for this predefined frame
              CANDurs[receivedNum] = fdur;        // Zero denotes an inactive timer
              CANTimes[receivedNum] = 0;       // init to 0 so always triggers immediately if set
  
              if (fdur == 0)
              {
                // Send frame to the Serial Port
                Serial.println("Repeat CAN request cleared...");
              }
              else
              {
                // Send frame to the Serial Port
                Serial.println("Repeat CAN request set...");
              }
            }
          }
          else
          {
            Serial.print("Error: Predefined frame number not within 0..");
            Serial.print(PreDefinedFrames,DEC);
            Serial.println(" range.");
          }
        }
      }    // end of 'r'
   
      // Check for full specified frame
      if (receivedChar == 's')
      {
        bool fsuc = false;    // at least some frame ID found
        bool bfin = false;   // No more byte data available
        bool ferr = false;    // error detected in the input data
        int bcnt = 0;         // number of data bytes found
        delay(1);
        
        if (Serial.available() > 0)
        {
          // Fetch value and check for error
          int canidtemp = 0;
          canidtemp = parseHexInt();
          if (canidtemp >= 0)
          {
            // Grab the ID for the frame
            outgoing.can_id = canidtemp;
            fsuc = true;
            
            // There could another 8 data entries for the frame
            for (int i = 0; i < 8; i++)
            {
              if (!bfin)
              {
                delay(1);
                if (Serial.available() > 0)
                {
                  int datavaluetemp = 0;
                  datavaluetemp = parseHexInt();
                  if (datavaluetemp >= 0)
                  {
                    outgoing.data[i] = datavaluetemp;
                    bcnt = i + 1;
                  }
                  else
                  {
                    // Record error
                    ferr = true;
                    Serial.print("Error in frame byte ");
                    Serial.println(i,DEC);
                  }
                }
                else
                {
                  bfin = true;      // Flag that no more bytes were found
                }
              }
            }
            outgoing.can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
          }
          else
          {
            ferr = true;
            Serial.println("Error detected in input CAN ID.");
          }  // end of error check
        }
        
        // Output status of the frame input
        if (fsuc and !ferr)
        {
          Serial.println("CAN Frame sent...");
          sendMessage(can0, outgoing);
          outputCANframe(outgoing);      
        }
        else
        {
          Serial.println("No CAN Frame sent...");
        }
  
      }  // end of 's'
  
      // Check for frame to load into predefined array
      if (receivedChar == 'q')
      {
        bool fsuc = false;    // at least some frame ID found
        bool bfin = false;   // No more byte data available
        bool ferr = false;    // indicates an error found in input
        int bcnt = 0;         // number of data bytes found
        int rNum = 0;         // number of predefined entry to replace
        delay(1);
        
        // Test for the predefined record num
        if (Serial.available() > 0)
        {
          rNum = Serial.parseInt();
  
          // Check that entry is within range
          if ((rNum >= 0) && (rNum < PreDefinedFrames))
          {
            delay(1);
            if (Serial.available() > 0)
            {
              // Grab the ID for the frame
              int canidtemp = 0;
              canidtemp = parseHexInt();
              if (canidtemp >= 0)
              {
                CANPkg[rNum].can_id = canidtemp;
                fsuc = true; // Sufficient frame data found
            
                // There could another 8 data entries for the frame
                for (int i = 0; i < 8; i++)
                {
                  if (!bfin)
                  {
                    delay(1);
                    if (Serial.available() > 0)
                    {
                      int datavaluetemp = 0;
                      datavaluetemp = parseHexInt();
                      if (datavaluetemp >= 0)
                      {
                        CANPkg[rNum].data[i] = datavaluetemp;
                        bcnt = i + 1;
                      }
                      else
                      {
                        // Record error
                        ferr = true;
                        Serial.print("Error in frame byte ");
                        Serial.println(i,DEC);
                      }
                    }
                    else
                    {
                      bfin = true;      // Flag that no more bytes were found
                    }
                  }
                }
                CANPkg[rNum].can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
              }
              else
              {
                ferr = true;
                Serial.println("Error detected in input CAN ID.");
              }  // end of error check
            }
          }
          else
          {
            Serial.print("Error: Predefined frame number not within 0..");
            Serial.print(PreDefinedFrames,DEC);
            Serial.println(" range.");
          }
        }
        
        // Output status of the frame input
        if (fsuc && !ferr)
        {
          Serial.println("CAN Frame loaded...");
          outputCANframe(CANPkg[rNum]);      
        }
        else
        {
          Serial.println("No CAN Frame loaded...");
        }
  
      }  // end of 'q'
  
    }  // end of serial port commands check
  
    // Check for repeating timed frame sending
    for (int t = 0; t < PreDefinedFrames; t++)
    {
      // Duration must be > 0 for the timed frames to be active
      if (CANDurs[t] != 0)
      {
        // Check if the duration has passed for this repeated frame
        if (CANTimes[t] < (millis() - CANDurs[t]))
        {
          // Update the timer
          CANTimes[t] = millis();
        
          // Send the selected prepackaged CAN frame
          sendMessage(can0, CANPkg[t]);
  
          // Send frame to the Serial Port
          Serial.println("Preprepared CAN Frame repeated...");
          outputCANframe(CANPkg[t]);
        }
      }
    }  // end of repeat check
    
  } // end of !readCANSuccess
}
