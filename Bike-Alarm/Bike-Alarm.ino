#include <RunningAverage.h>

/* 
 *  Sketch using an Arduino Nano, a FONA 808, MFRC522 RFID and an MPU6050 as a silent bicycle alarm. 
 *  Works by sensing presence of RFID. If not present alarm is armed. If alarm is armed and 
 *  motion is detected the unit can:
 *   - Sends a text message to the specified phone number
 *   - Voice call the specified phone number
 *   - Enable GPS and 'phone home' with a fix at specified intervals until the alarm is disarmed.
 *   
 *   To use this sketch there are several variables that you'll have to edit, scroll down to BEGIN USER CONFIG 
 */

/* 
 *  We are pushing the limits of the NANO here!
 *  So lots of serial output is DISABLED in this sketch to save space,
 *  uncomment to debug if you have issues with a particular section.
 */
 
/*
 * Debugging defines - Show extra messages for debugging - Since we are pretty much out of space, these are pretty much
 * GUARANTEED not to work. 
 * 
 * You'll need to uncomment any debug messages you need on a case-by-case basis.
 */

// #define DEBUG_FONA
// #define DEBUG_MPU
// #define DEBUG_GPS
// #define DEBUG_RFID

#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#include <SPI.h>
#include <MFRC522.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define FONA_RX 5
#define FONA_TX 6
#define FONA_RST 7

// this is a large buffer for replies
char replybuffer[255];

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

byte  readline(char *buff, byte  maxbuff, uint16_t timeout = 0);


#define RST_PIN         9           // Configurable, see typical pin layout above
#define SS_PIN          10          // Configurable, see typical pin layout above

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

MFRC522::MIFARE_Key masterKey; // master key object - New!
// MFRC522::MIFARE_Key defaultKey; // default key (all 0xFF) for provisioning new keys

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
boolean blinkState = false;

// MPU control/status vars
boolean dmpReady = false;  // set true if DMP init was successful
byte mpuIntStatus;   // holds actual interrupt status byte from MPU
byte  devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
byte  fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// store accel values to compare against sensor when alarm is activated.
// we are using just accel here as it's int16 and faster than messing with 
// pitch/roll/yaw as floats

//using running average library to debounce MPU data and prevent false positives....
RunningAverage meanX(8);
RunningAverage meanY(8);
RunningAverage meanZ(8);




// BEGIN USER CONFIG

// the following values are user configurable. Key, keyUIDs and blockTokenData must match 
//    what's in the newKey sketch for authentication to succeed.

//the cell phone number to send alert SMS to
const char alertPhone[] = "8005551212"; // Change this to your cell #!!!!

//alarm sensitivity in percentage
byte alarmSensitivity = 60; //set sensitivity of alarm 1 = lowest 100 = highest

// this is the actual gap allowed between the sensor reading and the average before motion sensed
uint16_t sense = (101 - alarmSensitivity) * 10;

//this is key A for our RFID (Note : DO NOT use the default key of FFFFFFFFFFFF or the one provided!!! Make up your own.)
byte  alarmKeyA[6] = { 0x3b, 0x32, 0xa8, 0xc3, 0x3f, 0xf5 };

//put the UIDs of your RFID PICCs here and define the number of keys...
// this is the number of provisioned UIDs
#define N_UIDS   3

// These are the provisioned UIDs, one per row. You MUST add your key UIDs here after you provision.
// MUST be one key per row and match the number of keys in N_UIDS above.
byte keyUIDs[N_UIDS][4] =  {
    { 0x00, 0x00, 0x00, 0x00 }, //key 1 UID
    { 0x00, 0x00, 0x00, 0x00 }, //key 2 UID
    { 0x00, 0x00, 0x00, 0x00 }  //key 3 UID
};

//define the sector and blocks used by the application
byte sector = 1;

//this is the token block (i.e. THIS IS YOUR KEY!) Change it to something unique 
byte tokenBlockData[16]    = {
    0x54, 0x69, 0xdd, 0xa7,
    0x34, 0x66, 0x01, 0xdc,
    0x9a, 0x2d, 0x0b, 0xaa, 
    0xa0, 0x9b, 0xd8, 0x2b
};

byte settleTime = 15;  //time to let the MPU settle after arming. Increase if you get false positive alarm right after arming.

//   Integrate some time management to limit SMS and keep from TEXT BOMBING ourselves
int messageDelay = 1800; //delay between messages in seconds * 10      (*default == 1800 == 3 minutes)

//  (set to true to temporarily disable sending of SMS. WARNING - will still send msg after messageDelay expires.)
boolean alertSent = false;

//END USER CONFIG




//calculate block addresses from sector chosen.
byte tokenBlock = sector * 4;;
byte trailingBlock = tokenBlock + 3;

//cached values (i.e. these will be overwritten during normal operation)
boolean alarmArmed = false; 
boolean alarmInit = false;
boolean phoneHome = false;
boolean manageMe = false; //true if serial plugged in

//count samples from mpu to let the averaging buffer fill

byte samples = 0;
uint16_t loopCounter = 0;

/*
 *  Helper routine to compare byte arrays.,..
 */
boolean compareByteArray(byte a[], byte b[],int array_size)
{
   for (int i = 0; i < array_size; ++i)
     if (a[i] != b[i])
       return(false);
   return(true);
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dumpByteArray(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}
/**
 * Helper routine to copy a substring (we are avoiding the 'String' object due to size constraints.
 */
char* subString (const char* input, int offset, int len, char* dest)
{
  int input_len = strlen (input);

  if (offset + len > input_len)
  {
     return NULL;
  }

  strncpy (dest, input + offset, len);
  return dest;
}
/*
 * Try using the PICC (the tag/card) with the given key to access block 0.
 * On success, it will show the key details, and dump the block data on Serial.
 *
 * @return true when the given key worked, false otherwise.
 */
boolean tryKey(MFRC522::MIFARE_Key *key)
{
    boolean result = false;
    byte buffer[18];
    byte status;
#ifdef DEBUG_RFID
    if(manageMe)
      Serial.println(F("Authenticating using key A..."));
#endif
    status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailingBlock, key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK) {
#ifdef DEBUG_RFID
       if(manageMe)
       {
         Serial.print(F("AUTH failed: "));
         Serial.println(mfrc522.GetStatusCodeName(status));
       }
#endif       
        return false;
    }

    // Read block
    byte byteCount = sizeof(buffer);
    status = mfrc522.MIFARE_Read(trailingBlock, buffer, &byteCount);
    if (status != MFRC522::STATUS_OK) {
#ifdef DEBUG_RFID      
      if(manageMe)
      {
          Serial.print(F("READ failed: "));
          Serial.println(mfrc522.GetStatusCodeName(status));
      }
#endif  
    }
    else {
        // Successful read
        result = true;
#ifdef DEBUG_RFID        
        if(manageMe)
        {
          Serial.print(F("Success with key:"));
          dumpByteArray((*key).keyByte, MFRC522::MF_KEY_SIZE);
          Serial.println();
          // Dump block data
          Serial.print(F("Block ")); Serial.print(trailingBlock); Serial.print(F(":"));
          dumpByteArray(buffer, 16);
          Serial.println();
        }
#endif 
    }

    return result;
}


boolean checkRFIDKey()
{
   // Look for new cards
    if(mfrc522.PICC_IsNewCardPresent())
    {
    
      // Select one of the cards
   
      if ( ! mfrc522.PICC_ReadCardSerial())
      {
        alarmArmed;
      }
      else
      {
        boolean cardPresent = true;
        byte _uid[4] = {0, 0, 0, 0};
        for (byte u = 0; u < N_UIDS; u++) {
          for (byte j = 0; j < 4; j++) {
            _uid[j] = keyUIDs[u][j];
          }
          
          if(compareByteArray(_uid, mfrc522.uid.uidByte, 4))
          {
            // is masterUID skip EEPROM
            if(tryKey(&masterKey))
            {
              if(manageMe)
                Serial.print(F("Card Detected."));
              //success! check token
              byte status;
              byte buffer[18];
              byte size = sizeof(buffer);
              while(cardPresent)
              {
                // Read data from the block
#ifdef DEBUG_RFID                
                if(manageMe)
                {
                  Serial.print(F("READ block: ")); Serial.print(tokenBlock);
                  Serial.println(F(" ..."));
                }
#endif       
                status = mfrc522.MIFARE_Read(tokenBlock, buffer, &size);
                if (status != MFRC522::STATUS_OK) {
#ifdef DEBUG_RFID                  
                  if(manageMe)
                  {
                    Serial.print(F("READ failed: "));
                    Serial.println(mfrc522.GetStatusCodeName(status));
                  }
#endif       
                  cardPresent = false;
                }
#ifdef DEBUG_RFID                
                if(manageMe)
                { 
                  Serial.print(F("Data in block ")); Serial.print(tokenBlock); Serial.println(F(":"));
                  dumpByteArray(buffer, 16); 
                  Serial.println();
                }
#endif
                if(compareByteArray(buffer, tokenBlockData, 16))
                {
                  alarmArmed = false;
                  meanX.clear();
                  meanY.clear();
                  meanZ.clear();
                  samples = 0;

                  if(manageMe)
                    Serial.println(F("AUTH SUCCESS! Alarm Disarmed."));
                    alarmArmed = false;
                    
                    //reset multiple alert delay
                    loopCounter = 0;
                    //check once per second.
                    delay(1000);
                  //note this loop will continue to execute and until the PICC is removed 
                  //  at which point the alarm will ARM!
                
                }
                else
                {
                  alarmArmed = true;
                  cardPresent = false;  
                }
              }  
            }
            else
            {
              alarmArmed = true;
            }   
          }
        }
        //MPU FIFO has built up while we were doing this...
        mpu.resetFIFO();
      }
    }
}

void setup() {
  if(Serial)
    manageMe = true;

  if(manageMe)
  {
    Serial.begin(115200);
    
#ifdef DEBUG_FONA    
    Serial.print(F("Initializing FONA....(May take 3 seconds)"));
    Serial.print("\n");
#endif
  }

  fonaSS.begin(4800); // if you're using software serial
  //Serial1.begin(4800); // if you're using hardware serial

  if (! fona.begin(fonaSS)) {           // can also try fona.begin(Serial1)
#ifdef DEBUG_FONA     
    if(manageMe)
    {
      Serial.print(F("Couldn't find FONA"));
      Serial.print("\n");
    }
#endif    
    while (1);
  }
#ifdef DEBUG_FONA  
  if(manageMe)
  {
    Serial.print(F("FONA is OK"));
    Serial.print("\n");

    // Print SIM card IMEI number.
    char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
    byte  imeiLen = fona.getIMEI(imei);
    if (imeiLen > 0) {
      Serial.print("SIM card IMEI: "); 
      Serial.print(imei);
      Serial.print("\n");
    }
  }
#endif  
  // now set up the MFRC522
  SPI.begin();        // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522 card

  // Prepare the key
  for (byte i = 0; i < 6; i++) {
      masterKey.keyByte[i] = alarmKeyA[i];  
  }
#ifdef DEBUG_RFID  
  if(manageMe)
  {
    Serial.print(F("Using key (for A and B):"));
    dumpByteArray(defaultKey.keyByte, MFRC522::MF_KEY_SIZE);
    Serial.print("\n");
  }
#endif
//now initialize the MPU6050
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
#ifdef DEBUG_MPU 
    if(manageMe)
    {
      Serial.print(F("Initializing I2C devices..."));
      Serial.print("\n");
    }
#endif  
    mpu.initialize();

    // verify connection
#ifdef DEBUG_MPU     
    if(manageMe)
    {
      Serial.print(F("Testing device connections..."));
      Serial.print("\n");
      Serial.print(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      Serial.print("\n");
    }
#endif
    mpu.testConnection();
    // load and configure the DMP
    if(manageMe)
    {
      Serial.print(F("Initializing DMP..."));
      Serial.print("\n");
    }
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(180);
    mpu.setYGyroOffset(90);
    mpu.setZGyroOffset(-96);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
#ifdef DEBUG_MPU         
        if(manageMe)
        {
          Serial.print(F("Enabling DMP..."));
          Serial.print("\n");
        }
#endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
#ifdef DEBUG_MPU         
        if(manageMe)
        {
          Serial.print(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
          Serial.print("\n");
        }
#endif
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef DEBUG_MPU         
        if(manageMe)
        {
          Serial.print(F("DMP ready! Waiting for first interrupt..."));
          Serial.print("\n");
        }
#endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
#ifdef DEBUG_MPU        
        if(manageMe)
        {
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.print(F(")"));
          Serial.print("\n");
        }
#endif  
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);



}

void loop() {
    if(Serial)
      manageMe = true;
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
#ifdef DEBUG_MPU       
      if(manageMe)
      {
        Serial.print(F("MPU programming Failed!!!!"));
        Serial.print("\n");
      }
#endif  
      return;
    }

    alarmArmed = !checkRFIDKey();
    
    if(alarmArmed)
    {
      if(manageMe)
        Serial.println(F("Alarm ARMED!"));
      mfrc522.PICC_HaltA();       // Halt PICC
      mfrc522.PCD_StopCrypto1();  // Stop encryption on PCD

      // reset the MPU FIFO or we will get an overflow on the first read.
      mpu.resetFIFO();
      // wait for MPU interrupt or extra packet(s) available
      while (!mpuInterrupt && fifoCount < packetSize) {
        if(mfrc522.PICC_IsNewCardPresent()) 
          return;
       }

      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow 
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
#ifdef DEBUG_MPU          
          if(manageMe)
          {
            Serial.print(F("FIFO overflow!"));
            Serial.print("\n");
          }
#endif          
          // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else if (mpuIntStatus & 0x02) {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
      
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
            
          // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

          if(samples == 0) //not initialized
          {

            meanX.addValue(aaWorld.x);
            meanY.addValue(aaWorld.y);
            meanZ.addValue(aaWorld.z);
            samples = 1;
                        
            //now turn on the GPS...
            if (!fona.enableGPS(true))
            {
#ifdef DEBUG_GPS              
              if(manageMe)  
                Serial.println(F("Failed to turn on GPS"));
#endif                
            }
#ifdef DEBUG_MPU            
            else if(manageMe)  
                Serial.println(F("GPS on. Waiting for Fix..."));
#endif      
            alarmInit = true;
          }
          else
          {
            
            if(abs(aaWorld.x) > 12000 || abs(aaWorld.y) > 12000 || abs(aaWorld.z) > 12000)
            {
              // every once in a while the MPU will generate a spurious reading check for and ignore.
            }
            else
            {
              
              int16_t minX = meanX.getAverage() - sense;
              int16_t maxX = meanX.getAverage() + sense;
              int16_t minY = meanY.getAverage() - sense;
              int16_t maxY = meanY.getAverage() + sense;
              int16_t minZ = meanZ.getAverage() - sense;
              int16_t maxZ = meanZ.getAverage() + sense;
                        
              // is initialized... look for movement.
              if(samples > 11 && (aaWorld.x < minX || aaWorld.x > maxX ||
                aaWorld.y < minY || aaWorld.y > maxY ||
                aaWorld.z < minZ || aaWorld.z > maxZ)){
#ifdef DEBUG_MPU                   
                  //motion detected!!!
                  Serial.println(F("Motion Detected!!! "));
                  Serial.print(aaWorld.x);
                  Serial.print(":");
                  Serial.println(meanX.getAverage());
                  
                  Serial.print(aaWorld.y);
                  Serial.print(":");
                  Serial.println(meanY.getAverage());
                  
                  Serial.print(aaWorld.z);
                  Serial.print(":");
                  Serial.println(meanZ.getAverage());                 
#endif

#ifdef DEBUG_GPS
                  //query coordinates
                  int8_t stat = fona.GPSstatus();
                  // check GPS fix
                  Serial.print(F("GPS status: "));
                  switch(stat)
                  {
                    case 0:
                      Serial.println(F("off"));
                      break;
                    case 1:
                      Serial.println(F("No fix"));
                      break;
                    case 2:
                      Serial.println(F("2D fix"));
                      break;
                    case 3:
                      Serial.println(F("3D fix"));
                      break;
                  }
#endif                 
                  char coord[13];
                  char gpsdata[80];
                  fona.getGPSlocation(gpsdata, 80);
                  Serial.println(gpsdata);
                  char _sectionIndex = 0;

                  char* _section = strtok(gpsdata, ",");

                  char latD[3] = "";
                  char latM[8] = "";
                  char lonD[4] = "";
                  char lonM[9] = "";

                  short latDI = 0;
                  short lonDI = 0;
                  float latMF = 0.0;
                  float lonMF = 0.0;
                  float speedMS = 0.0;
                  float headingDeg = 0.0;
                  
                  
                  while (_section != 0)
                  { 
                    if(_sectionIndex == 1)
                    {
                      subString(_section, 0, 2, latD);
                      subString(_section, 2, 8, latM);
                      latDI = atoi(latD);
                      latMF = atof(latM);
                      //is latitude
                    }
                    if(_sectionIndex == 2)
                    {
                      // is longitude
                      subString(_section, 0, 3, lonD);
                      subString(_section, 3, 9, lonM);
                      lonDI = atoi(lonD);
                      lonMF = atof(lonM);
                    }
                    // 3 is altitude in meters
                    // 4 is UTC time as YYYYMMDDHHMMSS.mS
                    // 5 is time to fix in seconds
                    // 6 is # of sateellites
                    if(_sectionIndex == 7)
                    {
                      // is speed
                      speedMS = atof(_section);
                    }
                    if(_sectionIndex == 8)
                    {
                      // is heading in degrees
                      headingDeg = atof(_section);
                    }
                    // Find the next section in input string
                    _section = strtok(0, ",");
                    _sectionIndex++;
                  }

                  //this next section parses the NMEA location string and generates a google maps link
                  char msg[141] = "Bike Alarm Triggered!!! Speed: ";
                  char szTmp[12];
                  
                  dtostrf(speedMS, 2, 4, szTmp);               
                  strncat(msg, szTmp, 7);
                  strncat(msg, " M/S, Dir: ", 13);
                             
                  dtostrf(headingDeg, 3, 3, szTmp);
                  strncat(msg, szTmp, 7);
                  strncat(msg, " Deg\n", 4);
                  strncat(msg, "GPS: http://maps.google.com/maps?q=", 35);

                  float _lat = latDI + (latMF / 60);
                  dtostrf(_lat, 8, 6, szTmp);
                  strncat(msg, szTmp, 9);    
                  
                  strncat(msg, ",-", 2); //assuming longitude is east... fix this later ;-)                             
                  
                  float _lon = lonDI + (lonMF / 60);
                  dtostrf(_lon, 9, 6, szTmp);
                  strncat(msg, szTmp, 10);

                  
                  // now then send text message (with coordinates if available)

                  Serial.println(msg);
                  if(!alertSent && ( loopCounter * 10 >= settleTime))
                  {    
                    if (!fona.sendSMS(alertPhone, msg)) {
                      Serial.println(F("SMS Failed"));
                    } else {
                      Serial.println(F("Sent SMS!"));
                      alertSent = true;
                    }
                  }
                  
                  //that took long enough that we should reset our buffer to prevent a false positive
                  meanX.clear();
                  meanY.clear();
                  meanZ.clear();
                  samples = 0;
                  mpu.resetFIFO();
                  
               }
              meanX.addValue(aaWorld.x);
              meanY.addValue(aaWorld.y);
              meanZ.addValue(aaWorld.z);
            }
            
          }

          if(samples < 12)
            samples++;

        // do 10 loops per second
        delay(100);
        //we just delayed 100ms so let's clear the FIFO!
        mpu.resetFIFO();
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        alarmArmed = !checkRFIDKey();
        loopCounter++;
        if(loopCounter > messageDelay) //delay has elapsed, reset and send an SMS on the next loop...
        {
          loopCounter = 0;
          alertSent = false;
          Serial.println(F("Delay expired!"));
        }
      }
    }
}

