/***********************************************************/
//Demo for the Serial MP3 Player by Catalex
//Hardware: Serial MP3 Player *1
//Board:  Arduino UNO R3
//IDE:	  Arduino-1.0
//Function:  To play the first song in the micro sd card.
//Store: http://www.aliexpress.com/store/1199788
//          http://www.dx.com/
#include <SoftwareSerial.h>
#include "Statistic.h"
#include "ArrayShuffle.cpp"

#define TOTAL_SONGS 26
#define ACCELEROMETER_TRASHHOLD 0.03
#define SONG_PLAY_TIME 18000
#define STD_DEV_SIZE 10

int scale = 3; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true;
Statistic myStatsX; 
Statistic myStatsY;
Statistic myStatsZ;

#define ARDUINO_RX 5//should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 6//connect to RX of the module
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX);

static int8_t Send_buf[8] = {0} ;

#define CMD_PLAY_W_INDEX 0X03
#define CMD_SET_VOLUME 0X06
#define CMD_SEL_DEV 0X09
#define DEV_TF 0X02
#define CMD_PLAY 0X0D
#define CMD_PAUSE 0X0E
#define CMD_SINGLE_CYCLE 0X19
#define SINGLE_CYCLE_ON 0X00
#define SINGLE_CYCLE_OFF 0X01
#define CMD_PLAY_W_VOL 0X22
float scaledX, scaledY, scaledZ; // Scaled values for each axis
int rawX,rawY,rawZ;
float lastSongTime;
bool isPlaying = false;

void setup()
{
  myStatsX.clear();
  myStatsY.clear();
  myStatsZ.clear();

	mySerial.begin(9600);
  randomSeed(analogRead(0));
	delay(500);//Wait chip initialization is complete
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card  
  delay(200);
  Serial.begin(115200);
  Serial.println("Started");
}

void loop() 
{
  readAccelerometer();
  myStatsX.add(scaledX);
  myStatsY.add(scaledY);
  myStatsZ.add(scaledZ);
  
  Serial.print("X: "); Serial.print(scaledX);
  Serial.print("Y: "); Serial.print(scaledY);
  Serial.print("Z: "); Serial.print(scaledZ);
  Serial.println();

  // Should stop song
  if(isPlaying && millis() - lastSongTime >= SONG_PLAY_TIME) {
    isPlaying = false;
    Serial.println("Stop song");
    sendCommand(CMD_PAUSE, 0);
  }


  // Error cleaning - Check for std dev every STD_DEV_SIZE calls
  if (myStatsX.count() >= STD_DEV_SIZE) {
    float stdX = myStatsX.pop_stdev();
    float stdY = myStatsY.pop_stdev();
    float stdZ = myStatsZ.pop_stdev();
    Serial.print("STD Dev X: "); Serial.println(stdX);
    Serial.print("STD Dev Y: "); Serial.println(stdY);
    Serial.print("STD Dev Z: "); Serial.println(stdZ);
    myStatsX.clear();
    myStatsY.clear();
    myStatsZ.clear();

    if(
      (stdX > ACCELEROMETER_TRASHHOLD) ||
      (stdY > ACCELEROMETER_TRASHHOLD) ||
      (stdZ > ACCELEROMETER_TRASHHOLD)) {

        if(millis() - lastSongTime >= SONG_PLAY_TIME) {
          lastSongTime = millis();
          isPlaying = true;
          int8_t randSong = random(TOTAL_SONGS);
          Serial.print("Play a song "); Serial.println(randSong);
          sendCommand(CMD_PLAY_W_VOL, 0X1E01+(randSong*2)); //TODO Not sure why *2
        } else {
          Serial.println("Already playing");
        }
      }
  }

  delay(50);
//  int8_t randNumber = random(TOTAL_SONGS);
//  for(int8_t i=0; i<100;i++) {
//    sendCommand(CMD_PLAY_W_VOL, 0X01E1+i);//play the first song with volume 15 class
//    //delay(10000);
//  }
}

void readAccelerometer() {
  rawX = analogRead(A0);
  rawY = analogRead(A1);
  rawZ = analogRead(A2);
  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  if (micro_is_5V) // Microcontroller runs off 5V
  {
    scaledX = mapf(rawX, 0, 675, -scale, scale); // 3.3/5 * 1023 =~ 675
    scaledY = mapf(rawY, 0, 675, -scale, scale);
    scaledZ = mapf(rawZ, 0, 675, -scale, scale);
  }
  else // Microcontroller runs off 3.3V
  {
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
  }
}

void sendCommand(int8_t command, int16_t dat)
{
  delay(20);
  Send_buf[0] = 0x7e; //starting byte
  Send_buf[1] = 0xff; //version
  Send_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_buf[3] = command; //
  Send_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  Send_buf[5] = (int8_t)(dat >> 8);//datah
  Send_buf[6] = (int8_t)(dat); //datal
  Send_buf[7] = 0xef; //ending byte
  for(uint8_t i=0; i<8; i++)//
  {
    mySerial.write(Send_buf[i]) ;
  }
}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




