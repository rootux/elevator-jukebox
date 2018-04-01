/***********************************************************/
// Music Jukebox
// Triggered by a button
// Also added a silence track to keep the speaker from going to sleep mode
/***********************************************************/
#include <SoftwareSerial.h>
#include "Statistic.h"

#define TOTAL_SONGS_FOLDER_1 27
#define TOTAL_SONGS_FOLDER_2 6
#define SONG_PLAY_TIME 200 // Time to wait before enabling another press on the button
// Keeping the speaker from going to sleep mode
#define TIME_TO_PLAY_SILENCE_TRACK 30000

#define ARDUINO_RX 5 //should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 6 //connect to RX of the module
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX);

// Buttons
const int firstButtonPin = 8;
const int secondButtonPin = 9;
int firstButtonState = 0;
int secondButtonState = 0;
int folderToPlayFrom = 0;
int shouldPlay = false;

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
#define CMD_PLAY_FLD 0x0F

#define FIRST_FOLDER 0x0100  //Folder 01 on the sd card
#define SECOND_FOLDER 0x0200 //Folder 02 on the sd card

float lastTimePlayed;
bool isPlaying = false;

int currentArrayCountFolder1 = 0;
int currentArrayCountFolder2 = 0;

int songsArrayFolder1[TOTAL_SONGS_FOLDER_1];
int songsArrayFolder1Size = sizeof(songsArrayFolder1) / sizeof(int);

int songsArrayFolder2[TOTAL_SONGS_FOLDER_2];
int songsArrayFolder2Size = sizeof(songsArrayFolder2) / sizeof(int);

void setup()
{
  // Create songs index array
  // Skipping 1st track as this is the silence track
  for(int i=1;i< TOTAL_SONGS_FOLDER_1; i++) {
    songsArrayFolder1[i]=i;
  }
  for(int i=1;i< TOTAL_SONGS_FOLDER_2; i++) {
    songsArrayFolder2[i]=i;
  }

  Serial.begin(115200);
  Serial.println("Started");
  randomSeed(analogRead(2));
  // Fisher-yates shuffle randomly the array
  shuffle(songsArrayFolder1, songsArrayFolder1Size, sizeof(int));
  shuffle(songsArrayFolder2, songsArrayFolder2Size, sizeof(int));

  // initialize the pushbutton pin as an input:
  pinMode(firstButtonPin, INPUT);
  digitalWrite(firstButtonPin, HIGH);
  pinMode(secondButtonPin, INPUT);
  digitalWrite(secondButtonPin, HIGH);

  mySerial.begin(9600);
  delay(500);//Wait chip initialization is complete
  sendCommand(CMD_SEL_DEV, DEV_TF);//select the TF card
  delay(200);
  lastTimePlayed = -1 * SONG_PLAY_TIME;
  sendCommand(CMD_SEL_DEV, 0x001E); //Set volume to max
  delay(200);
}

void test() {
  for(int i=0;i<TOTAL_SONGS_FOLDER_1;i++) {
    Serial.print("Play a song "); Serial.println(i);
    sendCommand(CMD_PLAY_FLD, FIRST_FOLDER + 0x01 + (i));
    delay(3000);
  }
}

void randomizeNewSongsArray1() {
  currentArrayCountFolder1 = 0;
  randomSeed(analogRead(2));
  shuffle(songsArrayFolder1, songsArrayFolder1Size, sizeof(int));
}

void randomizeNewSongsArray2() {
  currentArrayCountFolder2 = 0;
  randomSeed(analogRead(2));
  shuffle(songsArrayFolder2, songsArrayFolder2Size, sizeof(int));
}

void loop()
{
  // Check if should randomize new songs array
  if(currentArrayCountFolder1 >= TOTAL_SONGS_FOLDER_1) {
    randomizeNewSongsArray1();
  }

  if(currentArrayCountFolder2 >= TOTAL_SONGS_FOLDER_2) {
    randomizeNewSongsArray2();
  }

  // read the state of the pushbutton value:
  firstButtonState = digitalRead(firstButtonPin);
  secondButtonState = digitalRead(secondButtonPin);

  // Check if any of the buttons were pressed
  if (firstButtonState == LOW) {
    folderToPlayFrom = FIRST_FOLDER;
    shouldPlay = true;
  }else {
    shouldPlay = false;
  }

  if (secondButtonState == LOW) {
    folderToPlayFrom = SECOND_FOLDER;
    shouldPlay = true;
  }

  if(shouldPlay) {
    if(millis() - lastTimePlayed >= SONG_PLAY_TIME) {
      lastTimePlayed = millis();
      isPlaying = true;

      int8_t randSong = 0;
      if(folderToPlayFrom == FIRST_FOLDER) {
        randSong = songsArrayFolder1[currentArrayCountFolder1++];
      }else if(folderToPlayFrom == SECOND_FOLDER) {
        randSong = songsArrayFolder2[currentArrayCountFolder2++];
      }

      Serial.print("Play a song "); Serial.println(randSong + 1);
      sendCommand(CMD_PLAY_FLD, folderToPlayFrom + 0x01 + (randSong));
      delay(50);
    } else {
      Serial.println("Already playing");
    }

    // Should stop song
    //if(isPlaying && millis() - lastTimePlayed >= SONG_PLAY_TIME) {
    //  isPlaying = false;
    //  Serial.println("Stop song");
    //  sendCommand(CMD_PAUSE, 0);
    //}

    // The following prevent sleeping
//    bool notPlaying = ((millis() - lastTimePlayed) > SONG_PLAY_TIME);
//    if (notPlaying && (millis() % TIME_TO_PLAY_SILENCE_TRACK) > TIME_TO_PLAY_SILENCE_TRACK - 1000) {
//        Serial.println("Playing silence to stay awake");
//        sendCommand(CMD_PLAY_W_VOL, 0X0315); //Playing some song (21 0x15) with low volumes (03)
//        delay(600);
//        sendCommand(CMD_PAUSE, 0);
//        delay(50);
//        sendCommand(CMD_PAUSE, 0);
//    }
  }

  shouldPlay = false;
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


// generate a value between 0 <= x < n, thus there are n possible outputs
int rand_range(int n)
{
   int r, ul;
   ul = RAND_MAX - RAND_MAX % n;
   while ((r = random(RAND_MAX+1)) >= ul);
   return r % n;
}


void shuffle_swap(int index_a, int index_b, int *array, int size)
{
   char *x, *y, tmp[size];

   if (index_a == index_b) return;

   x = (char*)array + index_a * size;
   y = (char*)array + index_b * size;

   memcpy(tmp, x, size);
   memcpy(x, y, size);
   memcpy(y, tmp, size);
}

// shuffle an array using fisher-yates method, O(n)
void shuffle(int *array, int nmemb, int size)
{
   int r;
   
   while (nmemb > 1) {                                                                      
       r = rand_range(nmemb--);                                                              
       shuffle_swap(nmemb, r, array, size);
   }
}