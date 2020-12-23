/*****************************************************************

   "iRadio" embeded code for Arduino that handles the
   user interface.

   Created: 1/11/2015 - Lyle Chamberlain

   Libraries:

   Connected Hardware:  Arduino DUE R3-E

 ****************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Stepper.h>
#include <Bounce2.h>
#include <Encoder.h>

#include <Adafruit_GFX.h>  //Used for OLED display driving
#include <Adafruit_SSD1306.h> //Used for OLED display driving

#define STEPS 600
#define STEPPER_SPEED 30

//Input knob encoders/pushbuttons
#define ENC1_PUSH 27
#define ENC1_A    25
#define ENC1_B    23

#define ENC2_PUSH 26
#define ENC2_A    24
#define ENC2_B    22

uint8_t enc1_state = 0;  //State variables for knob quadrature encoders
uint8_t enc2_state = 0;
Bounce enc1A_bounce = Bounce();  //Software debounce objects for the knob inputs
Bounce enc1B_bounce = Bounce();
Bounce enc2A_bounce = Bounce();
Bounce enc2B_bounce = Bounce();
Bounce enc1Push_bounce = Bounce();
Bounce enc2Push_bounce = Bounce();

bool enc1Push_latch = false;
bool enc2Push_latch = false;

//Encoder knob1Enc(ENC1_A, ENC1_B);
//Encoder knob2Enc(ENC2_A, ENC2_B);

#define KNOB1_MAX  600
#define KNOB1_MIN  1

#define KNOB2_MAX 600
#define KNOB2_MIN 1

uint16_t knob1 = KNOB1_MIN;
uint16_t knob2 = KNOB2_MIN;

#define DEBOUNCE_INTERVAL 5   //Debounce interval in ms

//UI stuff
#define UI_OSCOPE 1
#define UI_TUNING 2
#define UI_SHUTDOWN 3

#define UI_KNOB2_TIMEOUT_MS 5000  //How long to wait after knob touched to switch back to oscope mode (ms)
#define UI_SHUTDOWN_TIMEOUT_MS 5000 //How long to wait for second button press before dismissing shutdown request (ms)

#define PREV_STATION_ADDRESS 0  //EEPROM address of where the last selected station is stored

uint8_t numStations = 0;  //Number of Pandora stations available.  Will be read from RasPi and set during init()
uint8_t prevUserStation = 0;  //Last station user was tuned to before device was shut off.  Will be read from RasPi and set during init

//Display parameters
// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13

//Define a global instance of "display"
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define AUDIO_ANALOG_PIN 0 //Which analog input to read audio waveform from
#define AUDIO_SAMPLE_INTERVAL 250 //How long to wait between ADC reads (in us)
#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH 128

#define OSCOPE_WIDTH 128  //How big the OSCOPE display will be
#define OSCOPE_HEIGHT 64
#define OSCOPE_CENTER 0
#define DISP_FILTER_CONST 0.65  //Low-pass filter constant (0<=const<=1) 

#define AD_MAX_VALUE 4096 //2^12 for a 12-bit conversion


// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 4, 5, 6, 7);

void setup() {

  //GPIO for knobs.  Switches connect to ground.
  pinMode(ENC1_PUSH, INPUT_PULLUP);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_PUSH, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  //Lyle's quadrature reading
  enc1A_bounce.attach(ENC1_A);
  enc1A_bounce.interval(DEBOUNCE_INTERVAL);

  enc1B_bounce.attach(ENC1_B);
  enc1B_bounce.interval(DEBOUNCE_INTERVAL);

  enc2A_bounce.attach(ENC2_A);
  enc2A_bounce.interval(DEBOUNCE_INTERVAL);

  enc2B_bounce.attach(ENC2_B);
  enc2B_bounce.interval(DEBOUNCE_INTERVAL);


  /***A/D Port Setup***/
  //analogReference(DEFAULT); //Set ref voltage of A/D to 3.3V
  analogReadResolution(12);  //DUO can do this


  SerialUSB.begin(9600);

  /***Display Setup***/
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);

  display.clearDisplay();

  //Startup Splash Screen
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextWrap(false);
  display.setCursor(0, 0);
  display.println("Musicotron ");
  display.setCursor(0,20);
  display.println("2000");
  display.display();
 // display.startscrollright(0x00, 0x0F);



  delay(2000);

  SerialUSB.setTimeout(120000);  //Wait up to 120 seconds for the RasPi to come to the party

  numStations = SerialUSB.parseInt();  //Wait for RasPi to boot, load pianobar, and report the number of stations.
  //numStations=16; //Debug TEMP

  //Try setting the station to the last station selected on the last time the device was used
  //prevUserStation = flashStorage.read(PREV_STATION_ADDRESS);  //THIS FUCKING LIBRARY BRICKS THE DUE
  prevUserStation = 0; //TEMP debug
  if ((prevUserStation < 1) || (prevUserStation >= numStations)) prevUserStation = 1; //Make sure theres a valid value in there just in case.

  /***Indicator Needle Stepper Setup/Init***/
  stepper.setSpeed(STEPPER_SPEED);

  //Zero guage and set initial station position (regular loop will move needle)
  knob1 = (uint16_t)(prevUserStation * (uint8_t)(STEPS / numStations)) + 1;
  stepper.step(STEPS - knob1);
  stepper.step(-STEPS);

  display.clearDisplay();


}

void loop()
{

  static uint16_t audioSamples[OSCOPE_WIDTH];  //Buffer of sampled audio
  static bool infoUpdated = true;

  static String currentArtist = "Mojo";
  static String currentSong = "Requiem";
  static String currentStation = "Jazz to the Max";

  static uint8_t UI_mode = UI_OSCOPE;
  static uint32_t lastKnob2Time = millis();

  static uint32_t lastButton2time = millis();

  static uint8_t shutdownButtonCount = 0;  //When this gets to 2, system shutdown is initiated

  /*
     Lyle's quadrature reading
  */

  static long oldPosition  = 1;  //TEMP HACK FOR NOW
  long newPosition = knob1;

  static uint8_t currentKnobStation = 0;  //Which station is selected on knob? (good for change detection)

  update_knob2_state();
  if (update_knob1_state()) //Someone has turned the knob (change things back to knob 2 later)
  {
    UI_mode = UI_TUNING;

    lastKnob2Time = millis();
  }

  //Refresh pushbutton debounce
  enc1Push_bounce.update();
  enc2Push_bounce.update();

  if (0) //enc2Push_bounce.read()!=0)
  {
    UI_mode = UI_SHUTDOWN;

    lastButton2time = millis();
  }

  switch (UI_mode)
  {
    case (UI_OSCOPE):
      updateOscilloscope(audioSamples);  //Update oscilloscope
      display.display(); //Update the display
      break;

    case (UI_TUNING):
      if (abs(newPosition - oldPosition) > 1)
      {
        stepper.step(newPosition - oldPosition);
        oldPosition = newPosition;
        //Serial.println(newPosition);
        uint8_t stationPosition = get_subdivision(numStations, 1, 600, newPosition);
        if (stationPosition != currentKnobStation)
        {
          currentKnobStation = stationPosition;

          display.clearDisplay();
          display.setTextSize(0);
          display.setTextColor(WHITE);
          display.setTextWrap(true);
          display.setCursor(0, 0);
          display.println("Station: "); //display.println(stationPosition);

          while (SerialUSB.available()) SerialUSB.read(); //Clear the input buffer
          SerialUSB.println(stationPosition);  //Ask the RasPi for the station name
          String knobStationName = SerialUSB.readStringUntil('\n');
          display.println(knobStationName);
        }
        display.display();

        //If knob button is pressed, ask the RasPi to change the station
        /*
          if((enc1Push_bounce.read() != 0) & (enc1Push_latch == false)) //See if button is pressed first time
          {
          SerialUSB.println('S');
          enc1Push_latch = true;
          }
          else if(enc1Push_bounce.read()==0) enc1Push_latch = false;
        */
      }



      if ((millis() - lastKnob2Time) > UI_KNOB2_TIMEOUT_MS)
      {
        UI_mode = UI_OSCOPE; //Timeout
        SerialUSB.println('S');  //Change the station
        display.clearDisplay();
        //flashStorage.write(PREV_STATION_ADDRESS, currentKnobStation); //Remember what you switched to.  THIS BRICKS YOUR SHIT!!!
      }
      break;

    case (UI_SHUTDOWN):
      if ((millis() - lastButton2time) > UI_SHUTDOWN_TIMEOUT_MS) //Leave this state if timout occurs
      {
        UI_mode = UI_OSCOPE;  //Timeout
        shutdownButtonCount = 0;  //Reset shutdown button press counter
        display.clearDisplay();
      }

      //Increment count of unique knob presses
      if ((enc2Push_bounce.read() != 0) & (enc2Push_latch == false)) //See if button is press is unique
      {
        enc2Push_latch = true;
        shutdownButtonCount++;

        display.clearDisplay();
        display.setTextSize(0);
        display.setTextWrap(true);
        display.println("Press again to shut down");
        display.display();

      }
      else if (enc2Push_bounce.read() == 0) enc2Push_latch = false;

      if (shutdownButtonCount >= 2) //Initiate shutdown
      {
        SerialUSB.println('D');  //Tell the Pi to shut down.

        //Tell the user what's going on
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setTextWrap(false);
        display.setCursor(0, 0);
        display.println("Shutting down...   ");
        display.display();
        display.startscrollright(0x00, 0x0F);

        //Now wait to give it time
        delay(15000);

        display.clearDisplay();
        display.setTextSize(0);
        display.println("Please turn off switch.");
        display.display();
        display.startscrollright(0x00, 0x0F);

        while (1) //Wait for the user to turn off power
        {
          //noop.
        }
      }

      break;

    default:
      UI_mode = UI_OSCOPE;
      break;
  }

}

void updateOscilloscope(uint16_t *audioSamples)
{

  static uint8_t i, j; //Counting indexes

  //First clear out previous waveform without clearing whole screen
  //Current pixel values are stored in the "audioSamples[]" buffer
  for (i = 0; i < OSCOPE_WIDTH; i++)
  {
    display.drawPixel(i, audioSamples[i], BLACK);
  } //next i

  //Then sample some of the audio into that buffer now that we've used it
  //to erase the previous pixels
  getAudioSamples(audioSamples, OSCOPE_WIDTH, AUDIO_SAMPLE_INTERVAL);

  //Scale those samples to display them onto the screen correctly
  mapAudioSamples(audioSamples, OSCOPE_WIDTH, OSCOPE_HEIGHT, OSCOPE_CENTER);

  //Low-pass filter the samples so they look nice and pretty on our screen:
  for (i = 1; i < OSCOPE_WIDTH; i++)
  {
    audioSamples[i] = audioSamples[i - 1] + DISP_FILTER_CONST * (audioSamples[i] - audioSamples[i - 1]);
  }

  //Now update the pixels to draw the newly-measured waveform
  for (i = 0; i < OSCOPE_WIDTH; i++)
  {
    display.drawPixel(i, audioSamples[i], WHITE);
  } //next i

  return;
}

void getAudioSamples(uint16_t *buffer, uint8_t numSamples, uint16_t sampleDelay)
{
  uint8_t i;
  for (i = 0; i < numSamples; i++)
  {
    buffer[i] = analogRead(AUDIO_ANALOG_PIN);
    if (sampleDelay != 0) delayMicroseconds(sampleDelay);
  }
  return;
}

void mapAudioSamples(uint16_t *buffer, uint8_t width, uint8_t height, uint8_t center)
{
  uint8_t i; //counting

  for (i = 0; i < width; i++)
  {
    buffer[i] = (height * buffer[i]) / (AD_MAX_VALUE);
    if (buffer[i] > height) buffer[i] = (uint16_t)height;
    buffer[i] += center;
  } //next i

  return;
}

uint8_t update_knob1_state()
{
  uint8_t newState;

  enc1A_bounce.update();
  enc1B_bounce.update();

  newState = (enc1A_bounce.read() << 1) | (enc1B_bounce.read());

  if (newState != enc1_state)
  {
    switch (enc1_state) {
      case 0:
        if (newState == 1) knob1++;
        if (newState == 2) knob1--;
        //if(newState==3) invalid.  Don't do anything.
        break;

      case 1:
        if (newState == 3) knob1++;
        if (newState == 0) knob1--;
        //if(newState==2) invalid.  Don't do anything.
        break;

      case 2:
        if (newState == 0) knob1++;
        if (newState == 3) knob1--;
        //if(newState==1) invalid.  Don't do anything.
        break;

      case 3:
        if (newState == 2) knob1++;
        if (newState == 1) knob1--;
        //if(newState==0) invalid.  Don't do anything.
        break;

      default:
        break;

    }

    if (knob1 > KNOB1_MAX) knob1 = KNOB1_MAX;
    else if (knob1 < KNOB1_MIN) knob1 = KNOB1_MIN;
    enc1_state = newState;
    return true;
  }
  else
    return false;
}

uint8_t update_knob2_state()
{
  uint8_t newState;

  enc2A_bounce.update();
  enc2B_bounce.update();

  newState = (enc2A_bounce.read() << 1) | (enc2B_bounce.read());

  if (newState != enc2_state)
  {
    enc2_state = newState;
    return true;
  }
  else
    return false;

}

uint8_t get_subdivision(uint8_t num_divisions, uint16_t min_num, uint16_t max_num, uint16_t input)
{
  uint16_t intervalSize = max_num - min_num;
  uint16_t divisionSize = (uint16_t)intervalSize / num_divisions;

  uint8_t i; //counting variable

  for (i = 1; i < num_divisions; i++)
  {
    if (input + min_num <= i * divisionSize) break; //Stop when you've found the right subdivision
  }

  return i;

}



