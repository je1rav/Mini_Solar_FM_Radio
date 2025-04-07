/*
  This sketch runs on ESP32C3 with 0.42-inch OLED device.
  The sketch was made by JE1RAV with modifying “RDA5807_06_ESP8266_OLED.ino” in the “PU2CLR RDA5807” library.

  Arduino ESP32 Boards manager ver.2.0.17 was used to compile this sketch.

  ESP32C3 and components wire up.
  | Device name               | Device Pin / Description      |  ESP32C3      |
  | ----------------          | ----------------------------- | ------------  |
  |    OLED                   |                               |               |
  |                           | SDA/SDIO                      |  GPIO5        |
  |                           | SCL/SCLK                      |  GPIO6        |
  |    RDA5807                |                               |               |
  |                           | +Vcc 3.3V [1]                 |   3.3V        |
  |                           | SDIO (pin 8)                  |   GPIO5       |
  |                           | SCLK (pin 7)                  |   GPIO6       |
  |    BUTTUNS                |                               |               |
  |                           | MODE                          |  GPIO 3       |
  |                           | DOWN                          |  GPIO 7       |
  |                           | UP                            |  GPIO 10      |
  |    ADC                    |                               |               |
  |                           |                               |  GPIO 1       |
  |    Digital OUTPUT         |                               |               |
  |                           |                               |  GPIO 9       |

  References:
   PU2CLR RDA5807 documentation: https://github.com/pu2clr/RDA5807
  By PU2CLR, Ricardo.
*/

#include <Arduino.h>
#include <Wire.h>
#include "EEPROM.h"
#include <RDA5807.h>
#include "driver/adc.h" 

#include <U8g2lib.h> 
// there is no 72x40 constructor in u8g2 hence the 72x40 screen is mapped in the middle of the 132x64 pixel buffer of the SSD1306 controller 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5); 
int oled_width = 72; 
int oled_height = 40; 
//int xOffset = 30; // for ESP32 Supermini 0.42 OLED
//int yOffset = 12; // for ESP32 Supermini 0.42 OLED
int xOffset = 28; // for 0.42inch OLED module
int yOffset = 24; // for 0.42inch OLED module
 
// INPUT PINs
#define PIN_UP GPIO_NUM_10 // GPIO10
#define PIN_DOWN GPIO_NUM_7 // GPIO7
#define PIN_MODE GPIO_NUM_3 // GPIO3  mode change, wake up from deep sleep
// OUTPUT PIN
#define PIN_DRAIN GPIO_NUM_4 // GPIO9  drain a few current to avoid auto power-off of mobile battery 
//ADC for Battery check
#define ADC_CHANNEL ADC_CHANNEL_1 // GPIO1

#define STORE_TIME 10000 // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).
#define PUSH_MIN_DELAY 200
#define EEPROM_SIZE 512
#define MIN_ELAPSED_TIME 200
#define ELAPSED_PINS 1000 // Time to enter the light sleep mode (millisecond)
#define SEEKTHRESHOLD 55 //RSSI Seek Threshold (0 to 127)
#define TIMER_WAKEUP_PERIOD 25 // Timer wake-up period(second))

// MENU CONTROL
bool cmdFreq = false;
bool cmdVolume = false;
bool cmdStereo = false;
bool cmdSeek = false;
bool cmdBattery = false;
bool cmdAntiauto_off = false;

long elapsedClick = millis();
long elapsedCommand = millis();
long elapsedPins = millis();

uint8_t nivelMenu = 0;

// END MENU CONTROL

const uint8_t app_id = 43; // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
long storeTime = millis();

bool bSt = true;
//bool bRds = false;

long pollin_elapsed = millis();
bool deep_sleep = false;
uint32_t read_ADC = 0;
uint32_t voltage = 0;
bool bAapo = false; //Anti auto power off

// Encoder control variables
uint16_t currentFrequency;
uint16_t previousFrequency;
float Freq; 

int8_t step = 0; //100 kHz
uint8_t tabStep[] = {100, 200, 50, 25};

// Devices class declarations
RDA5807 rx;

esp_sleep_wakeup_cause_t wakeup_reason;

void setup()
{
  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_MODE, INPUT_PULLUP);

  pinMode(PIN_DRAIN, OUTPUT); //for Anti auto power off of moble battrey
  pinMode(GPIO_NUM_8, OUTPUT); //for Anti auto power off of moble battrey

  delay(1000);
  u8g2.begin();  //
  u8g2.setContrast(255); // set contrast to maximum 
  u8g2.setBusClock(100000); //100kHz I2C 

  EEPROM.begin(EEPROM_SIZE);

  digitalWrite(PIN_DRAIN, LOW);
  digitalWrite(GPIO_NUM_8, HIGH);

  // If you want to reset the eeprom, keep the MODE PUSH BUTTON  pressed during statup
  if (digitalRead(PIN_MODE) == LOW)
  {
    EEPROM.write(eeprom_address, 0);
    EEPROM.commit();
    u8g2.clearBuffer(); // clear the internal memory 
  //u8g2.drawFrame(xOffset+0, yOffset+0, width, height); //draw a frame around the border 
  //u8g2.setCursor(xOffset+15, yOffset+10); 
    u8g2.setFont(u8g2_font_helvB10_tr); 
    u8g2.drawStr(xOffset,yOffset+30,"EEPROM RESET");
    u8g2.sendBuffer(); // transfer internal memory to the display
    delay(3000);
  }

  rx.setup();
  rx.setBand(2); // Sets band: 76–108 MHz (world wide)  
  delay(3000);

  // Checking the EEPROM content
  if (EEPROM.read(eeprom_address) == app_id)
  {
    readAllReceiverInformation();
  }
  else
  {
    // Default values
    rx.setVolume(1);
    rx.setMono(false); // Force stereo
    rx.setRBDS(false);  //  set RDS and RBDS. See setRDS.
    rx.setRDS(false);
    rx.setRdsFifo(false);
    currentFrequency = previousFrequency = 8470;
  }

  rx.setFrequency(currentFrequency); // It is the frequency you want to select in MHz multiplied by 100.
  rx.setSeekThreshold(SEEKTHRESHOLD); // Sets RSSI Seek Threshold (0 to 127)

  cmdSeek = true;
  showStatusSeek();

  gpio_wakeup_enable(PIN_MODE, GPIO_INTR_LOW_LEVEL);  //light sleep
  gpio_wakeup_enable(PIN_DOWN, GPIO_INTR_LOW_LEVEL);  //light sleep
  gpio_wakeup_enable(PIN_UP, GPIO_INTR_LOW_LEVEL);  //light sleep
  esp_sleep_enable_gpio_wakeup();
  sntp_set_system_time(1688564256, 0);
  esp_sleep_enable_timer_wakeup(TIMER_WAKEUP_PERIOD * 1000 * 1000UL);  // timer wakeup for battery check and Anti-APO(Auto Power OFF) 

  esp_deep_sleep_enable_gpio_wakeup(BIT(3), ESP_GPIO_WAKEUP_GPIO_LOW); //wake up from deep sleep by pushing GPIO3 (PIN_MODE)
}

// MENU CONTROL IMPLEMENTATION
/**
    Set all command flags to false
    When all flags are disabled (false), the encoder controls the frequency
*/
void disableCommands()
{
  cmdFreq = false;
  cmdVolume = false;
  cmdStereo = false;
  cmdSeek = false;
  cmdBattery = false;
  cmdAntiauto_off = false;
}

/*
   writes the conrrent receiver information into the eeprom.
   The EEPROM.write avoid write the same data in the same memory position. It will save unnecessary recording.
*/
void saveAllReceiverInformation()
{
  EEPROM.begin(EEPROM_SIZE);

  // The write function/method writes data only if the current data is not equal to the stored data.
  EEPROM.write(eeprom_address, app_id);
  EEPROM.write(eeprom_address + 1, rx.getVolume());          // stores the current Volume
  EEPROM.write(eeprom_address + 2, currentFrequency >> 8);   // stores the current Frequency HIGH byte for the band
  EEPROM.write(eeprom_address + 3, currentFrequency & 0xFF); // stores the current Frequency LOW byte for the band
  EEPROM.write(eeprom_address + 4, (uint8_t)bAapo);
  EEPROM.write(eeprom_address + 5, (uint8_t)bSt);
  EEPROM.write(eeprom_address + 6, step);
  EEPROM.commit();
  EEPROM.end();
}

/**
 * reads and set the last receiver status.
 */
void readAllReceiverInformation()
{
  rx.setVolume(EEPROM.read(eeprom_address + 1));
  currentFrequency = EEPROM.read(eeprom_address + 2) << 8;
  currentFrequency |= EEPROM.read(eeprom_address + 3);
  previousFrequency = currentFrequency;

  bAapo = (bool)EEPROM.read(eeprom_address + 4); // Rescue RDS status (ON / OFF)
  //rx.setRDS(bRds);
  //rx.setRdsFifo(bRds);

  bSt = (bool)EEPROM.read(eeprom_address + 5); // Rescue Stereo status (ON / OFF)
  rx.setMono(bSt);

  step = EEPROM.read(eeprom_address + 6); // Rescue step: 0 = 100; 1 = 200; 2 = 50; 3 = 25 (in kHz)
  rx.setSpace((uint8_t)step);                      // See tabStep
}

/*
   To store any change into the EEPROM, it is needed at least STORE_TIME  milliseconds of inactivity.
*/
void resetEepromDelay()
{
  delay(PUSH_MIN_DELAY);
  storeTime = millis();
  elapsedCommand = millis();
  previousFrequency = 0;
}


/**
 * Shows frequency information on Display
 */
void showFrequency()
{
  currentFrequency = rx.getFrequency();
  Freq = (float)currentFrequency/100.0;
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"Freq.");
  u8g2.setCursor(xOffset+7, yOffset+40); 
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.printf("%5.1f", Freq); 
  u8g2.sendBuffer(); // transfer internal memory to the display
}

void showFrequencySeek()
{
  currentFrequency = rx.getFrequency();
  Freq = (float)currentFrequency/100.0;
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"Seek");
  u8g2.setCursor(xOffset+4, yOffset+40); 
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.printf("%5.1f", Freq); 
  u8g2.sendBuffer(); // transfer internal memory to the display
}

void showStatus()
{
  showFrequency();
  //showStereoStatusMono();
  showRSSI();
  showBattery();
}

void showStatusSeek()
{
  showFrequencySeek();
  showStereoStatusMono();
  showRSSI();
  showBattery();
}

void showVolume()
{
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"Vol");
  u8g2.setCursor(xOffset+30, yOffset+40); 
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.printf("%2u", rx.getVolume()); 
  u8g2.sendBuffer(); // transfer internal memory to the display
  showBattery();
}

void showStereoStatus()
{
  char aux[12];
  sprintf(aux, "%s", (bSt) ? "ON" : "OFF");
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"STEREO");
  u8g2.setCursor(xOffset+14, yOffset+40); 
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.printf("%s", aux); 
  u8g2.sendBuffer(); // transfer internal memory to the display
  showBattery();
}

void showRSSI()
{
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.setCursor(xOffset+50, yOffset+11); 
  u8g2.printf("%3u", rx.getRssi()); 
  u8g2.sendBuffer(); // transfer internal memory to the display
}

void showStereoStatusMono()
{
  u8g2.setFont(u8g2_font_helvB10_tr); 
  if (rx.isStereo())
   u8g2.drawStr(xOffset+38, yOffset+11,"S"); 
  else
   u8g2.drawStr(xOffset+38, yOffset+11,"M"); 

  u8g2.sendBuffer(); // transfer internal memory to the display
}

void showBatteryStatus()
{ 
    read_ADC = analogRead(ADC_CHANNEL);
    for (int i=0;i<7;i++)
    {
      read_ADC += analogRead(ADC_CHANNEL);
      delay(10);
    }
    read_ADC = read_ADC / 8;
    voltage = read_ADC * 1.485;
    read_ADC = read_ADC/32;
    if (read_ADC > 88) read_ADC=25; // above 4.2V
    else if (read_ADC < 63) read_ADC=0; // below 3.0V
    else read_ADC = (read_ADC - 63 );
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"Battery");
  u8g2.setCursor(xOffset+7, yOffset+40); 
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.printf("%4.2f", (float)voltage/1000.0); 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset+58,yOffset+40,"v");
  u8g2.drawLine(xOffset+oled_width-4, yOffset+oled_height-26, xOffset+oled_width-1, yOffset+oled_height-26);
  u8g2.drawLine(xOffset+oled_width-2, yOffset+oled_height-read_ADC-1, xOffset+oled_width-2, yOffset+oled_height-1);
  u8g2.drawLine(xOffset+oled_width-1, yOffset+oled_height-read_ADC-1, xOffset+oled_width-1, yOffset+oled_height-1);
  u8g2.sendBuffer(); // transfer internal memory to the display
}

void showBattery()
{
    checkBattery();
    u8g2.drawLine(xOffset+oled_width-4, yOffset+oled_height-26, xOffset+oled_width-1, yOffset+oled_height-26);
    u8g2.drawLine(xOffset+oled_width-2, yOffset+oled_height-read_ADC-1, xOffset+oled_width-2, yOffset+oled_height-1);
    u8g2.drawLine(xOffset+oled_width-1, yOffset+oled_height-read_ADC-1, xOffset+oled_width-1, yOffset+oled_height-1);
    u8g2.sendBuffer();  
}

void checkBattery()
{
    read_ADC = analogRead(ADC_CHANNEL);
    voltage = read_ADC * 1.485;
    read_ADC = read_ADC/32;
    if (read_ADC > 88) read_ADC=25;
    else if (read_ADC < 63) read_ADC=0;
    else read_ADC = (read_ADC - 63 );
    // shutdown at low battery
    
    if (read_ADC <= 4)
    {
      doLowBatt();
      delay(1000);
      doDeepSleep();
    }
    
}

void showAntiauto_offStatus(void)
{
  char aux[12];
  sprintf(aux, "%s", (bAapo) ? "ON" : "OFF");
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_helvB10_tr); 
  u8g2.drawStr(xOffset,yOffset+11,"Anti APO");
  u8g2.setCursor(xOffset+14, yOffset+40); 
  u8g2.setFont(u8g2_font_helvB18_tr);
  u8g2.printf("%s", aux); 
  u8g2.sendBuffer(); // transfer internal memory to the display
  showBattery();
}

/*********************************************************

 *********************************************************/
 void doFreq(int8_t Direction)
{
  if (Direction == 1)
    rx.setFrequencyUp();
  else
    rx.setFrequencyDown();

  delay(100);
  showStatus();
  resetEepromDelay();
}

void doVolume(int8_t v)
{
  if (v == 1)
    rx.setVolumeUp();
  else
    rx.setVolumeDown();

  showVolume();
  resetEepromDelay();
}

void doStereo(int8_t v)
{
  bSt = (bool)v;
  rx.setMono(!bSt);
  showStereoStatus();
  resetEepromDelay();
}

void doSeek(int8_t seekDirection)
{
  rx.seek(RDA_SEEK_WRAP, seekDirection, showFrequencySeek); // showFrequency will be called by the seek function during the process.
  delay(200);
  showStatusSeek();
  resetEepromDelay();
}

void doPreSleep(void)
{
  if (deep_sleep==false)
  {
  deep_sleep=true;
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(xOffset+5,yOffset+30,"SLEEP");
  u8g2.sendBuffer(); // transfer internal memory to the display
  }
}

void doDeepSleep(void)
{
  rx.powerDown();
  u8g2.setPowerSave(true);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);  //disable timer wake up
  ::esp_deep_sleep_start(); //deep sleep
}

void doLowBatt(void)
{
  deep_sleep=true;
  u8g2.clearBuffer(); // clear the internal memory 
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(xOffset+3,yOffset+30,"LOW Battery");
  u8g2.sendBuffer(); // transfer internal memory to the display
}

void doAntiauto_off(int8_t v)
{
  bAapo = (bool)v;
  showAntiauto_offStatus();
  resetEepromDelay();
}

void loop()
{
  // Check if the buttons have been pressed.
  if (digitalRead(PIN_UP) == LOW)
  {
    elapsedPins = millis();
    if (cmdFreq)
      doFreq(1);
    else if (cmdVolume)
      doVolume(1);
    else if (cmdStereo)
      doStereo(1);
    else if (cmdSeek)
      doSeek(RDA_SEEK_UP);
    else if (cmdBattery)
      showBatteryStatus();
    else if (cmdAntiauto_off)
      doAntiauto_off(1);        
  }

  else if (digitalRead(PIN_DOWN) == LOW)
  {
    elapsedPins = millis();
    if (cmdFreq)
      doFreq(0);
    else if (cmdVolume)
      doVolume(-1);
    else if (cmdStereo)
      doStereo(0);
    else if (cmdSeek)
      doSeek(RDA_SEEK_DOWN);
    else if (cmdBattery)
      showBatteryStatus();
    else if (cmdAntiauto_off)
      doAntiauto_off(0);       
  }

  else if (digitalRead(PIN_MODE) == LOW)
  {
    uint16_t i = 0; 
    while (digitalRead(PIN_MODE) == LOW)
    {
      i++;
      delay(1);
      if (i>2000) doPreSleep();
    } 

    if (deep_sleep==true) doDeepSleep();
    else
    {
      elapsedPins = millis();
      nivelMenu++;
      if (nivelMenu == 1)
      {
        disableCommands();
        cmdVolume = true;
        showVolume();
      }
      else if (nivelMenu == 2)
      {
        disableCommands();
        cmdStereo = true;
        showStereoStatus();
      }
      else if (nivelMenu == 3)
      {
        disableCommands();
        cmdBattery = true;
        showBatteryStatus();
      }
      else if (nivelMenu == 4)
      {
        disableCommands();
        cmdAntiauto_off = true;
        showAntiauto_offStatus();
      }
      else if (nivelMenu == 5)
      {
        disableCommands();
        cmdFreq = true;
        showStatus();
      }
      else
      {
        disableCommands();
        cmdSeek = true;
        showStatusSeek();
        nivelMenu = 0;
      }
      delay(MIN_ELAPSED_TIME);
      elapsedCommand = millis();
      elapsedClick = millis();
    }
  } 

  elapsedClick = millis();

  // Save the current frequency only if it has changed
  if ((currentFrequency = rx.getFrequency()) != previousFrequency)
  {
    if ((millis() - storeTime) > STORE_TIME)
    {
      saveAllReceiverInformation();
      storeTime = millis();
      previousFrequency = currentFrequency;
    }
  }

  // Enter light sleep mode
  if ((millis() - elapsedPins) > ELAPSED_PINS)
  {
    digitalWrite(PIN_DRAIN, LOW);
    digitalWrite(GPIO_NUM_8, HIGH);
    ::esp_light_sleep_start(); //light sleep
    wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) 
    {
      if (cmdBattery) showBatteryStatus();
      else showBattery();
    }
    if (bAapo) {
      digitalWrite(PIN_DRAIN, HIGH);
      digitalWrite(GPIO_NUM_8, LOW);
    }
    elapsedPins = millis();
  }
  delay(5);
}
