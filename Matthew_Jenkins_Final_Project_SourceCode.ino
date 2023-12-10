//Included libraries
#include <dht.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>
//macros
#define RDA 0x80
#define TBE 0x20  
//PORT AND PIN SETUP VARIBLES
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20;

volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x25;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

volatile unsigned char* port_d = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* pin_d = (unsigned char*) 0x29;

volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int *myTCNT1 = (unsigned int *) 0x84;
volatile unsigned char *myTIFR1 = (unsigned char *) 0x36;
volatile unsigned char *myUCSR0A = (unsigned char *)0xC0;
volatile unsigned char *myUCSR0B = (unsigned char *)0xC1;
volatile unsigned char *myUCSR0C = (unsigned char *)0xC2;
volatile unsigned int *myUBRR0 = (unsigned int *) 0xC4;
volatile unsigned char *myUDR0 = (unsigned char *)0xC6;
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//LDC pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//VARIBLES
char state = 'd'; //where all the states are stored and accessed 'd'- disabled, 'i'- idle, 'r'- running, 'e'- error
bool statebutton_pressed = false;
int waterlevel = 0;
const int stepsPerRevolution = 200; 
Stepper myStepper(stepsPerRevolution, 23, 27, 25, 29);
dht DHT;
float temp = 0.0;
float tempthreshold = 20.0;
unsigned int currentTicks = 0;
int ErrorCode = 0;
const int ISRbuttonPin = 19;
volatile bool buttonPressed = false;
unsigned int lcdcount = 0;
unsigned char adc = 0;
RTC_DS1307 rtc;

void setup() {
  //leds (R,Y,B,G)
  *ddr_a  |= 0x01 << 0;
  *ddr_a  |= 0x01 << 2;
  *ddr_a  |= 0x01 << 4;
  *ddr_a  |= 0x01 << 6;
  //buttons(ControlL, ControlR, ON/OFF machine)
  *ddr_d  &= 0x01 << 7;
  *port_d |= 0x01 << 7;
  *ddr_g &= 0x01 << 1;
  *port_g |= 0x01 << 1;
  *ddr_d &= 0x01 << 1;
  *port_d |= 0x01 << 1;
  //sensors(Temp,Water,Water)
  *ddr_c &= 0x01 << 6;
  *port_c |= 0x01 << 6;
  *ddr_c &= 0x01 << 4;
  *port_c |= 0x01 << 4;
  *ddr_c |= 0x01 << 2;
  //fanmotor
  *ddr_c |= 0x01 << 5;
  Wire.begin();
  rtc.begin();
  Serial.begin(9600);
  U0init(9600);
  adc_init();
  lcd.begin(16, 2);
  attachInterrupt(digitalPinToInterrupt(ISRbuttonPin), buttonISR, RISING);
  myStepper.setSpeed(60);
}

int x = 0;

void loop() {
  if(buttonPressed){
    if(state == 'd'){
      state = 'i';
    } else if (state == 'e'){
      state = 'i';
      printTimeStamp();
    } else {
      state = 'd';
    }
    buttonPressed = false;
  }
  if(state == 'd'){
     *port_a |= (0x01 << 2);
     *port_a &= ~(0x01 << 0);
     *port_a &= ~(0x01 << 4);
     *port_a &= ~(0x01 << 6);
     stopdownFan();
     if(lcdcount == 0){
      lcdcount++;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DISABLED!");
     }
     x = 0;
  } else if (state == 'i'){
     *port_a &= ~(0x01 << 2);
     *port_a &= ~(0x01 << 0);
     *port_a &= ~(0x01 << 4);
     *port_a |= (0x01 << 6);
     if(x == 0){
      printTimeStamp();
      x++;
     }
     check_temp_and_hum();
     lcdcount = 0;
  } else if (state == 'e'){
     *port_a &= ~(0x01 << 2);
     *port_a |= (0x01 << 0);
     *port_a &= ~(0x01 << 4);
     *port_a &= ~(0x01 << 6);
     stopdownFan();
     lcdcount = 0;
     x = 0;
  } else if (state == 'r'){
     *port_a &= ~(0x01 << 2);
     *port_a &= ~(0x01 << 0);
     *port_a |= (0x01 << 4);
     *port_a &= ~(0x01 << 6);
     startupFan();
     check_temp_and_hum();
     lcdcount = 0;
     x = 0;
  }

  if (*pin_g & (0x01 << 1)) {
    myStepper.step(stepsPerRevolution);
    printTimeStamp();
  } else if (*pin_d & (0x01 << 7)) {
    myStepper.step(-stepsPerRevolution);
    printTimeStamp();
  } else {
    myStepper.step(0);
  }
}

void check_temp_and_hum(){
  int chk = DHT.read11(31);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temperature: ");
  lcd.print(DHT.temperature);
  temp = DHT.temperature;
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  //startupFan();
  minDelay(60000);
}

void minDelay(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while ((millis() - startTime) < milliseconds) {
    unsigned int waterlevel = adc_read(adc);
    if(temp > tempthreshold && state != 'r'){
      state = 'r';
      break;
    } else if (state == 'r' && temp < tempthreshold) {
      state = 'i';
      break;
    }
    if(waterlevel < 200){
      state = 'e';
      ErrorCode = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ERROR: ");
      lcd.setCursor(0, 1);
      lcd.print("LOW WATER");
      break;
    }
    if (*pin_g & (0x01 << 1)) {
      myStepper.step(stepsPerRevolution);
      printTimeStamp();
    } else if (*pin_d & (0x01 << 7)) {
      myStepper.step(-stepsPerRevolution);
      printTimeStamp();
    } else {
      myStepper.step(0);
    }
    if(buttonPressed){
      state = 'd';
      buttonPressed = false;
      break;
    }
  }
}

void startupFan(){
  *port_c |= (0x01 << 5);
}

void stopdownFan(){
  *port_c &= ~(0x01 << 5);
}

void buttonISR(){
    buttonPressed = true;
}

void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void adc_init()
{
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000; 
  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  *my_ADMUX  &= 0b01111111;
  *my_ADMUX  |= 0b01000000;
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

void printTimeStamp() {
  DateTime now = rtc.now();
  char time_stamp[2];
  dtostrf(now.year(),4,2,time_stamp);
  for(int i = 0; i < 4; i++){
    U0putchar(time_stamp[i]);
  }
  U0putchar('/');
  dtostrf(now.month(),2,0,time_stamp);
  for(int i = 0; i < 2; i++){
    U0putchar(time_stamp[i]);
  }
  U0putchar('/');
  dtostrf(now.day(),2,0,time_stamp);
  for(int i = 0; i < 2; i++){
    U0putchar(time_stamp[i]);
  }
  U0putchar(' ');
  dtostrf(now.hour(),2,0,time_stamp);
  for(int i = 0; i < 2; i++){
    U0putchar(time_stamp[i]);
  } 
  U0putchar(':');
  dtostrf(now.minute(),2,0,time_stamp);
  for(int i = 0; i < 2; i++){
    U0putchar(time_stamp[i]);
  }
  U0putchar(':');
  dtostrf(now.second(),2,0,time_stamp);
  for(int i = 0; i < 2; i++){
    U0putchar(time_stamp[i]);
  }
  U0putchar('\n');
}
