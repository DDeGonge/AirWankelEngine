#include <HX711_ADC.h>

#define encoderPinA 0 //RXD0 PB16
#define encoderPinB 1 //TXD0 PB17

#define HX711_dout 12 //mcu > HX711 dout pin
#define HX711_sck 13 //mcu > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);
static float calibrationValue = -1883854.87;
static float arm_len_m = 0.05;
static float ticks_per_rev = 2400;
static float g_accel = 9.81;
static int readrate_hz = 25;

volatile long encoderTicks = 0;

void setup() {
  //encoder IO setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(encoderPinA, handleA, CHANGE);
  attachInterrupt(encoderPinB, handleB, CHANGE);

  Serial.begin(115200);
  while (!Serial);

  LoadCell.begin();
  LoadCell.start(2000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    LoadCell.setSamplesInUse(2);
    Serial.println("HX711 Startup is complete");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float time_elapsed_s = 0, load_mNm = 0, vel_rps = 0, power_W = 0, power_hp = 0, work_Wh = 0;
  long last_encoderTicks = 0;
  long readInc_us = 1000000 / readrate_hz;
  long lastRead_us = micros();

  // Flush out any invalid reads
  for(uint8_t i = 0; i < 10; i++)
  {
    LoadCell.update();
    delay(100);
  }

  while(true)
  {
    while(micros() < (lastRead_us + readInc_us)) {};

    // calculate torque and power
    time_elapsed_s = (micros() - lastRead_us);
    time_elapsed_s /= 1000000;
    vel_rps = abs(encoderTicks - last_encoderTicks);
    vel_rps /= (ticks_per_rev * time_elapsed_s);
    LoadCell.update();
    load_mNm = LoadCell.getData() * g_accel * arm_len_m * 1000;
    power_W = (load_mNm * vel_rps * 60) / 9548.8;
    power_hp = power_W / 745.7;
    work_Wh += (power_W * time_elapsed_s) / (3600);

    // print results
    Serial.print("torque_mNm:");
    Serial.print(load_mNm);
    Serial.print("\tpower_mW:");
    Serial.print(power_W * 1000);
//    Serial.print("\tpower_hp:");
//    Serial.print(power_hp);
    Serial.print("\twork_milliWatt/hr:");
    Serial.print(work_Wh * 1000);
    Serial.println("");

    // 2L at 60psi is ~2.3 watt/hrs

    // save last values
    last_encoderTicks = encoderTicks;
    lastRead_us = micros();
  }
}

/* EXTERNAL INTERRUPT PROTOCOLS */
void handleA()
{
  bool aState = 0, bState = 0; //sets states low as default
  uint32_t readval = PORT->Group[PORTB].IN.reg; //reads portA
  
  if (readval & (1<<16)) //if register for pin A is high, make aState high
    aState = 1;
  if (readval & (1<<17)) //same for B
    bState = 1;
    
  if ((!bState && aState) || (bState && !aState)) //increment or decrement encoder count appropriately
    encoderTicks++;
  else
    encoderTicks--;
}

void handleB()
{
  bool aState = 0, bState = 0;
  uint32_t readval = PORT->Group[PORTB].IN.reg;
  
  if (readval & (1<<16))
    aState = 1;
  if (readval & (1<<17))
    bState = 1;
    
  if ((!bState && !aState) || (bState && aState))
    encoderTicks++;
  else
    encoderTicks--;
}
