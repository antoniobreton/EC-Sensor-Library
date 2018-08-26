#include <ECMeter.h>

EC_Meter ec;
float calibrationSolution_High = 1.2;
float calibrationSolution_Low  = 0.08;
int calibratingLow;
int calibratingHigh;

void setup() {
    // Initialize Serial COMM
    Serial.begin(9600);
    //Wait until the device initialize
  
    //Initializing
    Serial.println("Initializing Sensor...");
    delay(6000);

    //Reset all parameters
    ec.reset();

    //Setting the cell K constant to the device
    ec.setK(1.4);
    // Config the temperature compensation as true
    ec.useTemperatureCompensation(true);
    // Setting temperature constant as 25°C 
    ec.setTempConstant(25); 
    // Config the intterruption as true
    ec.useInterruptionMode(true);

    //Get the k cell constant
    Serial.print("K: "); Serial.println(ec.getK());
    Serial.print("High Reading: "); Serial.println(ec.getCalibrateHighReading());
    Serial.print("High Reference: "); Serial.println(ec.getCalibrateHigh());
    Serial.print("Low Reading: "); Serial.println(ec.getCalibrateLowReading());
    Serial.print("Low Reference: "); Serial.println(ec.getCalibrateLow());
    Serial.print("Temperature Constant: "); Serial.print(ec.getTempConstant());Serial.println("°C");
    delay(2000);
}

void loop() {
    
  // If using interruption mode print using interruption mode
  if (ec.usingInterruptionMode()){
      Serial.println("USING INTERRUPTION MODE");
  }
  else{
      Serial.println("NOT USING INTERRUPTION MODE");
  }

  delay(2000);
 
  // Reads the conductivity measurement
  float mS = ec.measureEC();
  Serial.print("mS: "); Serial.println(mS, 2);
  delay(2000);

  uint8_t tempConstant = ec.getTempConstant();
  Serial.print("Temperature Constant: "); Serial.println(tempConstant);
  delay(2000);

}