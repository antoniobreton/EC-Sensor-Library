#include <ECMeter.h>

const float EC_Meter::tempCoefEC        = 0.019;


//Class Constructor//
EC_Meter::EC_Meter(uint8_t i2cAddress){
    _address = i2cAddress;
    Wire.begin();
}

EC_Meter::EC_Meter(){
    _address = EC_METER;
    Wire.begin();
}

//Class Destructor//
EC_Meter::~EC_Meter(){}

// EC-Meter measureEC(float tempCoefficient, bool newTemp) Function
// Code eg: float mS = EC_Meter::measureEC(ec.tempCoefEC, true)
// Starts an EC Measurement:
// The device starts an EC Measurement:
// Param: tempCoefficient- The coefficient used to compensate for temperature.
// Param: newTemp- Boolean to take a new temperature measurement
// post #uS, #mS, #uS, #tempC, #tempF, #PPM500, #PPM640, #PPM700 are updaterd
// returns mS

float EC_Meter::measureEC(float tempCoefficient, bool newTemp){

    if(newTemp){
        measureTemp();
    }

    _write_register(EC_TEMPCOEF_REGISTER, tempCoefficient);
    _send_command(EC_MEASURE_EC);
    delay(EC_EC_MEASUREMENT_TIME);
    mS = _read_register(EC_MS_REGISTER);

    if (mS == mS){
        PPM500 = mS * 500;
        PPM640 = mS * 640;
        PPM700 = mS * 700;
        uS     = mS * 1000;
        uS     = mS / 1000;
    }

    return mS;
}

// EC-Meter measureEC() Function
// Code eg: ms = EC_Meter::measureEC();
// Convenience function to measure EC in freshwater
// Calls #EC_Meter::measureEC(#EC_Meter::tempCoefEC, #EC_Meter::usingTemperatureCompensation())

float EC_Meter::measureEC(){

    return measureEC(tempCoefEC, usingTemperatureCompensation());
}

// EC-Meter measureTemp() Function
// Code eg: tempC = EC_Meter::measureTemp();
// Starts a temperature measurement
// post #tempC and #tempF are updated
// Note: A value of -127 means the device is not connected
// return temperrature in C

float EC_Meter::measureTemp(){
    _send_command(EC_MEASURE_TEMP);
    delay(EC_TEMP_MEASUREMENT_TIME);
    tempC = _read_register(EC_TEMP_REGISTER);
    tempF = ((tempC * 9) / 5) + 32;
    return tempC;
}

// EC-Meter setK(float k) Function
// Code eg: EC_Meter::setK(float k);
// Updates the device with a new cell constant and saves it in the EEPROM
// Param: K - The new cell constant

void EC_Meter::setK(float k){
    _write_register(EC_K_REGISTER, k);
}

// EC-Meter getK() Function
// Code eg: k = EC_Meter::getK();
// Retrieves the cell constant from the device
// return the new cell constant

float EC_Meter::getK(){
    return _read_register(EC_K_REGISTER);
}

// EC-Meter useTemperatureCompensation(bool b) Function
// Code eg: EC_Meter::useTemperatureCompensation(true);
// Configures device to use temperature compensation or not
// Param: B - true/false

void EC_Meter::useTemperatureCompensation(bool b){
    uint8_t retval;
    uint8_t config = _read_byte(EC_CONFIG_REGISTER);

    if(b){
        retval = bitSet(config, EC_TEMP_COMPENSATION_CONFIG_BIT);
    }
    else{
        retval = bitClear(config, EC_TEMP_COMPENSATION_CONFIG_BIT);
    }

    _write_byte(EC_CONFIG_REGISTER, retval);
}

// EC-Meter usingTemperatureCompensation() Function
// Code eg: EC_Meter::usingTemperatureCompensation();
// Determines if temperature compensation is being used
// return true if using compensation, false otherwise

bool EC_Meter::usingTemperatureCompensation(){
    uint8_t retval;

    retval = _read_byte(EC_CONFIG_REGISTER);
    return (retval >> 1) & 0x01;
}

// EC-Meter getVersion() Function
// Code eg: version = EC_Meter::getVersion();
// Retrieves the firmware version of the device
// return version of firmware

uint8_t EC_Meter::getVersion(){
    return _read_byte(EC_VERSION_REGISTER);
}

// EC-Meter setTempConstant(uint8_t b) Function
// Code eg: EC_Meter::setTempConstant(25); EC_Meter::setTempConstant(0xFF) // Use the actual temperature
// Configures device to use the provided temperature constant
// By default, the temperature constant is set to 0xFF which instructs the actual temperature to be used for
// temperature compensation, however any number can be specified. Tu use the actual temperature, resture the value
// to 0xFF
// Param: b the temperature tu use for compensation

void EC_Meter::setTempConstant(uint8_t b){
    _write_byte(EC_TEMP_COMPENSATION_REGISTER, b);
} 

// EC-Meter getTempConstant() Function
// Code eg: uint8_t tempConst = EC_Meter::getTempConstant();
// Retreives the temperature constant from the device
// return the temperature constant use for compensation

uint8_t EC_Meter::getTempConstant(){
    _read_byte(EC_TEMP_COMPENSATION_REGISTER);
}

// EC-Meter useDualPoint(bool b) Function
// Code eg: EC_Meter::useDualPoint(true);
// Configures device to use dual-point calibration
// Param: b true/false
void EC_Meter::useDualPoint(bool b){
    uint8_t retval;
    uint8_t config = _read_byte(EC_CONFIG_REGISTER);

    if (b){
        retval = bitSet(config, EC_DUALPOINT_CONFIG_BIT);
    }
    else{
        retval = bitClear(config, EC_DUALPOINT_CONFIG_BIT);
    }

    _write_byte(EC_CONFIG_REGISTER, retval);
}

// EC-Meter usingDualPoint() Function
// Code eg: EC_Meter::usingDualPoint();
// Determines if dual-point calibration is being used
// return true if using dual-point, false otherwise
bool EC_Meter::usingDualPoint(){
    uint8_t retval;

    retval = _read_byte(EC_CONFIG_REGISTER);
    return (retval >> 0) & 0x01;
}

// EC-Meter useInterruptionMode(bool b) Function
// Code eg: EC_Meter::useInterruptionMode(true);
// Configures device to use interruption mode
// Param: b true/false
void EC_Meter::useInterruptionMode(bool b){
    uint8_t retval;
    uint8_t config = _read_byte (EC_CONFIG_REGISTER);

    if (b){
        retval = bitSet(config, EC_INTERRUPTION_CONFIG_BIT);
    }
    else{
        retval = bitClear(config, EC_INTERRUPTION_CONFIG_BIT);
    }

    _write_byte(EC_CONFIG_REGISTER, retval);
}

// EC-Meter usingInterruptionMode() Function
// Code eg: EC_Meter::usingInterruptionMode();
// Determines if interruption mode is being used
// return true, return false otherwise
bool EC_Meter::usingInterruptionMode(){
    uint8_t retval;
    
    retval = _read_byte(EC_CONFIG_REGISTER);
    return (retval >> 2) & 0x01;
}

// EC-Meter calibrateProbe(float solutionEC, float tempCoef) Function
// Code eg: EC_Meter::calibrateProbe(2.77, ec.tempCoefEC);
// Calibrates the connected probe and saves the result in EEPROM
// Param: solutionEC- The EC of the calibration in mS
// Param: tempCoef -  The coefficient used to calibrate the probe
// post               Offset will be saved in the device's EEPROM and used automatically thereafter

void EC_Meter::calibrateProbe(float solutionEC, float tempCoef){
    bool dualpoint= usingDualPoint();

    useDualPoint(false);
    _write_register(EC_TEMPCOEF_REGISTER, tempCoef);
    _write_register(EC_SOLUTION_REGISTER, solutionEC);
    _send_command(EC_CALIBRATE_PROBE);
    delay(EC_EC_MEASUREMENT_TIME);
    useDualPoint(dualpoint);
}

// EC-Meter setCalibrateOffset(float offset) Function
// Code eg: EC_Meter::calibrateOffset(1.7)
// Sets the single point offset value to the device
// Param: offset- single point ofsset value
void EC_Meter::setCalibrateOffset(float offset){
    _write_register(EC_CALIBRATE_OFFSET_REGISTER, offset);
}

// EC-Meter getCalibrateOffset() Function
// Code eg: float offset = EC_Meter::getCalibrateOffset();
// Retrieves the single point offset value from the device
// return single point offset value
float EC_Meter::getCalibrateOffset(){
    return _read_register(EC_CALIBRATE_OFFSET_REGISTER);
}

// EC-Meter calibrateProbeHigh(float solutionEC, float tempCoef) Function
// Code eg: EC_Meter::calibrateProbeHigh(3.0 , EC_Meter::tempCoefEC);
// Calibrates the dual-point values for high reading and saves them in the decice's EEPROM
// Param: solutionEC- The EC of the calibration solution in mS
// Param: tempCoef- The coefficient used to calibrate the probe
void EC_Meter::calibrateProbeHigh(float solutionEC, float tempCoef){
    bool dualpoint = usingDualPoint();

    useDualPoint(false);
    _write_register(EC_TEMPCOEF_REGISTER, tempCoef);
    _write_register(EC_SOLUTION_REGISTER, solutionEC);
    _send_command(EC_CALIBRATE_HIGH);
    delay(EC_EC_MEASUREMENT_TIME);
    useDualPoint(dualpoint);
}

// EC-Meter calibrateProbeLow(float solutionEC, float tempCoef) Function
// Code eg: EC_Meter::calibrateProbeLow(1.0 , EC_Meter::tempCoefEC);
// Calibrates the dual-point values for Low reading and saves them in the decice's EEPROM
// Param: solutionEC- The EC of the calibration solution in mS
// Param: tempCoef- The coefficient used to calibrate the probe
void EC_Meter::calibrateProbeLow(float solutionEC, float tempCoef){
    bool dualpoint = usingDualPoint();

    useDualPoint(false);
    _write_register(EC_TEMPCOEF_REGISTER, tempCoef);
    _write_register(EC_SOLUTION_REGISTER, solutionEC);
    _send_command(EC_CALIBRATE_LOW);
    delay(EC_EC_MEASUREMENT_TIME);
    useDualPoint(dualpoint);
}

// EC-Meter setDualPointCalibration(float refLow, float refHigh, float readLow, float readHigh) Function
// Code eg: EC_Meter::setDualPointCalibration(1.0, 3.0, 0.9, 3.2)
// Set all the values for a dual point calibration and saves them in the device's EEPROM
// Param: refLow           The reference Low Point
// Param: refHigh          The reference High Point
// Param: readLow          The measured low point in mS
// Param: readHigh         The measured high point in mS
void EC_Meter::setDualPointCalibration(float refLow, float refHigh, float readLow, float readHigh){
    _write_register(EC_CALIBRATE_REFLOW_REGISTER, refLow);
    _write_register(EC_CALIBRATE_REFHIGH_REGISTER, refHigh);
    _write_register(EC_CALIBRATE_READLOW_REGISTER, readLow);
    _write_register(EC_CALIBRATE_READHIGH_REGISTER, readHigh);
}

// EC-Meter getCalibrateHigh() Function
// Code eg: float high = EC_Meter::getCalibrateHigh();
// Retrieves the dual-point calibration high value
// return the dual-point calibration high value
float EC_Meter::getCalibrateHigh(){
    return _read_register(EC_CALIBRATE_REFHIGH_REGISTER);
}

// EC-Meter getCalibrateLow() Function
// Code eg: float low = EC_Meter::getCalibrateLow();
// Retrieves the dual-point calibration low value
// return the dual-point calibration low value
float EC_Meter::getCalibrateLow(){
    return _read_register(EC_CALIBRATE_REFLOW_REGISTER);
}

// EC-Meter getCalibrateHighReading() Function
// Code eg: float highRead = EC_Meter::getCalibrateHighReading();
// Retrieves the dual-point calibration reading high value
// return the dual-point calibration reading high value
float EC_Meter::getCalibrateHighReading(){
    return _read_register(EC_CALIBRATE_READHIGH_REGISTER);
}

// EC-Meter getCalibrateLowReading() Function
// Code eg: float lowRead = EC_Meter::getCalibrateLowReading();
// Retrieves the dual-point calibration reading low value
// return the dual-point calibration reading low value
float EC_Meter::getCalibrateLowReading(){
    return _read_register(EC_CALIBRATE_READLOW_REGISTER);
}

// EC-Meter calibrateDry() Function
// Code eg: float EC_Meter::calibrateDry();
// Measures the conductivity when the probe is dry
void EC_Meter::calibrateDry(){
    _send_command(EC_DRY);
    delay(EC_EC_MEASUREMENT_TIME);
}

// EC-Meter getCalibrateDry() Function
// Code eg: float dry = EC_Meter::getCalibrateDry();
// Gets the dry reading of the probe and saves the result in EEPROM
float EC_Meter::getCalibrateDry(){
    return _read_register(EC_DRY_REGISTER);
}

// EC-Meter setI2CAddress() function
// Code eg: setI2CAddress(0x4C);
// Changes the i2c address of the device
// If the default address of the device needs to be changed, call this 
// function to permanently change the address If you forget the i2c addres
// you will need to use an i2c scanner to recover it

void EC_Meter::setI2CAddress(uint8_t i2cAddress){
    _write_register(EC_SOLUTION_REGISTER, i2cAddress);
    _send_command(EC_I2C);
    _address = i2cAddress;
}



// EC-Meter reset(); Function
// Code eg: EC_Meter::reset(); 
// Reset All the stored Calibration information
void EC_Meter::reset(){
    _write_register(EC_K_REGISTER,                  NAN);
    _write_register(EC_CALIBRATE_OFFSET_REGISTER,   NAN);
    _write_register(EC_CALIBRATE_REFHIGH_REGISTER,  NAN);
    _write_register(EC_CALIBRATE_REFLOW_REGISTER,   NAN);
    _write_register(EC_CALIBRATE_READHIGH_REGISTER, NAN);
    _write_register(EC_CALIBRATE_READLOW_REGISTER,  NAN);
    setTempConstant(0);
    useDualPoint(false);
    useTemperatureCompensation(false);
    useInterruptionMode(false);
}



///////////////////////////////////
//COMUNICATION PROTOCOL FUNCTIONS//
///////////////////////////////////


void EC_Meter::_change_register(uint8_t r){
    Wire.beginTransmission(_address);
    Wire.write(r);
    Wire.endTransmission();
    delay(10);
}

void EC_Meter::_send_command(uint8_t command){
    Wire.beginTransmission(_address);
    Wire.write(EC_TASK_REGISTER);
    Wire.write(command);
    Wire.endTransmission();
    delay(10);
}

void EC_Meter::_write_register(uint8_t reg, float f){
    uint8_t b[5];
    float f_val = f;
    

    b[0] = reg;
    b[1] = *((uint8_t *)&f_val);
    b[2] = *((uint8_t *)&f_val + 1);
    b[3] = *((uint8_t *)&f_val + 2);
    b[4] = *((uint8_t *)&f_val + 3);

    Wire.beginTransmission(_address);
    Wire.write(b, 5);
    Wire.endTransmission();
    delay(10);
}

float EC_Meter::_read_register(uint8_t reg){

    float retval;

    _change_register(reg);
    Wire.requestFrom(_address, (uint8_t)1);
    *((uint8_t *)&retval) = Wire.read();
    Wire.requestFrom(_address, (uint8_t)1);
    *((uint8_t *)&retval + 1) = Wire.read();
    Wire.requestFrom(_address, (uint8_t)1);
    *((uint8_t *)&retval + 2) = Wire.read();
    Wire.requestFrom(_address, (uint8_t)1);
    *((uint8_t *)&retval + 3) = Wire.read();
    delay(10);
    
    return retval;

}

void EC_Meter::_write_byte(uint8_t reg, uint8_t val){

    uint8_t b[5];

    b[0] = reg;
    b[1] = val;
    Wire.beginTransmission(_address);
    Wire.write(b, 2);
    Wire.endTransmission();
    delay(10);
}

uint8_t EC_Meter::_read_byte(uint8_t reg){
    uint8_t retval;

    _change_register(reg);
    Wire.requestFrom(_address, (uint8_t)1);
    retval = Wire.read();
    delay(10);
    return retval;
}

