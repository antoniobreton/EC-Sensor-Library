#ifndef EC_METER_H
#define EC_METER_H

#include <math.h>
#include <Arduino.h>
#include <Wire.h>

#define EC_METER 0x4C

//*************************************DEFINITION OF COMMAND INDEX***************************************************************//
#define EC_MEASURE_EC 80                                    // Task register for measure EC
#define EC_MEASURE_TEMP 40                                  // Task register for measure Temperature
#define EC_CALIBRATE_PROBE 20                               // Task register for calibrate probe offset
#define EC_CALIBRATE_LOW 10                                 // Task index for calibrate low
#define EC_CALIBRATE_HIGH 8                                 // Task index for calibrate high
#define EC_DRY 4                                            // Task index for measure EC when probe is dry
#define EC_I2C 2                                            // Task index that runs the change I2C address
//*************************************END OF DEFINITION OF COMMAND INDEX********************************************************//

//*************************************DEFINITION OF VARIABLE REGISTERS**********************************************************//
#define EC_VERSION_REGISTER 0                               // Version                  register
#define EC_TEMP_COMPENSATION_REGISTER 1                     // Temperature compensation register
#define EC_CONFIG_REGISTER 2                                // Configuration            register
#define EC_TASK_REGISTER 3                                  // Task                     register
#define EC_MS_REGISTER 4                                    // Milli Siemens            register
#define EC_TEMP_REGISTER 8                                  // Temperature in C         register
#define EC_K_REGISTER 12                                    // Cell constant            register
#define EC_SOLUTION_REGISTER 16                             // Calibration solution     register
#define EC_TEMPCOEF_REGISTER 20                             // Temperature Coefficient  register
#define EC_CALIBRATE_OFFSET_REGISTER 24                     // Calibration offset       register
#define EC_CALIBRATE_REFHIGH_REGISTER 28                    // Reference High           register
#define EC_CALIBRATE_REFLOW_REGISTER 32                     // Reference Low            register
#define EC_CALIBRATE_READHIGH_REGISTER 36                   // Reading High             register
#define EC_CALIBRATE_READLOW_REGISTER 40                    // Reading Low              register
#define EC_DRY_REGISTER 44                                  // Dry EC measurement       register 

#define EC_EC_MEASUREMENT_TIME 1150                         // Delay between EC Measurements
#define EC_TEMP_MEASUREMENT_TIME 750                        // Delay for Temperature measurement

#define EC_DUALPOINT_CONFIG_BIT 0                           // Dual Point config Bit
#define EC_TEMP_COMPENSATION_CONFIG_BIT 1                   // Temperature compensation config Bit
#define EC_INTERRUPTION_CONFIG_BIT 2                        // Interruption mode config Bit


//*************************************END OF DEFINITION OF VARIABLE REGISTERS***************************************************//

//*************************************EC_METER CLASS DEFINITION*****************************************************************//

class EC_Meter{

    public:
        EC_Meter(uint8_t i2cAddress);
        EC_Meter();
        ~EC_Meter();
        float S;                                                // EC in Siemens
        float mS;                                               // EC in milli-Siemens
        float uS;                                               // EC in micro-Siemens
        long PPM500;                                            // Parts per million using 500 as a multiplier
        long PPM640;                                            // Parts per million using 640 as a multiplier
        long PPM700;                                            // Parts per million using 700 as a multiplier
        float tempC;                                            // Temperature in Celcius
        float tempF;                                            // Temperature in Farenheit
        static const float tempCoefEC;                          // Temperature compensation coefficient for EC measurement
        float measureEC(float tempCoefficient, bool newTemp);   // Measure EC with tempCoefficient and newTemp as parameters
        float measureEC();                                      // Measure EC 
        float measureTemp();                                    // Measure Temperature
        void setK(float k);                                     // Set K constant cell to de device
        float getK();  
        void setTempConstant(uint8_t b);
        uint8_t getTempConstant();
        void useTemperatureCompensation(bool b);
        bool usingTemperatureCompensation();
        void useDualPoint(bool b);
        bool usingDualPoint();
        void useInterruptionMode(bool b);
        bool usingInterruptionMode();
        uint8_t getVersion();
        void calibrateProbe(float solutionEC, float tempCoef);
        void setCalibrateOffset(float offset);
        float getCalibrateOffset();
        void calibrateProbeHigh(float solutionEC, float tempCoef);
        void calibrateProbeLow(float solutionEC, float tempCoef);
        float getCalibrateHigh();
        float getCalibrateLow();
        float getCalibrateHighReading();
        float getCalibrateLowReading();
        float getCalibrateDry();
        void reset();
        void setDualPointCalibration(float refLow, float refHigh, float readLow, float readHigh);
        void calibrateDry();
        void setI2CAddress(uint8_t i2cAddress);
        
     private:
        uint8_t _address;
        void _change_register(uint8_t r);
        void _send_command(uint8_t command);
        void _write_register(uint8_t reg, float f);
        void _write_byte(uint8_t reg, uint8_t val);
        float _read_register(uint8_t reg);
        uint8_t _read_byte(uint8_t reg);

};

#endif