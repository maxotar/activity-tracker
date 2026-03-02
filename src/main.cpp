#include "LSM6DS3.h"
#include "Wire.h"

// IMU instance
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Configuration
#define ODR_HZ 52
#define WINDOW_DURATION_S 10
#define SAMPLES_PER_WINDOW (ODR_HZ * WINDOW_DURATION_S) // 520
#define FIFO_WORDS (SAMPLES_PER_WINDOW * 3)             // 1560 words

// Data structure
struct DataPoint
{
    uint16_t rms_mg;
    uint16_t variance_mg2;
    uint16_t peak_mg;
    uint16_t temp_f_x10;
};

// Storage
#define MAX_SAMPLES 15000
DataPoint dataLog[MAX_SAMPLES];
uint16_t currentSample = 0;

// Compute functions
uint16_t computeRMS(float *data, int n)
{
    float sumSquares = 0;
    for (int i = 0; i < n; i++)
    {
        sumSquares += data[i] * data[i];
    }
    return (uint16_t)(sqrt(sumSquares / n) * 1000);
}

uint16_t computeVariance(float *data, int n)
{
    float mean = 0;
    for (int i = 0; i < n; i++)
        mean += data[i];
    mean /= n;

    float variance = 0;
    for (int i = 0; i < n; i++)
    {
        float diff = data[i] - mean;
        variance += diff * diff;
    }
    variance /= n;

    return (uint16_t)(variance * 1000000);
}

uint16_t computePeak(float *data, int n)
{
    float peak = 0;
    for (int i = 0; i < n; i++)
    {
        if (data[i] > peak)
            peak = data[i];
    }
    return (uint16_t)(peak * 1000);
}

// Read FIFO sample count
uint16_t getFIFOSamples()
{
    uint8_t statusLow, statusHigh;

    // Read FIFO_STATUS1 (bits 7-0 of sample count)
    myIMU.readRegister(&statusLow, LSM6DS3_ACC_GYRO_FIFO_STATUS1);

    // Read FIFO_STATUS2 (bits 11-8 of sample count)
    myIMU.readRegister(&statusHigh, LSM6DS3_ACC_GYRO_FIFO_STATUS2);

    return ((statusHigh & 0x0F) << 8) | statusLow;
}

void setupIMU()
{
    Serial.println("Configuring IMU...");

    // Disable gyroscope (saves power)
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);

    // Configure accelerometer: 52 Hz, ±8g, low power mode
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x3C);

    // Configure FIFO
    // 0x01: Accel no decimation. (Your previous 0x09 included Gyro data!)
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, 0x01);

    // 0x1E: FIFO ODR 52Hz (0011) + Continuous mode (110)
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0x1E);

    // Set FIFO watermark to 1560 words
    uint16_t watermark = FIFO_WORDS;
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, watermark & 0xFF);
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, (watermark >> 8) & 0x0F);

    Serial.println("IMU configured!");
}

void setup()
{
    Serial.begin(115200);
    // Wait for serial monitor to connect (optional, but good for debugging)
    while (!Serial)
        ;
    delay(1000);

    Serial.println("\n=== Dog Tracker V1 - FIFO Test ===\n");

    Serial.print("Initializing IMU... ");
    if (myIMU.begin() != 0)
    {
        Serial.println("FAILED!");
        while (1)
            ; // Halt if IMU isn't found
    }
    Serial.println("OK\n");

    setupIMU();

    Serial.print("\nStorage capacity: ");
    Serial.print(MAX_SAMPLES);
    Serial.println(" logs");

    // Updated to reflect the new Words vs Samples math
    Serial.print("FIFO target: ");
    Serial.print(SAMPLES_PER_WINDOW);
    Serial.print(" samples (");
    Serial.print(FIFO_WORDS);
    Serial.print(" words × 2 bytes = ");
    Serial.print(FIFO_WORDS * 2);
    Serial.println(" bytes) (< 4096 ✓)");

    Serial.println("\nWaiting for FIFO to fill (~10 seconds)...\n");
}
void loop()
{
    Serial.print("Waiting for FIFO... ");

    // Wait for the FIFO to fill with our target number of words
    uint16_t fifoWords = 0;
    while (fifoWords < FIFO_WORDS)
    {
        fifoWords = getFIFOSamples(); // This is returning words, not full XYZ samples
        delay(100);
    }

    Serial.println("FULL!");

    float accelMag[SAMPLES_PER_WINDOW];
    Serial.print("Reading FIFO... ");

    // Read the exact number of 3-axis samples
    for (int i = 0; i < SAMPLES_PER_WINDOW; i++)
    {
        uint8_t lsb, msb;
        int16_t rawX, rawY, rawZ;

        // Pop X
        myIMU.readRegister(&lsb, 0x3E); // FIFO_DATA_OUT_L
        myIMU.readRegister(&msb, 0x3F); // FIFO_DATA_OUT_H
        rawX = (msb << 8) | lsb;

        // Pop Y
        myIMU.readRegister(&lsb, 0x3E);
        myIMU.readRegister(&msb, 0x3F);
        rawY = (msb << 8) | lsb;

        // Pop Z
        myIMU.readRegister(&lsb, 0x3E);
        myIMU.readRegister(&msb, 0x3F);
        rawZ = (msb << 8) | lsb;

        // Convert raw LSBs to g (±8g scale = 0.244 mg / LSB)
        float ax = rawX * 0.000244;
        float ay = rawY * 0.000244;
        float az = rawZ * 0.000244;

        accelMag[i] = sqrt(ax * ax + ay * ay + az * az);
    }
    Serial.println("done");

    // Compute metrics
    uint16_t rms = computeRMS(accelMag, SAMPLES_PER_WINDOW);
    uint16_t variance = computeVariance(accelMag, SAMPLES_PER_WINDOW);
    uint16_t peak = computePeak(accelMag, SAMPLES_PER_WINDOW);

    // Read temperature
    float tempF = myIMU.readTempF();
    uint16_t tempF_x10 = (uint16_t)(tempF * 10);

    // Store
    if (currentSample < MAX_SAMPLES)
    {
        dataLog[currentSample].rms_mg = rms;
        dataLog[currentSample].variance_mg2 = variance;
        dataLog[currentSample].peak_mg = peak;
        dataLog[currentSample].temp_f_x10 = tempF_x10;
        currentSample++;
    }

    // Print
    Serial.print("Sample ");
    Serial.print(currentSample);
    Serial.print(": RMS=");
    Serial.print(rms);
    Serial.print(" Var=");
    Serial.print(variance);
    Serial.print(" Peak=");
    Serial.print(peak);
    Serial.print(" Temp=");
    Serial.print(tempF_x10 / 10);
    Serial.print(".");
    Serial.print(tempF_x10 % 10);
    Serial.println("°F");
    Serial.println();
}