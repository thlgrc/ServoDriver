#include "SysModel_PWMServoDriver.h"
#include <cstdint>
#include <time.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdio>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <chrono>


SysModel_PWMServoDriver::SysModel_PWMServoDriver()
    : _i2caddr(PCA9685_I2C_ADDRESS) {

	    begin();
	    setFreq(50); // default frequency
    }
   

void delay(int milliseconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * milliseconds;

    // Storing start time
    clock_t start_time = clock();

    // looping until required time is not achieved
    while (clock() < start_time + milli_seconds);
}

void SysModel_PWMServoDriver::sleep() {
    uint8_t awake = readReg(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
    writeReg(PCA9685_MODE1, sleep);
}

void SysModel_PWMServoDriver::wakeup() {
    uint8_t sleep = readReg(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    writeReg(PCA9685_MODE1, wakeup);
}

void SysModel_PWMServoDriver::setFreq(float freq){

    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
    freq = 1;
    if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = (( FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN){
    prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX){
    prescaleval = PCA9685_PRESCALE_MAX;
    }
    /*Update the prescale register*/
    uint8_t prescale = (uint8_t)prescaleval;
	sleep();
    writeReg(PCA9685_PRESCALE, prescale); // set the prescaler
    wakeup();

   // std::cout << "Prescale Value: 0x" << std::hex << (int)prescale << std::endl;
    _freq = freq;
    
}

void SysModel_PWMServoDriver::begin(){
    //initialize all LED registers to 0
    writeReg(PCA9685_ALLLED_ON_L,0);
    writeReg(PCA9685_ALLLED_ON_H,0);
    writeReg(PCA9685_ALLLED_OFF_L,0);
    writeReg(PCA9685_ALLLED_OFF_H,0);

}

void SysModel_PWMServoDriver::setAngle(int servoNum, int angle){
    float dutyCycle;
    float freqMicro = ((float)1/_freq)*1000000; // microseconds
    // printf("freqMicro: %f\n",freqMicro);    
    float pulseLength;
    int regValue;
    long pulseMicro = map(angle,SERVO_ANGLE_MIN,SERVO_ANGLE_MAX,SERVO_PULSE_MIN,SERVO_PULSE_MAX); //in microseconds
    dutyCycle = pulseMicro/freqMicro;
    pulseLength = (dutyCycle * 4096) - 1;
    regValue = (int)round(pulseLength);


    uint8_t buffer[4];
    buffer[0] = PCA9685_LED0_OFF_L + 4 * servoNum;
    buffer[1] = regValue & 0xFF; // to get the 8LSB
    buffer[2] = PCA9685_LED0_OFF_H + 4 * servoNum;
    buffer[3] = regValue >> 8; // to get the 4 MSB

    _servonum = servoNum;
    writeReg(buffer[0],buffer[1]);
    writeReg(buffer[2],buffer[3]);


}

/*Read/Write to Registers*/

void SysModel_PWMServoDriver::writeReg(uint8_t regNum,uint8_t value){
    int file;
    char *filename = (char*)I2C_BUS;
    uint8_t data[2];
    data[0] = regNum;
    data[1] = value;

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    if (write(file, &data, 2) == 0) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }


    close(file);

}

uint8_t SysModel_PWMServoDriver::readReg(uint8_t regNum){
    int file;
    char *filename = (char*)I2C_BUS;
    uint8_t data;
 
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }
 
    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    if (write(file, &regNum, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
 
    if (read(file, &data, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }
 
    //printf("Data read from register %x: 0x%x\n",regNum, data);
 
    close(file);
    
    return data;

}

long SysModel_PWMServoDriver::map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(){
	SysModel_PWMServoDriver servo;
	servo.begin();
	int count = 0;
	
	double averageSleep = 0;
	double sleepSum = 0;
	
	double averageWakeup = 0;
	double wakeupSum = 0;
	
	double averageSetFreq = 0;
	double setFreqSum = 0;
	
	double averageBegin = 0;
	double beginSum = 0;
	
	double averageSetAngle = 0;
	double setAngleSum = 0;
	
	double averageWriteReg = 0;
	double writeRegSum = 0;
	
	double averageReadReg = 0;
	double readRegSum = 0;
	
	double averageMap = 0;
	double mapSum = 0;
	
	double limit = 1000.0;
	while(count <= (int) limit){
		auto start_time = std::chrono::high_resolution_clock::now();
		servo.sleep(); //uses writeReg, readReg
		auto end_time = std::chrono::high_resolution_clock::now(); 
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		sleepSum += duration.count();  
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.wakeup(); //uses writeReg, readReg 
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		wakeupSum += duration.count();  
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.setFreq(50); //uses sleep, writeReg, wakeup
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		setFreqSum += duration.count();  
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.begin(); //uses writeReg
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		beginSum += duration.count();   
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.setAngle(0,155); //uses writeReg, map
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		setAngleSum += duration.count();  
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.writeReg(8,66); //uses linux filesystem
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		writeRegSum += duration.count();  
		
		start_time = std::chrono::high_resolution_clock::now();
		servo.readReg(8);
		end_time = std::chrono::high_resolution_clock::now(); 
		duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		readRegSum += duration.count();  
		
		count++;
	
	}
	
	averageSleep = sleepSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean sleep() Execution time: " << averageSleep << " microseconds" << std::endl;
	
	
	averageWakeup= wakeupSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean wakeup() Execution time: " << averageWakeup << " microseconds" << std::endl;
	
	
	averageSetFreq= setFreqSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean setFreq() Execution time: " << averageSetFreq << " microseconds" << std::endl;
	
	
	averageBegin = beginSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean begin() Execution time: " << averageBegin << " microseconds" << std::endl;
	
	
	averageSetAngle = setAngleSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean setAngle() Execution time: " << setAngleSum << " microseconds" << std::endl;
	
	
	averageWriteReg = writeRegSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean writeReg() Execution time: " << averageWriteReg << " microseconds" << std::endl;
	
	
	averageReadReg = writeRegSum/limit;
	std::cout << "_________________________________________________________________" << std::endl;
	
	std::cout << "Mean readReg() Execution time: " << averageReadReg << " microseconds" << std::endl;
	
	
	std::cout << "Test iterations: " << limit <<  std::endl;
	
	
	return EXIT_SUCCESS;
	
	return 0;
}

