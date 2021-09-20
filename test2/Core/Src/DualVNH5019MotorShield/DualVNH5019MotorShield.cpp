#include "DualVNH5019MotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

DualVNH5019MotorShield::DualVNH5019MotorShield()
{
	_IN1A.pin = D4_Pin;
	_IN1A.port = D4_GPIO_Port;
	_IN1B.pin = D2_Pin;
	_IN1B.port = D2_GPIO_Port;
	_IN2A.pin = D7_Pin;
	_IN2A.port = D7_GPIO_Port;
	_IN2B.pin = D8_Pin;
	_IN2B.port = D8_GPIO_Port;



	_PWM1 = &TIM3->CCR2; // TODO remove dead code
	_PWM2 = &TIM4->CCR1; //

	//TODO initialize adc read parameters
	/*

	_FAULT_AND_ENABLE_1.pin = D6_Pin;
	_FAULT_AND_ENABLE_1.port = D6_GPIO_Port; // needs pull up to enable the device (already done by the cpu config)
	_FAULT_AND_ENABLE_2.pin = D12_Pin;       // device will pull this down when fault occurs
	_FAULT_AND_ENABLE_2.port = D12_GPIO_Port;

*/
	_CURRENT_SENSE_CHANNEL_1 = 0;// current sense use an analog read to read the current
	_CURRENT_SENSE_CHANNEL_2 = 1;


	_MOTOR_ADC = get_adc();

}


// Public Methods //////////////////////////////////////////////////////////////
void DualVNH5019MotorShield::init()
{
	// Define pinMode for the pins and set the frequency for timer1.

	// not used on stm32 because gpio is initialized in main
	/*
	pinMode(_INA1,OUTPUT);
	pinMode(_INB1,OUTPUT);
	pinMode(_PWM1,OUTPUT);
	pinMode(_FAULT_AND_ENABLE_1,INPUT);
	pinMode(_CURRENT_SENSE_1,INPUT);
	pinMode(_INA2,OUTPUT);
	pinMode(_INB2,OUTPUT);
	pinMode(_PWM2,OUTPUT);
	pinMode(_FAULT_AND_ENABLE_2,INPUT);
	pinMode(_CURRENT_SENSE_2,INPUT);
*/

}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DualVNH5019MotorShield::setM1Speed(int speed)
{
	unsigned char reverse = 0;

	if (speed < 0)
	{
		speed = -speed;  // Make speed a positive quantity
		reverse = true;  // Reverse the direction
	}
	if (speed > 400)  // Max PWM dutycycle
		speed = 400;


	//*_PWM1 = (int)((float)speed * PWM_MULT_FACTOR);
	set_pwm_m1((int)((float)speed * PWM_MULT_FACTOR));
	//analogWrite(_PWM1,speed * 51 / 80); // map 400 to 255


	if (speed == 0)
	{
		HAL_GPIO_WritePin(_IN1A.port, _IN1A.pin, GPIO_PIN_RESET); // Make the motor coast no matter
		HAL_GPIO_WritePin(_IN1B.port, _IN1B.pin, GPIO_PIN_RESET); // which direction it is spinning.
	}
	else if (reverse)
	{
		HAL_GPIO_WritePin(_IN1A.port, _IN1A.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_IN1B.port, _IN1B.pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_IN1A.port, _IN1A.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(_IN1B.port, _IN1B.pin, GPIO_PIN_RESET);
	}
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void DualVNH5019MotorShield::setM2Speed(int speed)
{
	unsigned char reverse = 0;

	if (speed < 0)
	{
		speed = -speed;  // make speed a positive quantity
		reverse = true;  // Reverse the direction
	}
	if (speed > 400)  // Max;
		speed = 400;

	// Sets pwm register to the new value
	//*_PWM2 = (int)((float)speed * PWM_MULT_FACTOR);
	set_pwm_m2((int)((float)speed * PWM_MULT_FACTOR));

	if (speed == 0)
	{
		HAL_GPIO_WritePin(_IN2A.port, _IN2A.pin, GPIO_PIN_RESET); // Make the motor coast no matter
		HAL_GPIO_WritePin(_IN2B.port, _IN2B.pin, GPIO_PIN_RESET); // which direction it is spinning.
	}
	else if (reverse)
	{
		HAL_GPIO_WritePin(_IN2A.port, _IN2A.pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(_IN2B.port, _IN2B.pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_IN2A.port, _IN2A.pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(_IN2B.port, _IN2B.pin, GPIO_PIN_RESET);
	}
}

// Set speed for motor 1 and 2
void DualVNH5019MotorShield::setSpeeds(int m1Speed, int m2Speed)
{
	setM1Speed(m1Speed);
	setM2Speed(m2Speed);
}

// Brake motor 1, brake is a number between 0 and 400
void DualVNH5019MotorShield::setM1Brake(int brake)
{
	// normalize brake
	if (brake < 0)
	{
		brake = -brake;
	}
	if (brake > 400)  // Max brake
		brake = 400;
	HAL_GPIO_WritePin(_IN1A.port, _IN1A.pin, GPIO_PIN_RESET); // Make the motor coast no
	HAL_GPIO_WritePin(_IN1B.port, _IN1B.pin, GPIO_PIN_RESET); // matter which direction it is spinning.


	//analogWrite(_PWM1,brake * 51 / 80); // map 400 to 255
}

// Brake motor 2, brake is a number between 0 and 400
void DualVNH5019MotorShield::setM2Brake(int brake)
{
	// normalize brake
	if (brake < 0)
	{
		brake = -brake;
	}
	if (brake > 400)  // Max brake
		brake = 400;
	HAL_GPIO_WritePin(_IN2A.port, _IN2A.pin, GPIO_PIN_RESET); // Make the motor coast no
	HAL_GPIO_WritePin(_IN2B.port, _IN2B.pin, GPIO_PIN_RESET); // matter which direction it is spinning.

	//analogWrite(_PWM2,brake * 51 / 80); // map 400 to 255

}

// Brake motor 1 and 2, brake is a number between 0 and 400
void DualVNH5019MotorShield::setBrakes(int m1Brake, int m2Brake)
{
	setM1Brake(m1Brake);
	setM2Brake(m2Brake);
}

// Return motor 1 current value in milliamps.
unsigned int DualVNH5019MotorShield::getM1CurrentMilliamps()
{
	// 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
	HAL_ADC_Start(&_MOTOR_ADC);
	HAL_ADC_PollForConversion(&_MOTOR_ADC, _CURRENT_SENSE_CHANNEL_1);
	return HAL_ADC_GetValue(&_MOTOR_ADC) * 34;
}

// Return motor 2 current value in milliamps.
unsigned int DualVNH5019MotorShield::getM2CurrentMilliamps()
{
	// 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
	HAL_ADC_Start(&_MOTOR_ADC);
	HAL_ADC_PollForConversion(&_MOTOR_ADC, _CURRENT_SENSE_CHANNEL_2);
	return HAL_ADC_GetValue(&_MOTOR_ADC) * 34;
}

// Return error status for motor 1
unsigned char DualVNH5019MotorShield::getM1Fault()
{
	return !HAL_GPIO_ReadPin(_FAULT_AND_ENABLE_1.port, _FAULT_AND_ENABLE_1.pin);
}

// Return error status for motor 2
unsigned char DualVNH5019MotorShield::getM2Fault()
{
	return !HAL_GPIO_ReadPin(_FAULT_AND_ENABLE_2.port, _FAULT_AND_ENABLE_2.pin);
}
