#ifndef SHUARTCONTROLLER_H_
#define SHUARTCONTROLLER_H_

#include "stm32f0xx_hal.h"
#include "stdbool.h"

class SH_UartController {
public:
	SH_UartController(uint8_t DeviceAdress, UART_HandleTypeDef *huart); // Конструктор

	void update(uint8_t incomingByte); //Функция для обновления состояния в основном loop

private:
	const uint8_t DeviceAdress;  //адрес устройства
	UART_HandleTypeDef *huart;

	uint8_t calculateCRC4(const uint8_t *data, size_t length);

	bool checkCRC(uint8_t CRC4, uint8_t NCOM, uint8_t registerNumber,
			uint8_t registerValue);
	bool checkCRC(uint8_t CRC4, uint8_t NCOM, uint8_t registerNumber);
	bool checkCRC(uint8_t CRC4, uint8_t NCOM);

	void rs485Send(const uint8_t *buffer, size_t size);

	void sendRegstate(uint8_t registerNumber, uint8_t registerValue);
	void send4Regstate(uint8_t* registerNumbersAndValues);
	void sendConfirm(uint8_t registerNumber);

	void writeRegister(uint8_t registerNumber, uint8_t registerValue);
	void readRegister(uint8_t registerNumber);
	void read4Register();
};

#endif /* SHUARTCONTROLLER_H_ */
