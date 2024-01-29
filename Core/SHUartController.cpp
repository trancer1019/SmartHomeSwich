#include "SHUartController.h"

uint8_t in_packet_buffer_position = 0;  //текущая позиция в буфере приема
uint8_t in_packetCRC;                   //CRC принимаемого покета
uint8_t in_packetNCOM;                  //N команды принимаемого покета
uint8_t in_packetRegnum;                //Регистр принимаемого покета

// Реализация конструктора
SH_UartController::SH_UartController(uint8_t *DeviceAdress,
		UART_HandleTypeDef *huart) :
		DeviceAdress(DeviceAdress), huart(huart) {
}

//назначить состояние
void SH_UartController::update(uint8_t incomingByte) {
	//-если байт начала покета
	if (incomingByte == 0xAA) {
		in_packet_buffer_position = 1;  //переключаем позицию
	}
	//-позиция байта адреса устройства
	else if (in_packet_buffer_position == 1) {
		if (incomingByte == *DeviceAdress) {  //-если адрес текущего устройства
			in_packet_buffer_position = 2;  //переключаем позицию
		} else {
			in_packet_buffer_position = -1;  //сбрасываем позицию
		}
	}
	//-позиция байта N_команды + CRC
	else if (in_packet_buffer_position == 2) {
		in_packetNCOM = incomingByte & 0x0F;  //считываем номер команды
		in_packetCRC = incomingByte >> 4;     //считываем CRC

		if (in_packetNCOM == 1 || in_packetNCOM == 2) { //если номер команды Чтение / Запись / Чтение всего
			in_packet_buffer_position = 3;  //переключаем позицию
		} else if (in_packetNCOM == 5) { //если номер команды Чтение всего
			if (checkCRC(in_packetCRC, in_packetNCOM)) { //проверка CRC
				read4Register(); //запуск команды чтения всех регистров
			}
			in_packet_buffer_position = -1;  //сбрасываем позицию
		} else {
			in_packet_buffer_position = -1;  //сбрасываем позицию
		}
	}
	//-позиция байта Номер регистра
	else if (in_packet_buffer_position == 3) {
		if (in_packetNCOM == 1) { //команда чтения
			if (checkCRC(in_packetCRC, in_packetNCOM, incomingByte)) { //проверка CRC
				readRegister(incomingByte);  //запуск команды чтения регистра
			}
			in_packet_buffer_position = -1;  //сбрасываем позицию
		} else if (in_packetNCOM == 2) { //команда записи
			in_packetRegnum = incomingByte;  //сохраняем номер регистра
			in_packet_buffer_position = 4;   //переключаем позицию
		} else {
			in_packet_buffer_position = -1;  //сбрасываем позицию
		}
	}
	//-позиция байта Данные регистра
	else if (in_packet_buffer_position == 4) {
		if (checkCRC(in_packetCRC, in_packetNCOM, in_packetRegnum,
				incomingByte)) { //проверка CRC
			writeRegister(in_packetRegnum, incomingByte); //запуск команды записи регистра
		}
		in_packet_buffer_position = -1;  //сбрасываем позицию
	}
}

///-------------------------------------------------------------
// функция вычисления CRC4
uint8_t SH_UartController::calculateCRC4(const uint8_t *data, size_t length) {
	uint8_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];  // выполняем XOR с каждым байтом данных

		for (int j = 0; j < 4; ++j) {
			if (crc & 0x8) {
				crc = (crc << 1) ^ 0x3;  // полином x^4 + x^1 + 1
			} else {
				crc <<= 1;
			}
		}
	}

	return crc & 0xF;  // возвращаем младшие 4 бита CRC
}
//-функция проверки CRC
bool SH_UartController::checkCRC(uint8_t CRC4, uint8_t NCOM,
		uint8_t registerNumber, uint8_t registerValue) {
	uint8_t data[] = { *DeviceAdress, NCOM, registerNumber, registerValue };
	size_t dataLength = sizeof(data) / sizeof(data[0]);

	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	// Сравнение рассчитанной CRC с переданной CRC
	return (CRC4 == crcResult);
}
//-функция проверки CRC
bool SH_UartController::checkCRC(uint8_t CRC4, uint8_t NCOM,
		uint8_t registerNumber) {
	uint8_t data[] = { *DeviceAdress, NCOM, registerNumber };
	size_t dataLength = sizeof(data) / sizeof(data[0]);

	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	// Сравнение рассчитанной CRC с переданной CRC
	return (CRC4 == crcResult);
}
//-функция проверки CRC
bool SH_UartController::checkCRC(uint8_t CRC4, uint8_t NCOM) {
	uint8_t data[] = { *DeviceAdress, NCOM };
	size_t dataLength = sizeof(data) / sizeof(data[0]);

	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	// Сравнение рассчитанной CRC с переданной CRC
	return (CRC4 == crcResult);
}

///-------------------------------------------------------------
//-функция для отправки данных по rs485
void SH_UartController::rs485Send(const uint8_t *buffer, size_t size) {
	HAL_UART_Transmit_IT(huart, (uint8_t*) buffer, size);
}

//функция для отправки состояния запрашиваемого регистра
void SH_UartController::sendRegstate(uint8_t registerNumber,
		uint8_t registerValue) {
	if (registerValue == 0xAA)
		registerValue = registerValue + 1; //если значение регистра = 0xAA

	uint8_t data[] = { *DeviceAdress, 0x03, registerNumber, registerValue };
	size_t dataLength = sizeof(data) / sizeof(data[0]);
	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	uint8_t data_send[] = { 0xAA, *DeviceAdress,
			(uint8_t) (crcResult << 4 | 0x03), registerNumber, registerValue };
	size_t data_sendLength = sizeof(data_send) / sizeof(data_send[0]);
	rs485Send(data_send, data_sendLength);
}

//функция для отправки состояния запрашиваемого регистра
void SH_UartController::send4Regstate(uint8_t *registerNumbersAndValues) {
	uint8_t data[] = { *DeviceAdress, 0x06,
			registerNumbersAndValues[0] != 0xAA ?
					registerNumbersAndValues[0] :
					(uint8_t)(registerNumbersAndValues[0] + 1),
			registerNumbersAndValues[1] != (uint8_t)0xAA ?
					registerNumbersAndValues[1] :
					(uint8_t)(registerNumbersAndValues[1] + 1),
			registerNumbersAndValues[2] != (uint8_t)0xAA ?
					registerNumbersAndValues[2] :
					(uint8_t)(registerNumbersAndValues[2] + 1),
			registerNumbersAndValues[3] != (uint8_t)0xAA ?
					registerNumbersAndValues[3] :
					(uint8_t)(registerNumbersAndValues[3] + 1),
			registerNumbersAndValues[4] != (uint8_t)0xAA ?
					registerNumbersAndValues[4] :
					(uint8_t)(registerNumbersAndValues[4] + 1),
			registerNumbersAndValues[5] != (uint8_t)0xAA ?
					registerNumbersAndValues[5] :
					(uint8_t)(registerNumbersAndValues[5] + 1),
			registerNumbersAndValues[6] != (uint8_t)0xAA ?
					registerNumbersAndValues[6] :
					(uint8_t)(registerNumbersAndValues[6] + 1),
			registerNumbersAndValues[7] != (uint8_t)0xAA ?
					registerNumbersAndValues[7] :
					(uint8_t)(registerNumbersAndValues[7] + 1) };

	size_t dataLength = sizeof(data) / sizeof(data[0]);
	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	uint8_t data_send[] = { 0xAA, *DeviceAdress,
			(uint8_t) (crcResult << 4 | 0x06), data[2], data[3], data[4],
			data[5], data[6], data[7], data[8], data[9] };
	size_t data_sendLength = sizeof(data_send) / sizeof(data_send[0]);
	rs485Send(data_send, data_sendLength);
}

//функция для отправки подтвержения записи регистра
void SH_UartController::sendConfirm(uint8_t registerNumber) {
	uint8_t data[] = { *DeviceAdress, 0x04, registerNumber };
	size_t dataLength = sizeof(data) / sizeof(data[0]);
	// Вычисление CRC4
	uint8_t crcResult = calculateCRC4(data, dataLength);

	uint8_t data_send[] = { 0xAA, *DeviceAdress,
			(uint8_t) (crcResult << 4 | 0x04), registerNumber };
	size_t data_sendLength = sizeof(data_send) / sizeof(data_send[0]);
	rs485Send(data_send, data_sendLength);
}

extern uint8_t deviceReadRegister(uint8_t registerNumber); //обявленеи внешней функции
//функция для чтения регистра и отправки ответа
void SH_UartController::readRegister(uint8_t registerNumber) {
	uint8_t registerValue = deviceReadRegister(registerNumber); //выполняем команду
	sendRegstate(registerNumber, registerValue); //отправляем состояние регистра
}

extern uint8_t* deviceRead4Register(); //обявленеи внешней функции
//функция для чтения всех регистров и отправки ответа
void SH_UartController::read4Register() {
	uint8_t *registerNumbersAndValues = deviceRead4Register(); //выполняем команду
	send4Regstate(registerNumbersAndValues); //отправляем состояние регистра
	delete[] registerNumbersAndValues;
}

extern void deviceWriteRegister(uint8_t registerNumber, uint8_t registerValue); //обявленеи внешней функции
//функция для записи значения в регистр и отправки ответа
void SH_UartController::writeRegister(uint8_t registerNumber,
		uint8_t registerValue) {
	deviceWriteRegister(registerNumber, registerValue);  //выполняем команду
	sendConfirm(registerNumber);  //отправлем подтверждение приема данных
}
