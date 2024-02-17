#include "Debouncer.h"

Debouncer::Debouncer(GPIO_TypeDef *port, uint16_t pin) {
	GPIO_PORT = port;
	GPIO_PIN = pin;

	buttonState = HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN) == GPIO_PIN_SET;
	firstStartFlag = true; //флаг "только загрузились"
	counter = 0;

	switchings = 0;
	switchings_backup = 0;
}

void Debouncer::updateState() {
	bool currentState = HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN) == GPIO_PIN_SET;

	//-если состояние кнопки изменилось
	if (currentState != buttonState) {
		counter++; //добавляем значения к счетчику ожиданий антидребезга

		if (counter > 4) {
			buttonState = currentState; //обновляем состояние кнопки
			switchings++; //добавляем количество переключений
			counter = 0; //сбрасываем счетчик ожидания антидребезга
		}
	} else {
		counter = 0; //сбрасываем счетчик ожидания антидребезга
	}
}

uint8_t Debouncer::getState(bool repeat) {

	if(repeat) {
		switchings_backup += switchings;
		switchings = 0; //сбрасываем количество переключений
	}
	else {
		switchings_backup = switchings;
		switchings = 0; //сбрасываем количество переключений
	}

	uint8_t result = (((uint8_t) firstStartFlag) << 7)
				| (((switchings_backup <= 0x0F) ? switchings_backup : 0x0F) << 1)
				| (buttonState & 0x01);

	firstStartFlag = false; //флаг "только загрузились"
	return result;
}
