#include "Debouncer.h"

Debouncer::Debouncer(GPIO_TypeDef *port, uint16_t pin) {
	GPIO_PORT = port;
	GPIO_PIN = pin;

	buttonState = HAL_GPIO_ReadPin(GPIO_PORT, GPIO_PIN) == GPIO_PIN_SET;
	counter = 0;

	switchings = 0;
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

uint8_t Debouncer::getState() {
	uint8_t result = (((switchings <= 0x3F) ? switchings : 0x3F) << 1)
			| (buttonState & 0x01);
	switchings = 0; //сбрасываем количество переключений
	return result;
}
