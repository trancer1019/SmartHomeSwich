#ifndef DEBOUNCER_H_
#define DEBOUNCER_H_

#define DEBOUNCE_DELAY 50

#include "stm32f0xx_hal.h"
#include "stdbool.h"

class Debouncer {
public:
	Debouncer(GPIO_TypeDef *port, uint16_t pin);
	uint8_t getState(bool repeat);
	void updateState();

private:
	GPIO_TypeDef *GPIO_PORT;
	uint16_t GPIO_PIN;

	bool firstStartFlag; //фалг установленный с момента запуска и до первого запроса

	bool buttonState;
	uint8_t counter; //счетчик задержки антидребезга

	uint8_t switchings; //количество переключений
	uint8_t switchings_backup; //количество переключений с прошлого раза
};

#endif /* DEBOUNCER_H_ */
