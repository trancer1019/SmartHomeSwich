#ifndef SIMPLEFIFO_H_
#define SIMPLEFIFO_H_

#include "stm32f0xx_hal.h"

// Размер очереди
#define bufferSize 256

class SimpleFIFO {
private:
  uint8_t queue[bufferSize];    // Массив для хранения элементов
  int front;             // Индекс начала очереди
  int rear;              // Индекс конца очереди
  int itemCount;         // Текущее количество элементов в очереди

public:
  // Конструктор класса
  SimpleFIFO();

  // Функция для проверки, является ли очередь полной
  bool isFull();

  // Функция для проверки, является ли очередь пустой
  bool isEmpty();

  // Функция для добавления элемента в очередь
  void enqueue(uint8_t item);

  // Функция для извлечения элемента из очереди
  uint8_t dequeue();

  // Функция для получения текущего количества элементов в очереди
  int getItemCount();
};

#endif /* SIMPLEFIFO_H_ */
