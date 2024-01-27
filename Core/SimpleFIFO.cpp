#include "SimpleFIFO.h"

// Конструктор класса
SimpleFIFO::SimpleFIFO() : front(0), rear(-1), itemCount(0) {}

// Функция для проверки, является ли очередь полной
bool SimpleFIFO::isFull() {
  return itemCount == bufferSize;
}

// Функция для проверки, является ли очередь пустой
bool SimpleFIFO::isEmpty() {
  return itemCount == 0;
}

// Функция для добавления элемента в очередь
void SimpleFIFO::enqueue(uint8_t item) {
  if (!isFull()) {
    if (rear == bufferSize - 1) {
      rear = -1; // Циклический переход, если достигнут конец массива
    }
    queue[++rear] = item;
    itemCount++;
  }
}

// Функция для извлечения элемента из очереди
uint8_t SimpleFIFO::dequeue() {
  if (!isEmpty()) {
    uint8_t item = queue[front++];
    if (front == bufferSize) {
      front = 0; // Циклический переход, если достигнут конец массива
    }
    itemCount--;
    return item;
  } else {
    return 0; // Возвращаем нулевой элемент, если очередь пуста
  }
}

// Функция для получения текущего количества элементов в очереди
int SimpleFIFO::getItemCount() {
  return itemCount;
}
