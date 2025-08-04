#ifndef _DYNAMIC_WINDOW_H
#define _DYNAMIC_WINDOW_H

#include <iostream>
#include <vector>
#include <array>
#include <stdexcept>

class DynamicWindow {
public:
    // 构造函数，设置最大容量
    explicit DynamicWindow(size_t maxCapacity)
        : capacity(maxCapacity), head(0), currentSize(0) {
        if (capacity == 0) {
            throw std::invalid_argument("Capacity must be greater than zero");
        }
        buffer.resize(capacity);
    }

    // 添加新元素
    void push(const std::array<double, 2>& element) {
        if (capacity == 0) return;
        
        if (currentSize < capacity) {
            buffer[(head + currentSize) % capacity] = element;
            ++currentSize;
        } else {
            buffer[head] = element;
            head = (head + 1) % capacity;
        }
    }

    // 随机访问特定位置元素
    std::array<double, 2> at(size_t index) const {
        if (index >= currentSize) {
            throw std::out_of_range("Index out of range");
        }
        return buffer[(head + index) % capacity];
    }

    // 计算所有元素的平均值
    std::array<double, 2> mean() const {
        if (currentSize == 0) {
            return {0.0, 0.0};
        }
        
        std::array<double, 2> sum = {0.0, 0.0};
        for (size_t i = 0; i < currentSize; ++i) {
            const auto& element = buffer[(head + i) % capacity];
            sum[0] += element[0];
            sum[1] += element[1];
        }
        return {sum[0] / currentSize, sum[1] / currentSize};
    }

    // 获取当前元素数量
    size_t size() const {
        return currentSize;
    }

    // 获取最大容量
    size_t maxCapacity() const {
        return capacity;
    }

    // 清空窗口
    void clear() {
        head = 0;
        currentSize = 0;
    }
private:
    std::vector<std::array<double, 2>> buffer; // 存储元素的缓冲区
    size_t capacity;    // 最大容量
    size_t head;        // 指向最早元素的索引
    size_t currentSize; // 当前存储的元素数量
};// 实现方式类似于队列的顺序实现（循环队列数组）

#endif