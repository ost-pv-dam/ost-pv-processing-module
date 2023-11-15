//
// Created by Joseph Maffetone on 11/14/23.
//

#ifndef OST_PV_PROCESSING_MODULE_UART_CIRCULAR_BUFFER_HPP
#define OST_PV_PROCESSING_MODULE_UART_CIRCULAR_BUFFER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <string>

constexpr std::size_t UART_BUFFER_SIZE = 512;

struct UartCircularBufferRange {
    std::size_t begin;
    std::size_t end;

    UartCircularBufferRange() = default;
    UartCircularBufferRange(std::size_t begin, std::size_t end) : begin{begin}, end{end} {}
};

class UartCircularBuffer {
public:
    UartCircularBuffer() = default;

    void push(uint8_t el) {
        data[tail] = el;
        ++tail;
        if (tail >= data.size()) {
            tail = 0;
        }
    }

    uint8_t* get_tail() {
        return &data[tail];
    }

    char back() {
        if
        return static_cast<char>()
    }

    std::optional<UartCircularBufferRange> check_tail_string(std::string& str) {
        if (head <= tail) { // no wrap
            if (tail - head < str.size()) return std::nullopt;

            if (std::equal(data.begin()+tail-str.size(), data.begin()+tail, str.begin(), str.end())) {
                head = (tail >= data.size()-1) ? 0 : tail + 1;
                tail = head;
                return std::make_optional<UartCircularBufferRange>(head, tail);
            } else {
                return std::nullopt;
            }
        } else { // we've wrapped around
            std::size_t idx = tail;
            for (long long i = str.size(); i >= 0; i--) {
                if (str[i] != static_cast<char>(data[idx])) {
                    return std::nullopt;
                }

                if (idx == 0) {
                    idx = data.size()-1;
                } else {
                    --idx;
                }
            }

            head = (tail >= data.size()-1) ? 0 : tail + 1;
            tail = head;
            return std::make_optional<UartCircularBufferRange>(head, tail);
        }
    }

    std::string get_string(UartCircularBufferRange& range) {
        if (range.begin <= range.end) {
            return {(char*) &data[range.begin], range.end + 1 - range.begin};
        } else {
            std::string res{(char*) &data[range.begin],data.size() - range.begin};
            res += std::string{(char*) &data[0], range.end+1};

            return res;
        }
    }

private:
    std::array<uint8_t, UART_BUFFER_SIZE> data{};
    std::size_t head = 0;
    std::size_t tail = 0;
};


#endif //OST_PV_PROCESSING_MODULE_UART_CIRCULAR_BUFFER_HPP
