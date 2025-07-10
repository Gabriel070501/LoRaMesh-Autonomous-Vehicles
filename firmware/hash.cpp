#include <iostream>
#include <cstdint>

uint32_t fnv1a_hash(uint8_t* data, size_t len) {
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < len; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

int main() {
    // Simulando analogRead dos pinos 5, 6, 7
    uint16_t analog5 = 1234;
    uint16_t analog6 = 2048;
    uint16_t analog7 = 3777;

    // Empacota os valores em um array de bytes
    uint8_t buf[6];
    buf[0] = analog5 >> 8;
    buf[1] = analog5 & 0xFF;
    buf[2] = analog6 >> 8;
    buf[3] = analog6 & 0xFF;
    buf[4] = analog7 >> 8;
    buf[5] = analog7 & 0xFF;

    uint32_t hash = fnv1a_hash(buf, 6);

    std::cout << "Hash gerado: " << hash << std:\.:endl;
    return 0;
}