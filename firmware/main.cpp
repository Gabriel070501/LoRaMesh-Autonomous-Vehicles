#ifndef LoRaMESH_h
#define LoRaMESH_h

#include <Stream.h>
#include <Arduino.h>

#define MAX_PAYLOAD_SIZE     232
#define MAX_BUFFER_SIZE      237

#define MIN_RANGE            400
#define MAX_RETX               2

#define PASSWORD_RAM_ADDRESS 0x20000000U

#define BW125                0x00
#define BW250                0x01
#define BW500                0x02

#define SF7                  0x07
#define SF8                  0x08
#define SF9                  0x09
#define SF10                 0x0A
#define SF11                 0x0B
#define SF12                 0x0C
#define SF_FSK               0x00

#define CR4_5                0x01
#define CR4_6                0x02
#define CR4_7                0x03
#define CR4_8                0x04

#define CLASS_A              0x00
#define CLASS_C              0x02

#define WINDOW_5s            0x00
#define WINDOW_10s           0x01
#define WINDOW_15s           0x02

#define GPIO_MODE_READ       0x00
#define GPIO_MODE_WRITE      0x01
#define GPIO_MODE_CONFIG     0x02

#define LoRa_NOT_PULL        0x00
#define LoRa_PULLUP          0x01
#define LoRa_PULLDOWN        0x02

#define INOUT_DIGITAL_INPUT  0x00
#define INOUT_DIGITAL_OUTPUT 0x01
#define INOUT_ANALOG_INPUT   0x03


#ifndef INPUT
#define INPUT                0
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP         7
#endif

#ifdef  INPUT_PULLDOWN_16
#define INPUT_PULLDOWN       INPUT_PULLDOWN_16
#endif

#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN       8
#endif

#define NUM_NORMAL_CHANNELS 64
#define NUM_EXTRA_CHANNELS 8
#define BASE_NORMAL_FREQ_HZ 902300000  // 902.3 MHz
#define BASE_EXTRA_FREQ_HZ 901500000   // 915.1 MHz
#define CHANNEL_SPACING_HZ 200000  

uint32_t normal_channels[3];
uint32_t extra_channel;

unsigned long last_channel_update = 0;
const unsigned long CHANNEL_UPDATE_INTERVAL = 5 * 60 * 1000UL; // 5 minutos em ms

uint8_t normal_channel_index = 0;

extern uint32_t _end;
extern uint32_t _estack;

static uint8_t* next_free = (uint8_t*)&_end;

enum ChannelType { NORMAL, EXTRA };
ChannelType currentChannelType = NORMAL;
int currentNormalChannel = 0;
int currentExtraChannel = 0;

struct SensorData {
    uint32_t altitude;
    uint32_t latitude;
    uint32_t longitude;
    uint32_t timestamp;
    uint32_t noise;
};

void loop() {
    update_channels_if_needed();
    updateDynamicId();
    sendPacket();  

    uint16_t recv_id;
    uint8_t recv_command;
    uint8_t recv_payload[MAX_PAYLOAD_SIZE];
    uint8_t recv_payloadSize;

    bool received = ChannelHop(&recv_id, &recv_command, recv_payload, &recv_payloadSize, 5000);
    if (received) {
        // Processa o pacote recebido
    }
}

class LoRaMESH{
    private:
        bool analogEnabled = false;

    uint32_t fnv1a_hash(const uint8_t* data, size_t len) {
        uint32_t hash = 2166136261UL;
        for (size_t i = 0; i < len; ++i) {
            hash ^= data[i];
            hash *= 16777619UL;
        }
        return hash;
    }

    uint32_t pick_random_normal_channel() {
        int channel = rand() % NUM_NORMAL_CHANNELS;
        return BASE_NORMAL_FREQ_HZ + (channel * CHANNEL_SPACING_HZ);
    }

    uint32_t pick_random_extra_channel() {
        int channel = rand() % NUM_EXTRA_CHANNELS;
        return BASE_EXTRA_FREQ_HZ + (channel * CHANNEL_SPACING_HZ);
    }

    void update_channels_if_needed() {
        unsigned long now = millis();
        if (now - last_channel_update > CHANNEL_UPDATE_INTERVAL || last_channel_update == 0) {
            for (int i = 0; i < 3; i++) {
                normal_channels[i] = pick_random_normal_channel();
            }
            extra_channel = pick_random_extra_channel();
            last_channel_update = now;
            normal_channel_index = 0;
        }
        else{
            return
        }
    }

    bool ChannelHop(uint16_t* id, uint8_t* command, uint8_t* payload, uint8_t* payloadSize, uint32_t totalTimeoutMs) {
        const uint32_t singleAttemptTimeout = 500;
        uint32_t startTime = millis();
    
        while (millis() - startTime < totalTimeoutMs) {
            uint32_t freq = 0;
    
            if (currentChannelType == NORMAL) {
                freq = BASE_NORMAL_FREQ_HZ + currentNormalChannel * CHANNEL_SPACING_HZ;
            } else { // EXTRA
                freq = BASE_EXTRA_FREQ_HZ + currentExtraChannel * CHANNEL_SPACING_HZ;
            }
    
            change_channel(freq);
    
            bool received = receivePacketCommand(id, command, payload, payloadSize, singleAttemptTimeout, freq);
            if (received) {
                return true;
            }
    
            if (currentChannelType == NORMAL) {
                currentNormalChannel++;
                if (currentNormalChannel >= NUM_NORMAL_CHANNELS) {
                    currentNormalChannel = 0;
                    currentChannelType = EXTRA;
                }
            } else {
                currentExtraChannel++;
                if (currentExtraChannel >= NUM_EXTRA_CHANNELS) {
                    currentExtraChannel = 0;
                    currentChannelType = NORMAL;
                }
            }
        }
    
        return false;
    }

    uint8_t* getPasswordFromRAM() {
        return (uint8_t*)PASSWORD_RAM_ADDRESS;
    }

    void AES_128(uint8_t* input, uint8_t* output, uint32_t tamanho) {
        struct AES_ctx ctx;
        uint8_t* chave = getPasswordFromRAM();
    
        AES_init_ctx(&ctx, chave);
    
        for (uint32_t i = 0; i < tamanho; i += 16) {
            AES_ECB_encrypt(&ctx, input + i);
            memcpy(output + i, input + i, 16);
        }

        return output;
    }

    void AES_128_decrypt(const uint8_t* input, uint8_t* output, uint32_t length){
        struct AES_ctx ctx;
        AES_init_ctx(&ctx, obter_senha_da_ram());
    
        for(uint32_t i = 0; i < length; i += 16){
            memcpy(output + i, input + i, 16);
            AES_ECB_decrypt(&ctx, output + i);
        }

        return output
    }

    uint16_t pick_random_window(uint16_t max_ms) {
        return (rand() % max_ms) + 1; // de 1 ms até max_ms
    }

    SensorData getData() {
        SensorData data;
    
        if (SerialData.available() < 16) {
            data.altitude  = 0;
            data.latitude  = 0;
            data.longitude = 0;
            data.timestamp = 0;
        } else {
            data.altitude  = ((uint32_t)SerialData.read())       |
                             ((uint32_t)SerialData.read() << 8 ) |
                             ((uint32_t)SerialData.read() << 16) |
                             ((uint32_t)SerialData.read() << 24);
    
            data.latitude  = ((uint32_t)SerialData.read())       |
                             ((uint32_t)SerialData.read() << 8 ) |
                             ((uint32_t)SerialData.read() << 16) |
                             ((uint32_t)SerialData.read() << 24);
    
            data.longitude = ((uint32_t)SerialData.read())       |
                             ((uint32_t)SerialData.read() << 8 ) |
                             ((uint32_t)SerialData.read() << 16) |
                             ((uint32_t)SerialData.read() << 24);
    
            data.timestamp = ((uint32_t)SerialData.read())       |
                             ((uint32_t)SerialData.read() << 8 ) |
                             ((uint32_t)SerialData.read() << 16) |
                             ((uint32_t)SerialData.read() << 24);
        }
    
        data.noise = analogRead(localId, 8);
    
        return data;
    }

    void updateDynamicId() {
        uint32_t currentTime = millis();
        
        if (currentTime - lastIdUpdateTime >= 300000UL) {
            lastIdUpdateTime = currentTime;
    
            uint32_t altitude, latitude, longitude, timestamp, noise;
            readSensorData(&altitude, &latitude, &longitude, &timestamp, &noise);
    
            uint8_t buffer[12];
            memcpy(&buffer[0], &latitude, 4);
            memcpy(&buffer[4], &altitude, 4);
            memcpy(&buffer[8], &noise,    4);
    
            id = fnv1a_hash(buffer, sizeof(buffer));
            localId = id
            if (debug_serial) {
                Serial.print("Novo ID gerado: ");
                Serial.println(dynamicId, HEX);
            }
        }
    }

    public:
        bool debug_serial = false;
        typedef struct
        {
            uint8_t buffer[MAX_BUFFER_SIZE];
            uint8_t size;
            bool command;
        } Frame_Typedef;

        uint8_t bufferPayload[MAX_PAYLOAD_SIZE] = {0};
        uint8_t payloadSize = 0;

        uint16_t localId = 0;
        uint32_t localUniqueId;
        uint8_t command;
        bool isMaster;

        Frame_Typedef frame;
        uint16_t deviceId = -1;
        uint16_t deviceNet = -1;
        uint32_t deviceUniqueId = -1;

        uint32_t registered_password;

        uint8_t BW, SF, CR, LoRa_class, LoRa_window;

        Stream*  SerialLoRa;
        Stream*  SerialLoRat;

        LoRaMESH(Stream *_SerialLoRa, Stream *_SerialLoRat = NULL){
            SerialLoRa = _SerialLoRa;
            SerialLoRat = _SerialLoRat;
        }

        uint16_t ComputeCRC(uint8_t* data_in, uint16_t length){
            uint16_t i;
            uint8_t bitbang, j;
            uint16_t crc_calc;

            crc_calc = 0xC181;
            for(i=0; i<length; i++)
            {
                crc_calc ^= (((uint16_t)data_in[i]) & 0x00FF);
                
                for(j=0; j<8; j++)
                {
                    bitbang = crc_calc;
                    crc_calc >>= 1;
                    
                    if(bitbang & 1)
                    {
                        crc_calc ^= 0xA001;
                    }
                }
            }
            return (crc_calc&0xFFFF);
        }

        bool prepareFrameCommand(uint16_t id, uint8_t command, uint8_t* , uint8_t Size){
            if((id < 0)) return false;
            if(command < 0) return false;
            if( < 0) return false;
            if(Size < 0) return false;
            

            uint16_t crc = 0;

            frame.size = Size + 5;
            
            frame.buffer[0] = id&0xFF;
            frame.buffer[1] = (id>>8)&0xFF;
            
            frame.buffer[2] = command;
            
            if((payloadSize >= 0) && (payloadSize < MAX_PAYLOAD_SIZE))
            {
                memcpy(&(frame.buffer[3]), payload, payloadSize);
            
                crc = ComputeCRC((&frame.buffer[0]), payloadSize+3);
                frame.buffer[payloadSize+3] = crc&0xFF;
                frame.buffer[payloadSize+4] = (crc>>8)&0xFF;
            }
            else
            {
                memset(&frame.buffer[0], 0, MAX_BUFFER_SIZE);
                return false;
            }

            frame.command = true;

            return sendPacket();
        }

        bool PrepareFrameTransp(uint16_t id, uint8_t* payload, uint8_t payloadSize) {
            uint8_t i = 0;
        
            if(payload == NULL) return false;
            if(deviceId == -1) return false;
            if(payloadSize < 19) return false;
        
            uint8_t retx_byte_index = 0;
            if (payloadSize == 23) {
                retx_byte_index = 20;
                uint8_t retx_bits = payload[retx_byte_index] & 0x07;
                if (retx_bits >= MAX_RETX) return false;
            } else if (payloadSize == 19) {
                retx_byte_index = 16;
                uint8_t retx_bits = payload[retx_byte_index] & 0x07;
                if (retx_bits >= MAX_RETX) return false;
            }
        
            int index = -1;
            for (int j = 0; j < 4; j++) {
                if (ids[j] == (uint8_t)id) {
                    index = j;
                    break;
                }
            }
        
            uint32_t payloadTimestamp = 0;
            if (payloadSize == 23) {
                payloadTimestamp = 
                    ((uint32_t)payload[16]) |
                    ((uint32_t)payload[17] << 8) |
                    ((uint32_t)payload[18] << 16) |
                    ((uint32_t)payload[19] << 24);
            } else if (payloadSize == 19) {
                payloadTimestamp = 
                    ((uint32_t)payload[12]) |
                    ((uint32_t)payload[13] << 8) |
                    ((uint32_t)payload[14] << 16) |
                    ((uint32_t)payload[15] << 24);
            }
        
            if (index != -1) {
                if (payloadTimestamp <= timestamps[index]) {
                    return false;
                }
                timestamps[index] = payloadTimestamp;
            } else {
                return true;
            }
        
            uint8_t current_byte = payload[retx_byte_index];
            uint8_t retx_bits = current_byte & 0x07;
            retx_bits = (retx_bits + 1) & 0x07;  // incrementa e mantém 3 bits
            current_byte &= 0xF8;                // limpa bits 0-2
            current_byte |= retx_bits;           // atualiza bits 0-2
            payload[retx_byte_index] = current_byte;

            savePayloadOnRAM(payload);
            
            frame.size = payloadSize + 2;
            frame.buffer[i++] = id & 0xFF;
            frame.buffer[i++] = (id >> 8) & 0x03;
        
            delay(1000);
        
            if (payloadSize >= 0 && payloadSize < MAX_PAYLOAD_SIZE) {
                memcpy(&frame.buffer[i], payload, payloadSize);
            } else {
                memset(&frame.buffer[0], 0, MAX_BUFFER_SIZE);
                return false;
            }
        
            uint32_t freq_hz = 0;
            uint16_t window_ms = 0;
        
            if (payloadSize == 23) {
                freq_hz = pick_random_extra_channel();
                window_ms = pick_random_window(10);  // até 10 ms
            } else if (payloadSize == 19) {
                freq_hz = pick_random_normal_channel();
                window_ms = pick_random_window(40);  // até 40 ms
            }
        
            change_channel(freq_hz);
        
            delay(window_ms);  // espera a janela antes de enviar
        
            uint32_t tamanho_cript = ((frame.size + 15) / 16) * 16;
            uint8_t buffer_cript[tamanho_cript];
            memset(buffer_cript, 0, tamanho_cript);
        
            AES_128(frame.buffer, buffer_cript, tamanho_cript);
        
            if(debug_serial) {
                Serial.print("TX (cript): ");
                printHex(buffer_cript, tamanho_cript);
            }
        
            if(frame.command)
                SerialLoRa->write(buffer_cript, tamanho_cript);
            else {
                if(SerialLoRat == NULL)
                    return false;
                else
                    SerialLoRat->write(buffer_cript, tamanho_cript);
            }
        
            return true;
        }

        void sendPacket() {
            static uint32_t lastTransmissionTime = 0;
            static uint8_t packetCounter = 0;
        
            uint32_t currentTime = millis();
        
            if (currentTime - lastTransmissionTime >= 1000) {
                lastTransmissionTime = currentTime;
        
                SensorData data = getData();
        
                uint8_t hashInput[12];
                memcpy(&hashInput[0], &data.latitude, 4);
                memcpy(&hashInput[4], &data.altitude, 4);
                memcpy(&hashInput[8], &data.noise, 4);
                uint32_t id = fnv1a_hash(hashInput, 12);
        
                uint8_t payload[32];
                uint8_t index = 0;
        
                if (packetCounter < 4) {
                    memcpy(&payload[index], &id, 4); index += 4;
                    memcpy(&payload[index], &data.latitude, 4); index += 4;
                    memcpy(&payload[index], &data.longitude, 4); index += 4;
                    memcpy(&payload[index], &data.timestamp, 4); index += 4;
                    payload[index++] = 0x00; // 1 byte vazio
                } else {
                    memcpy(&payload[index], &id, 4); index += 4;
                    memcpy(&payload[index], &data.latitude, 4); index += 4;
                    memcpy(&payload[index], &data.longitude, 4); index += 4;
                    memcpy(&payload[index], &data.altitude, 4); index += 4;
                    memcpy(&payload[index], &data.timestamp, 4); index += 4;
                    payload[index++] = 0x00; // 1 byte vazio
                }
        
                uint16_t crc = ComputeCRC(payload, index);
                payload[index++] = crc & 0xFF;
                payload[index++] = (crc >> 8) & 0xFF;
        
                buildFrame(id, payload, index);
                sendFrameEncrypted();
        
                packetCounter = (packetCounter + 1) % 5;
            }
        }

        void buildFrame(uint32_t id, uint8_t* payload, uint8_t payloadSize) {
            uint8_t i = 0;
            frame.size = payloadSize + 2;
            frame.buffer[i++] = id & 0xFF;
            frame.buffer[i++] = (id >> 8) & 0x03;
            memcpy(&frame.buffer[i], payload, payloadSize);
        }

        bool sendFrameEncrypted() {
            if (frame.size == 0) return false;
        
            uint32_t tamanho_cript = ((frame.size + 15) / 16) * 16;
            uint8_t buffer_cript[tamanho_cript];
            memset(buffer_cript, 0, tamanho_cript);
        
            AES_128(frame.buffer, buffer_cript, tamanho_cript);
        
            if(debug_serial) {
                Serial.print("TX (cript): ");
                printHex(buffer_cript, tamanho_cript);
            }
        
            if(frame.command)
                SerialLoRa->write(buffer_cript, tamanho_cript);
            else {
                if(SerialLoRat == NULL)
                    return false;
                else
                    SerialLoRat->write(buffer_cript, tamanho_cript);
            }
            return true;
        }

        void change_channel(uint32_t frequency_hz) {
           
            LoRa.setFrequency(frequency_hz / 1e6); // setFrequency usa MHz
        }

        bool receivePacketCommand(uint16_t* id, uint8_t* command, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout, uint32_t frequency_hz){
            uint16_t waitNextByte = 500;
            uint8_t index = 0;
            uint16_t crc = 0;
        
            LoRa.setFrequency(frequency_hz / 1e6); // MHz
            LoRa.receive();
        
            if(id == NULL || command == NULL || payload == NULL || payloadSize == NULL)
                return false;
        
            while(((timeout > 0) || (index > 0)) && (waitNextByte > 0))
            {
                if(SerialLoRa->available() > 0)
                {
                    uint8_t byteReceived = SerialLoRa->read();
                    frame.buffer[index++] = byteReceived;
                    waitNextByte = 200;
                }
                if(index > 0){
                    waitNextByte--;
                }
                timeout--;
                delay(1);
            }
        
            if(debug_serial && index > 0){
                Serial.print("RX (encrypted): ");
                printHex(frame.buffer, index);
            }
        
            if((timeout == 0) && (index == 0)) return false;
        
            crc = (uint16_t)frame.buffer[index - 2] | ((uint16_t)frame.buffer[index - 1] << 8);
            if(ComputeCRC(&frame.buffer[0], index - 2) != crc) return false;
        
            uint8_t decryptedBuffer[64];
            memset(decryptedBuffer, 0, sizeof(decryptedBuffer));
            AES_128_decrypt(frame.buffer, decryptedBuffer, index - 2);
        
            *id = (uint16_t)decryptedBuffer[0] | ((uint16_t)decryptedBuffer[1] << 8);
            *command = decryptedBuffer[2];
        
            *payloadSize = payloadSize
        
            // Copia o payload descriptografado
            memcpy(payload, &decryptedBuffer[3], *payloadSize);
        
            if(debug_serial){
                Serial.print("RX (decrypted): ");
                printHex(payload, *payloadSize);
            }
        
            bool sent = PrepareFrameTransp(*id, payload, *payloadSize);
            if(!sent) {
                if(debug_serial) Serial.println("PrepareFrameTransp failed.");
                return false;
            }
        
            return true;
        }

        void savePayloadOnRAM(uint32_t valor) {
            if ((uintptr_t)(next_free + sizeof(uint32_t)) < (uintptr_t)&_estack) {
                *((uint32_t*)next_free) = valor;
                next_free += sizeof(uint32_t);
            } else {
                // RAM cheia — entra em loop infinito ou reinicia
                while (1);
            }
        }

        bool receivePacketTransp(uint16_t* id, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout)
        {
            uint16_t waitNextByte = 500;
            uint8_t i = 0;
            
            
            if((id == NULL) && (deviceId == 0)) return false;
            if(payload == NULL) return false;
            if(payloadSize == NULL) return false;
            if(deviceId == -1) return false;
            */
            /* Waits for reception */
            while( ((timeout > 0 ) || (i > 0)) && (waitNextByte > 0) )
            {
                if(SerialLoRat->available() > 0)
                {
                    frame.buffer[i++] = SerialLoRat->read();
                    waitNextByte = 500;
                }
                
                if(i > 0)
                {
                waitNextByte--;
                }
                timeout--;
                delay(1);
            }

            if((timeout == 0) && (i == 0)) return false;

            if(deviceId == 0)
            {
                /* Copies ID */
                *id = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
                /* Copies payload size */
                *payloadSize = i-2;
                /* Copies payload */
                memcpy(payload, &frame.buffer[3], i-2);
            }
            else
            {
                /* Copies payload size */
                *payloadSize = i;
                /* Copies payload */
                memcpy(payload, &frame.buffer[0], i);
            }
            
            return sendPacket();
        }



        bool localRead(){
            uint8_t b = 0;
            
            bufferPayload[b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(0, 0xE2, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xE2)
                {
                    registered_password = bufferPayload[1] << 8;
                    registered_password += bufferPayload[0];
                    
                    localUniqueId = bufferPayload[2];
                    localUniqueId = bufferPayload[3] + (localUniqueId << 8);
                    localUniqueId = bufferPayload[4] + (localUniqueId << 8);
                    localUniqueId = bufferPayload[5] + (localUniqueId << 8);
                    
                    return true;
                }
            } 
            return false;
        }

        void begin(bool _debug_serial = false){
            debug_serial = _debug_serial;
            localRead();
            localId = (uint16_t)frame.buffer[0] | ((uint16_t)frame.buffer[1] << 8);
        }

        bool setNetworkID(uint16_t id){
            uint8_t b = 0;
            
            bufferPayload[b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 24);
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 16);
            bufferPayload[++b] = (uint8_t)(localUniqueId >> 8);
            bufferPayload[++b] = (uint8_t)(localUniqueId & 0xFF);
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x04;

            prepareFrameCommand(id, 0xCA, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xCA)
                return true;
            } 
            return false;
        }

        bool setPassword(uint32_t password){
            if(password < 0x00 || password < 0x00 || password > 0xFFFFFFFF)
                return false;
            uint8_t b = 0;

            uint32_t buffer_password; 
            
            bufferPayload[b] = 0x04;
            bufferPayload[++b] = password % 256;
            bufferPayload[++b] = (password / 256) & 0xFF;
            bufferPayload[++b] = ((password / 256) >> 8) & 0xFF;
            bufferPayload[++b] = ((password / 256) >> 16) & 0xFF;
            
            prepareFrameCommand(localId, 0xCD, bufferPayload, b + 1);

            buffer_password = bufferPayload[2] << 8;
            buffer_password += bufferPayload[1];
                
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xCD)
                {
                    localRead();
                    
                    if(buffer_password == registered_password)
                        return true;
                }
            }
            
            return false;  
        }

        bool getBPS(bool ignore_cmd = false){
            uint8_t b = 0;

            if(!ignore_cmd){
                bufferPayload[b] = 0x00;
                bufferPayload[++b] = 0x01;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(localId, 0xD6, bufferPayload, b + 1);
            }

            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xD6){
                    BW = bufferPayload[2];
                    SF = bufferPayload[3];
                    CR = bufferPayload[4];
                    return true;
                }
            }
            return false;
        }

        bool getClass(bool ignore_cmd = false){
            uint8_t b = 0;
            if(!ignore_cmd){
                bufferPayload[b] = 0x00;
                bufferPayload[++b] = 0xFF;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(localId, 0xC1, bufferPayload, b + 1);
            }
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                if(command == 0xC1){
                    LoRa_class = bufferPayload[2];
                    LoRa_window = bufferPayload[3];
            
                    return true;
                }
            }
            return false;
        }

        bool setBPS(uint8_t bandwidth = BW500, uint8_t spreading_factor = SF7, uint8_t coding_rate = CR4_5){
            if(bandwidth < 0x00 || bandwidth > 0x02)
                return false;
            else if(spreading_factor < 0x00 || spreading_factor > 0x00 && spreading_factor < 0x07 || spreading_factor > 0x0C)
                return false;
            else if(coding_rate < 0x00 || coding_rate < 0x01 || coding_rate > 0x04)
                return false;

            uint8_t b = 0;

            bufferPayload[b] = 0x01;
            bufferPayload[++b] = 0x14;
            bufferPayload[++b] = bandwidth;
            bufferPayload[++b] = spreading_factor;
            bufferPayload[++b] = coding_rate;
            
            prepareFrameCommand(localId, 0xD6, bufferPayload, b + 1);

            getBPS(true);

            if(BW == bandwidth && SF == spreading_factor && CR == coding_rate)
                return true;

            return false;
        }

        bool setClass(uint8_t lora_class = CLASS_C, uint8_t lora_window = WINDOW_5s){
            if(lora_class < 0x00 || lora_class != 0x00 && lora_class != 0x02)
                return false;
            else if(lora_window < 0x00 || lora_window > 0x02)
                return false;
            
            uint8_t b = 0;

            bufferPayload[b] = 0x00;
            bufferPayload[++b] = lora_class;
            bufferPayload[++b] = lora_window;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(localId, 0xC1, bufferPayload, b + 1);
            getClass(true);

            if(LoRa_class == lora_class && LoRa_window == lora_window)
                return true;
            
            return false;
        }    

        bool pinMode(uint8_t id, uint8_t gpio, uint8_t inout, uint8_t logical_level = LOW){
            if(gpio < 0x00 || gpio > 0x07)
                return false;
            /*else if(pull < 0x00 || pull > 0x02)
                return false;*/
            else if(inout < 0x00 || inout == 0x02 || inout == 0x03 && gpio < 0x05 && inout == 0x03 && gpio > 0x06 || inout > 0x03)
                return false;
            else if(logical_level < 0x00 || logical_level > 0x03)
                return false;
            uint8_t pull;

            if(inout == INPUT){
              pull = LoRa_NOT_PULL;

            }
            switch(inout){
                case INPUT:          inout = INOUT_DIGITAL_INPUT; pull = LoRa_NOT_PULL; break;
                case INPUT_PULLUP:   inout = INOUT_DIGITAL_INPUT; pull = LoRa_PULLUP; break;
                case INPUT_PULLDOWN: inout = INOUT_DIGITAL_INPUT; pull = LoRa_PULLDOWN; break;
            }

            uint8_t b = 0;
            if(gpio > 4 && gpio < 7 && inout == INOUT_DIGITAL_INPUT){
                bufferPayload[b] = 0x02;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = 0x00;
                bufferPayload[++b] = INOUT_ANALOG_INPUT;
                analogEnabled = true;
            }
            else{
                bufferPayload[b] = 0x02;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = pull;
                bufferPayload[++b] = inout;
                bufferPayload[++b] = logical_level;
            }

            prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);
            
            if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
            {
                if(command == 0xC2){
                return true;
                }
            }

            return false;
        }

        void getGPIOStatus(int16_t id, uint8_t gpio){
            uint8_t b = 0;

            bufferPayload[b] = 0x00;
            bufferPayload[++b] = gpio;
            bufferPayload[++b] = 0x00;
            bufferPayload[++b] = 0x00;

            prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);
        }

        uint16_t analogRead(int16_t id, uint8_t gpio){
            for(uint8_t i = 0; i < 3; ++i){
                getGPIOStatus(id, gpio);
                //0100C2 00 00 05 0A 79 1F0D
                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
                {
                    if(command == 0xC2){
                        uint16_t rawAnalog = bufferPayload[3]  << 8;
                        rawAnalog |= bufferPayload[4];
                        return rawAnalog;
                    }
                }
            }
            return 0;
        }

        uint8_t digitalRead(int16_t id, uint8_t gpio){
            getGPIOStatus(id, gpio);
            if(analogEnabled && gpio > 4 && gpio < 7){
                if(analogRead(id, gpio) >= 4096 / 2)
                    return 1;
                return 0;
            }
            else{
                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000))
                    if(command == 0xC2)
                        return bufferPayload[4];
            }
            return 0;
        } 

        bool digitalWrite(int16_t id, uint8_t gpio, uint8_t logical_level){
            if(gpio < 0x00 || gpio > 0x07)
                return false;
            else if(logical_level < 0x00 || logical_level > 0x03)
                return false;
            else if(analogEnabled && gpio > 4 && gpio < 7)
                return false;

            for(uint8_t i = 0; i < 3; ++i){
                uint8_t b = 0;

                bufferPayload[b] = 0x01;
                bufferPayload[++b] = gpio;
                bufferPayload[++b] = logical_level;
                bufferPayload[++b] = 0x00;

                prepareFrameCommand(id, 0xC2, bufferPayload, b + 1);

                if(receivePacketCommand(&localId, &command, bufferPayload, &payloadSize, 1000)){
                    if(command == 0xC2 && bufferPayload[2] == gpio && bufferPayload[3] == logical_level)
                        return true;
                }
            }

            return false;
        }

        int getNoise(uint8_t id, uint8_t select = 1){
            uint8_t b = 0;
            bufferPayload[b] = 0;
            bufferPayload[++b] = 0;
            bufferPayload[++b] = 0;

            if(select > 2)
                select = 2;

            prepareFrameCommand(id, 0xD8, bufferPayload, b + 1);
            delay(100);
            uint8_t cmd = 0;
            
            if(receivePacketCommand(&localId, &cmd, bufferPayload, &payloadSize, 270)){
                if(cmd == 0xD8)
                    return bufferPayload[select];
                else
                    return 255;
            }
            else
                delay(100);
                
            return 255;
        }

        int getR1(uint16_t rawADC, int R2){
            return  R2 * (4096 - rawADC) / rawADC;
        }

        double getTemp(uint16_t rawADC, int beta, int Rt = 10000, int R2 = 10000){
            double R1 = getR1(rawADC, R2);
            double rx = Rt * exp(-beta/(273.0 + 25.0));
            double t = beta / log(R1/rx);
            return t - 273;
        }
        
        void printHex(uint8_t* num, uint8_t tam){
            char hexCar[4];
            uint8_t index;
            
            for(index = 0; index < tam - 1; index++)
            {
                sprintf(hexCar, "%02X", num[index]);
                Serial.print(hexCar);
            }
            
            sprintf(hexCar, "%02X", num[index]);
            Serial.println(hexCar);  
        }
};
#endif
