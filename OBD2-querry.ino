#include <ESP32-TWAI-CAN.hpp>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"
#include "driver/twai.h" // 直接包含ESP32 TWAI驱动

// Simple sketch that querries OBD2 over CAN for coolant temperature
// Showcasing simple use of ESP32-TWAI-CAN library driver.

// Default for ESP32
#define CAN_TX      5
#define CAN_RX      4

// ESP-NOW广播地址
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 使用全局CAN帧对象
CanFrame rxFrame = { 0 };  // 确保初始化为0

// 数据结构用于存储解析后的车辆数据
struct VehicleData {
    uint16_t rpm;
    float speed_kph;
    uint8_t throttle_pos;
    bool dataReady;  // 标记数据是否已更新
} vehicleData;

// ELM327响应常量
const char* ELM_INIT = "ATZ\r\rELM327 v1.5\r\r>";
const char* ELM_OK = "OK\r\r>";
const char* ELM_ERROR = "?\r\r>";

// 添加CAN总线错误恢复相关变量
unsigned long lastErrorCheckTime = 0;
const unsigned long ERROR_CHECK_INTERVAL = 2000; // 每2秒检查一次
unsigned int consecutiveErrors = 0;
const unsigned int MAX_ERRORS_BEFORE_RESET = 3;  // 3次连续错误后重置总线

// 添加OBD协议配置
bool useExtendedFrames = true;  // 默认使用29位扩展帧
#define OBD_STD_REQUEST_ID     0x7DF   // 11位标准帧请求ID
#define OBD_STD_RESPONSE_ID    0x7E8   // 11位标准帧响应ID
#define OBD_EXT_REQUEST_ID     0x18DB33F1  // 29位扩展帧请求ID
#define OBD_EXT_RESPONSE_BASE  0x18DAF100  // 29位扩展帧响应基址

// 定义支持的PID
#define PID_ENGINE_RPM      0x0C
#define PID_VEHICLE_SPEED   0x0D
#define PID_THROTTLE_POS    0x11
#define PID_ENGINE_RUNTIME  0x1F
#define PID_FUEL_LEVEL      0x2F
#define PID_AMBIENT_TEMP    0x46
#define PID_CUSTOM_DOORS_STATUS  0xD0    // 车门状态
#define PID_CUSTOM_GEARBOX      0xD1    // 变速箱状态
#define PID_CUSTOM_SCM_STATUS    0xD2    // SCM状态
#define PID_CUSTOM_STEERING     0xD3    // 转向传感器状态
#define PID_CUSTOM_DRIVE_MODE   0xD4    // 驾驶模式状态

// DBC相关定义
#define CAN_ID_ENGINE_DATA1    0x158    // 示例：发动机数据1
#define CAN_ID_ENGINE_DATA2    0x17C    // 示例：发动机数据2
#define CAN_ID_OBD_RESPONSE    0x18DAF133    // OBD-II 29位扩展帧响应ID
#define CAN_ID_DOORS_STATUS    0x405    // DOORS_STATUS报文ID
#define CAN_ID_GEARBOX         0x1A3    // GEARBOX报文ID
#define CAN_ID_SCM_FEEDBACK     0x326    // SCM_FEEDBACK报文ID
#define CAN_ID_STEERING_SENSORS 0x14A    // STEERING_SENSORS报文ID
#define CAN_ID_XXX_16          0x221    // XXX_16报文ID

// 在变速箱档位值定义
enum GEAR_SHIFTER_VALUES {
    GEAR_SHIFTER_P = 4,
    GEAR_SHIFTER_R = 8,
    GEAR_SHIFTER_N = 16,
    GEAR_SHIFTER_D = 32,
    GEAR_SHIFTER_S = 2
};

enum GEAR_VALUES {
    GEAR_P = 17,
    GEAR_R = 18,
    GEAR_N = 19,
    GEAR_D = 20,
    GEAR_S = 26
};

// 添加新的函数声明
void parseOBDResponse(const CanFrame& frame);
void parseVehicleData(const CanFrame& frame);
bool isOBDResponse(uint32_t canId);  // 新增：检查是否为OBD响应ID
void sendOBDRequest(uint8_t mode, uint8_t pid, bool useExtended);  // 新增：发送OBD请求

// 添加发送频率控制
#define MIN_SEND_INTERVAL    50  // 最小发送间隔(ms)
unsigned long lastSendTime = 0;   // 上次发送时间

// CAN ID发送频率配置（毫秒）
struct CANIDConfig {
    uint32_t id;           // CAN ID
    uint16_t interval;     // 最小发送间隔(ms)
    uint32_t lastSendTime; // 上次发送时间
    CANIDConfig(uint32_t _id, uint16_t _interval) : 
        id(_id), interval(_interval), lastSendTime(0) {}
};

// 修改CAN ID配置，移除OBD响应的频率限制
CANIDConfig canConfigs[] = {
    {CAN_ID_ENGINE_DATA1,     50},  // 发动机数据 20Hz
    {CAN_ID_ENGINE_DATA2,     50},  // 节气门位置 20Hz
    {CAN_ID_DOORS_STATUS,     200}, // 车门状态 5Hz
    {CAN_ID_GEARBOX,          200}, // 变速箱状态 5Hz
    {CAN_ID_SCM_FEEDBACK,     200}, // SCM状态 5Hz
    {CAN_ID_STEERING_SENSORS, 100}, // 转向传感器 10Hz
    {CAN_ID_XXX_16,           200}   // XXX_16 5Hz
};

// 检查是否可以发送特定CAN ID的数据
bool canSendData(uint32_t canId) {
    unsigned long currentTime = millis();
    
    // 查找对应的CAN ID配置
    for(auto& config : canConfigs) {
        if(config.id == canId) {
            // 检查是否达到发送间隔
            if(currentTime - config.lastSendTime >= config.interval) {
                config.lastSendTime = currentTime;
                return true;
            }
            Serial.printf("CAN ID 0x%03X 当前间隔: %lu ms\n", 
                        canId, currentTime - config.lastSendTime);
            return false;
        }
    }
    
    // 未找到配置的CAN ID不进行频率限制
    Serial.printf("CAN ID 0x%03X 未配置频率限制\n", canId);
    return true;
}

// 检查是否为OBD响应ID
bool isOBDResponse(uint32_t canId) {
    if (useExtendedFrames) {
        // 29位扩展帧：检查响应ID范围 (0x18DAF100 to 0x18DAF1FF)
        return ((canId & 0xFFFFFF00) == OBD_EXT_RESPONSE_BASE);
    } else {
        // 11位标准帧：检查响应ID是否为0x7E8
        return (canId == OBD_STD_RESPONSE_ID);
    }
}

// 新增：重置CAN总线函数
void resetCANBus() {
    Serial.println("正在重置CAN总线...");
    
    // 停止CAN总线
    ESP32Can.end();
    delay(200);  // 稍微延长等待时间
    
    // 重新初始化CAN总线
    if (!ESP32Can.begin(TWAI_SPEED_500KBPS)) {
        Serial.println("CAN总线重新初始化失败!");
        setLED(false);  // LED指示错误
        delay(1000);
        esp_restart();  // 如果重新初始化失败，重启设备
    }
    
    Serial.println("CAN总线已重置");
    consecutiveErrors = 0;  // 重置错误计数
}

// 修改：简化的CAN总线状态检查
void checkCANBusStatus() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastErrorCheckTime >= ERROR_CHECK_INTERVAL) {
        lastErrorCheckTime = currentTime;
        
        // 使用直接的TWAI驱动API获取状态
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            // 如果处于总线错误状态，重置总线
            if (status.state == TWAI_STATE_BUS_OFF || 
                status.state == TWAI_STATE_RECOVERING) {
                Serial.printf("CAN总线状态异常: %d, 需要重置\n", status.state);
                resetCANBus();
            }
            
            // 打印状态信息用于调试
            Serial.printf("CAN状态: 状态=%d, 发送错误=%d, 接收错误=%d, Tx排队=%d, Rx排队=%d\n",
                         status.state, status.tx_error_counter, status.rx_error_counter,
                         status.msgs_to_tx, status.msgs_to_rx);
        }
    }
}

// 发送OBD请求函数
void sendOBDRequest(uint8_t mode, uint8_t pid, bool useExtended) {
    CanFrame requestFrame;
    
    if (useExtended) {
        // 使用29位扩展帧
        requestFrame.identifier = OBD_EXT_REQUEST_ID;
        requestFrame.extd = 1;
    } else {
        // 使用11位标准帧
        requestFrame.identifier = OBD_STD_REQUEST_ID;
        requestFrame.extd = 0;
    }
    
    requestFrame.data_length_code = 8;
    
    // 设置请求数据
    requestFrame.data[0] = 0x02;  // 数据长度
    requestFrame.data[1] = mode;  // 模式
    requestFrame.data[2] = pid;   // PID
    
    // 填充剩余字节为0
    for(int i = 3; i < 8; i++) {
        requestFrame.data[i] = 0x00;
    }
    
    // 使用非阻塞发送
    if (ESP32Can.writeFrame(requestFrame)) {
        Serial.printf("已发送OBD请求 - ID: 0x%X, 模式: %02X, PID: %02X, %s\n", 
                    requestFrame.identifier, mode, pid, 
                    useExtended ? "29位扩展帧" : "11位标准帧");
        consecutiveErrors = 0;  // 成功发送，重置错误计数
    } else {
        Serial.println("OBD请求发送失败");
        consecutiveErrors++;
        
        // 如果连续失败次数超过阈值，重置CAN总线
        if (consecutiveErrors >= MAX_ERRORS_BEFORE_RESET) {
            resetCANBus();
        }
    }
}

// ESP-NOW接收回调
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
    // 检查数据长度
    if (data_len < 2) return;
    
    // 打印接收到的原始数据
    Serial.print("收到OBD请求: ");
    for(int i = 0; i < data_len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    // 解析OBD请求
    CanFrame frame;
    
    // 检查是否是AT命令
    if (data[0] == 'A' && data[1] == 'T') {
        // 处理AT命令
        if (strncmp((char*)data, "ATZ", 3) == 0) {
            sendElm327Response(ELM_INIT, 0);
        } else if (strncmp((char*)data, "ATSP", 4) == 0) {
            // 检查是否设置协议为CAN (11/500 或 29/500)
            if (data_len >= 5) {
                if (data[4] == '6') { // ATSP6 - ISO 15765-4, CAN (11/500)
                    useExtendedFrames = false;
                    Serial.println("设置为11位标准帧模式");
                } else if (data[4] == '7') { // ATSP7 - ISO 15765-4, CAN (29/500)
                    useExtendedFrames = true;
                    Serial.println("设置为29位扩展帧模式");
                }
            }
            sendElm327Response(ELM_OK, 0);
        } else {
            sendElm327Response(ELM_OK, 0);
        }
        return;
    }

    // 解析模式和PID
    // 示例: "01 0C" -> 模式01, PID 0C
    uint8_t mode = 0;
    uint8_t pid = 0;
    
    // 将ASCII十六进制转换为数值
    auto hex2int = [](char c) -> uint8_t {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return 0;
    };

    // 解析模式（前两个字符）
    if (isxdigit(data[0]) && isxdigit(data[1])) {
        mode = (hex2int(data[0]) << 4) | hex2int(data[1]);
    }

    // 解析PID（后两个字符，如果存在）
    if (data_len >= 4 && isxdigit(data[2]) && isxdigit(data[3])) {
        pid = (hex2int(data[2]) << 4) | hex2int(data[3]);
    }
    
    // 发送OBD请求（使用当前帧类型设置）
    sendOBDRequest(mode, pid, useExtendedFrames);
}

// ESP-NOW发送回调
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_OK) {
        Serial.println("发送失败");
    }
}

// 修改sendElm327Response函数，移除重复的频率检查
void sendElm327Response(const char* response, uint32_t canId) {
    size_t len = strlen(response);
    if (len > 31) {  // 确保不超过缓冲区大小
        Serial.println("响应数据过长");
        return;
    }
    
    // 发送数据
    if (esp_now_send(broadcastAddress, (uint8_t*)response, len) == ESP_OK) {
        Serial.printf("ESP-NOW发送[0x%03X]: %s\n", canId, response);
    } else {
        Serial.println("ESP-NOW发送失败");
    }
}

// OBD响应解析函数
void parseOBDResponse(const CanFrame& frame) {
    // 验证数据长度
    if (frame.data[0] < 2 || frame.data[0] > 7) {
        Serial.println("无效的OBD响应长度");
        return;
    }
    
    // 32字节足够存储最长的响应
    char response[32] = {0};
    char temp[8];
    
    // 添加模式字节验证
    if (frame.data[1] != 0x41 && frame.data[1] != 0x42) {
        Serial.printf("未支持的OBD响应模式: %02X\n", frame.data[1]);
        sendElm327Response(ELM_ERROR, frame.identifier);
        return;
    }
    
    // 检查剩余缓冲区大小
    size_t remaining = sizeof(response);
    char* ptr = response;
    
    for(int i = 1; i <= frame.data[0]; i++) {
        int written = snprintf(ptr, remaining, "%02X ", frame.data[i]);
        if (written < 0 || written >= remaining) {
            Serial.println("响应缓冲区溢出");
            return;
        }
        ptr += written;
        remaining -= written;
    }
    
    // 确保有足够空间添加结尾
    if (remaining < 4) {  // "\r\r>" + null terminator
        Serial.println("响应缓冲区不足");
        return;
    }
    strcat(response, "\r\r>");
    
    Serial.printf("解析OBD响应: %s\n", response);
    sendElm327Response(response, frame.identifier);
}

// 修改parseVehicleData函数，提前进行频率检查
void parseVehicleData(const CanFrame& frame) {
    // 提前检查发送频率（除了OBD响应）
    if (frame.identifier != CAN_ID_OBD_RESPONSE && !canSendData(frame.identifier)) {
        Serial.printf("CAN ID 0x%03X 发送间隔过短，跳过解析\n", frame.identifier);
        return;
    }

    bool dataUpdated = false;
    char response[32] = {0};  // 32字节的缓冲区
    
    switch(frame.identifier) {
        case CAN_ID_ENGINE_DATA1: {
            // 解析车速 (比例因子 0.01)
            uint16_t speed_raw = (frame.data[0] << 8) | frame.data[1];
            vehicleData.speed_kph = speed_raw * 0.01;
            
            // 解析发动机转速
            vehicleData.rpm = (frame.data[2] << 8) | frame.data[3];
            
            Serial.printf("从CAN ID 0x158解析 - 车速: %.2f km/h, 转速: %d rpm\n", 
                         vehicleData.speed_kph, vehicleData.rpm);

            // 发送车速PID响应
            int len = snprintf(response, sizeof(response), "41 0D %02X\r\r>", 
                             (uint8_t)vehicleData.speed_kph);
            if (len < 0 || len >= sizeof(response)) {
                Serial.println("车速响应缓冲区溢出");
                return;
            }
            sendElm327Response(response, frame.identifier);
            
            // 发送转速PID响应
            uint16_t rpm_value = vehicleData.rpm * 4;
            len = snprintf(response, sizeof(response), "41 0C %02X %02X\r\r>",
                         (rpm_value >> 8) & 0xFF,
                         rpm_value & 0xFF);
            if (len < 0 || len >= sizeof(response)) {
                Serial.println("转速响应缓冲区溢出");
                return;
            }
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_ENGINE_DATA2: {
            vehicleData.throttle_pos = frame.data[0];
            Serial.printf("从CAN ID 0x17C解析 - 节气门开度: %d%%\n", 
                         vehicleData.throttle_pos);
            
            // 发送节气门位置PID响应
            uint8_t throttle = (vehicleData.throttle_pos * 255) / 100;
            sprintf(response, "41 11 %02X\r\r>", throttle);
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_DOORS_STATUS: {
            // 解析车门状态
            uint8_t doors_status = 0;
            
            // 从bit37-41提取各个车门状态
            bool door_fl = (frame.data[4] >> 5) & 0x01;  // bit37
            bool door_fr = (frame.data[4] >> 6) & 0x01;  // bit38
            bool door_rl = (frame.data[4] >> 7) & 0x01;  // bit39
            bool door_rr = (frame.data[5] >> 0) & 0x01;  // bit40
            bool trunk = (frame.data[5] >> 1) & 0x01;    // bit41
            
            // 将所有状态组合到一个字节中
            doors_status = (door_fl << 0) |      // bit0: 左前门
                          (door_fr << 1) |      // bit1: 右前门
                          (door_rl << 2) |      // bit2: 左后门
                          (door_rr << 3) |      // bit3: 右后门
                          (trunk << 4);         // bit4: 后备箱
            
            Serial.printf("从CAN ID 0x405解析 - 车门状态: FL:%d FR:%d RL:%d RR:%d TRUNK:%d\n",
                         door_fl, door_fr, door_rl, door_rr, trunk);
            
            // 构造自定义PID响应 (PID: 0xD0)
            sprintf(response, "41 D0 %02X\r\r>", doors_status);
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_GEARBOX: {
            // 解析变速箱状态 - 只解析实际档位 (32-39 bits)
            uint8_t gear = frame.data[4];             // byte 4 (32-39 bits)
            
            // 将档位值转换为更简单的格式
            uint8_t gear_status;
            switch(gear) {
                case GEAR_P: gear_status = 0x01; break;  // P档
                case GEAR_R: gear_status = 0x02; break;  // R档
                case GEAR_N: gear_status = 0x03; break;  // N档
                case GEAR_D: gear_status = 0x04; break;  // D档
                case GEAR_S: gear_status = 0x05; break;  // S档
                default:    gear_status = 0x00; break;   // 未知档位
            }
            
            Serial.printf("从CAN ID 0x1A3解析 - 实际档位: %d\n", gear);
            
            // 构造自定义PID响应 (PID: 0xD1)
            sprintf(response, "41 D1 %02X\r\r>", gear_status);
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_SCM_FEEDBACK: {
            // 解析SCM状态
            uint8_t scm_status = 0;
            
            // 从各个位提取状态
            bool driver_door = (frame.data[2] >> 1) & 0x01;  // bit17
            bool left_blinker = (frame.data[3] >> 2) & 0x01; // bit26
            bool right_blinker = (frame.data[3] >> 3) & 0x01;// bit27
            bool main_on = (frame.data[3] >> 4) & 0x01;      // bit28
            uint8_t cmbs_states = (frame.data[2] >> 6) & 0x03; // bit22-23
            
            // 将所有状态组合到一个字节中
            scm_status = (driver_door << 0)   |   // bit0: 驾驶员门
                         (left_blinker << 1)  |   // bit1: 左转向灯
                         (right_blinker << 2) |   // bit2: 右转向灯
                         (main_on << 3)       |   // bit3: MAIN开关
                         (cmbs_states << 4);      // bit4-5: CMBS状态
            
            Serial.printf("从CAN ID 0x326解析 - 驾驶员门:%d 左转:%d 右转:%d MAIN:%d CMBS:%d\n",
                         driver_door, left_blinker, right_blinker, main_on, cmbs_states);
            
            // 构造自定义PID响应 (PID: 0xD2)
            sprintf(response, "41 D2 %02X\r\r>", scm_status);
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_STEERING_SENSORS: {
            // 解析转向角度 (7|16@0- (-0.1,0))
            int16_t steer_angle_raw = (frame.data[1] << 8) | frame.data[0];
            float steer_angle = steer_angle_raw * -0.1;  // 比例因子-0.1
            
            // 解析转向角速度 (23|16@0- (-1,0))
            int16_t steer_rate_raw = (frame.data[2] << 8) | frame.data[3];
            float steer_rate = steer_rate_raw * -1.0;    // 比例因子-1
            
            // 解析传感器状态位
            uint8_t sensor_status = ((frame.data[4] >> 0) & 0x07);  // bit32-34
            
            // 解析方向盘角度 (47|16@0- (-0.1,0))
            int16_t wheel_angle_raw = (frame.data[5] << 8) | frame.data[6];
            float wheel_angle = wheel_angle_raw * -0.1;  // 比例因子-0.1
            
            Serial.printf("从CAN ID 0x14A解析 - 转向角:%.1f° 角速度:%.1f°/s 方向盘:%.1f° 状态:%d\n",
                         steer_angle, steer_rate, wheel_angle, sensor_status);
            
            // 构造自定义PID响应 (PID: 0xD3)
            // 发送4个字节：转向角(2字节) + 角速度(1字节) + 传感器状态(1字节)
            int16_t angle_value = (int16_t)(steer_angle * 10);  // 放大10倍以保持精度
            uint8_t rate_value = (uint8_t)(abs(steer_rate) * 0.1);  // 转换为0-255范围
            
            sprintf(response, "41 D3 %02X %02X %02X %02X\r\r>",
                    (angle_value >> 8) & 0xFF,    // 转向角高字节
                    angle_value & 0xFF,           // 转向角低字节
                    rate_value,                   // 角速度
                    sensor_status);               // 传感器状态
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        case CAN_ID_XXX_16: {
            // 解析驾驶模式相关状态
            
            // 解析ECON开关状态 (bit23)
            bool econ_on = (frame.data[2] >> 7) & 0x01;     // bit23
            
            // 解析驾驶模式 (bit37-38)
            uint8_t drive_mode = (frame.data[4] >> 5) & 0x03;  // bit37-38
            
            // 将ECON状态和驾驶模式分开发送
            uint8_t econ_status = econ_on ? 0x01 : 0x00;    // 第一个字节: ECON状态
            uint8_t mode_status = drive_mode;               // 第二个字节: 驾驶模式
            
            Serial.printf("从CAN ID 0x221解析 - ECON:%d 驾驶模式:%d\n",
                         econ_on, drive_mode);
            
            // 构造自定义PID响应 (PID: 0xD4)
            // 返回两个字节：ECON状态 + 驾驶模式
            int len = snprintf(response, sizeof(response), "41 D4 %02X %02X\r\r>", 
                             econ_status, mode_status);
            if (len < 0 || len >= sizeof(response)) {
                Serial.println("驾驶模式响应缓冲区溢出");
                return;
            }
            sendElm327Response(response, frame.identifier);
            
            dataUpdated = true;
            break;
        }

        default:
            Serial.printf("未处理的CAN ID: 0x%03X\n", frame.identifier);
            break;
    }

    if (dataUpdated) {
        vehicleData.dataReady = true;
    }
}

// WS2812定义
#define WS2812_PIN      21      // WS2812 LED引脚
#define WS2812_NUM      1       // WS2812 LED数量
#define WS2812_BRIGHTNESS 50    // WS2812亮度 (0-255)

// 创建WS2812对象
Adafruit_NeoPixel ws2812(WS2812_NUM, WS2812_PIN, NEO_GRB + NEO_KHZ800);

// LED控制函数
void setLED(bool state) {
    if (state) {
        ws2812.setPixelColor(0, ws2812.Color(0, 0, WS2812_BRIGHTNESS));  // 蓝色
    } else {
        ws2812.setPixelColor(0, ws2812.Color(0, 0, 0));  // 关闭
    }
    ws2812.show();
}

// 修改看门狗配置
#define WDT_TIMEOUT_SECONDS 5  // 5秒超时

// 在适当的位置添加内存检查函数
void checkHeapMemory() {
    static uint32_t lastCheck = 0;
    const uint32_t CHECK_INTERVAL = 10000;  // 每10秒检查一次
    
    if (millis() - lastCheck >= CHECK_INTERVAL) {
        Serial.printf("可用堆内存: %d bytes\n", ESP.getFreeHeap());
        lastCheck = millis();
    }
}

// 添加定时查询车速的函数
void queryVehicleSpeed() {
    static unsigned long lastQueryTime = 0;
    const unsigned long QUERY_INTERVAL = 1000;  // 1秒间隔
    
    unsigned long currentTime = millis();
    if (currentTime - lastQueryTime >= QUERY_INTERVAL) {
        // 发送OBD请求，使用当前帧类型设置
        sendOBDRequest(0x01, 0x05, useExtendedFrames);
        lastQueryTime = currentTime;
    }
}

void setup() {
    // 初始化WS2812
    ws2812.begin();
    ws2812.setBrightness(WS2812_BRIGHTNESS);
    ws2812.clear();
    ws2812.show();
    
    // 开机指示
    setLED(true);
    
    Serial.begin(115200);
    Serial.println("\n=== Starting ESP32 OBD2 Bridge ===");
    
    // 初始化CAN总线 - 使用ESP32Can而不是自己创建对象
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if (!ESP32Can.begin(TWAI_SPEED_500KBPS)) {
        Serial.println("CAN总线初始化失败!");
        setLED(false);  // LED指示错误
        while(1) {
            delay(1000);
        }
    }

    // 初始化WiFi为站点模式
    WiFi.mode(WIFI_STA);

    // 初始化ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW初始化错误");
        setLED(false);  // LED指示错误
        while(1) {
            delay(1000);
        }
    }

    // 注册ESP-NOW回调
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // 添加ESP-NOW对等点(广播)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    Serial.println("初始化完成，等待命令...");

    // 初始化看门狗
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // 监控所有核心
        .trigger_panic = true  // 启用panic处理
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);  // 将当前任务添加到看门狗
}

void loop() {
    checkHeapMemory();
    // 喂狗
    esp_task_wdt_reset();
    
    // 检查CAN总线状态
    checkCANBusStatus();
    
    // 定期查询车速
    //queryVehicleSpeed();
    
    // 使用ESP32Can接收消息
    if (ESP32Can.readFrame(rxFrame)) {
        // 收到数据时点亮LED
        setLED(true);
        
        // 检查是否为OBD响应
        if (isOBDResponse(rxFrame.identifier)) {
            Serial.printf("收到CAN帧 - ID: 0x%08X, DLC: %d, 扩展帧: %d\n", 
                rxFrame.identifier, 
                rxFrame.data_length_code,
                rxFrame.extd);
            parseOBDResponse(rxFrame);
        } else {
            parseVehicleData(rxFrame);
        }
        
        // 处理完成后关闭LED
        setLED(false);
    }
}