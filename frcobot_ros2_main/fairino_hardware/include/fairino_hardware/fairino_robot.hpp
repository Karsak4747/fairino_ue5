#ifndef _FR_ROBOT_
#define _FR_ROBOT_

// C++ стандартные заголовки (используем угловые скобки)
#include <thread>
#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <queue>
#include <iostream>
#include <vector>   // может пригодиться для буферов в cpp
#include <cstring>  // memcpy, ... если нужно

// Системные заголовки (оставлены ваши)
#include "data_type_def.h"
#include "xmlrpc/XmlRpc.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "fcntl.h"

#include "rclcpp/rclcpp.hpp"

namespace fairino_hardware {

class fairino_robot {
public:
    explicit fairino_robot();
    ~fairino_robot();

    int inittorquecontrol();
    int initpositioncontrol();
    int stoprobot();

    // Сохранена исходная сигнатура
    FR_rt_state& read();
    void write(double cmd[6]); // servoJ and servoJT

    friend class FairinoHardwareInterface;

protected:
    // Конфигурация / соединение
    std::string _controller_ip;
    std::unique_ptr<std::thread> _control_thread;                // Enter position or torque information
    std::unique_ptr<XmlRpc::XmlRpcClient> _xml_client_ptr;       // xmlrpc client
    int _port_xmlrpc = 20003;
    int _port_state  = 20004;

    // Инициализируем как -1 для явности (socket not open)
    int _socket_state = -1;

    int _control_mode; // 0-none, 1-Position control, 2-Torque control

private:
    // Флаг реконнекта: atomic для быстрого и безопасного чтения/записи из разных потоков
    std::atomic<bool> _is_reconnect{false};

    // Мьютекс для защиты сложных операций реконнекта (закрытие/переоткрытие сокета и т.п.)
    std::mutex _reconnect_mutex;
};

} // namespace fairino_hardware

#endif // _FR_ROBOT_
