#include "fairino_hardware/fairino_robot.hpp"
#include <sstream>
#include <iomanip>


namespace fairino_hardware{

fairino_robot::fairino_robot(){
//Configure the status feedback port and introduce the xmlrpc library
    _control_mode = 0;
    _controller_ip = CONTROLLER_IP;//IP-адрес контроллера по умолчанию
    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Start creating status feedback socket");    
    _socket_state = socket(AF_INET,SOCK_STREAM,0);//The status acquisition port is only TCP
    if(_socket_state == -1){
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot错误: Create status feedback socket: failed!");    
        exit(0);//Failed to create a set of characters, an error was thrown
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: create status feedback socket: Success, start connecting to the controller...");    
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(_port_state);//rt feedback port
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = connect(_socket_state,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        if(res1){
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Error: Unable to connect to the controller data port, the program exits!");    
            exit(0);//The connection failed, an error was thrown and returned
        }else{
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: The controller status port is connected successfully"); 
            //Setting the tcp channel is not non-blocking, if the timeout is set, the connection will be disconnected.
            int flags1 = fcntl(_socket_state,F_GETFL,0);
            fcntl(_socket_state,F_SETFL,flags1|SOCK_NONBLOCK);   
            // //Set the send timeout and accept timeout of the TCP channel
            // struct timeval timeout_val;
            // timeval.tv_sec = 1;//1s
            // timeval.tv_msec = 0;
            // setsockopt(_socket_state,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val));
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Start creating an xmlrpc connection...");    
    _xml_client_ptr = std::make_unique<XmlRpc::XmlRpcClient>(_controller_ip.c_str(), _port_xmlrpc);//Create an xmlrpc connection
    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: xmlrpc Successful connection!");    
    _xml_client_ptr->setKeepOpen(1);//Set keep link
}


fairino_robot::~fairino_robot(){
    if(_socket_state != -1){//Close the TCP port
        shutdown(_socket_state,SHUT_RDWR);
    }
    _xml_client_ptr->close();//Close the xmlrpc port
}


int fairino_robot::stoprobot(){
    XmlRpc::XmlRpcValue noArg, result;
    _xml_client_ptr->execute("StopMotion",noArg,result);
    return int(result);
    return 0;
}


int fairino_robot::inittorquecontrol(){
    if(!stoprobot()){
        _control_mode = TORQUE_CONTROL_MODE;
        return 0;
    }else{
        return -1;
    }
}


int fairino_robot::initpositioncontrol(){
//Do some preparatory work before calling servoj
    if(!stoprobot()){
        _control_mode = POSITION_CONTROL_MODE;
        return 0;
    }else{
        return -1;
    }
}

// необходимые include (если ещё не добавлены в начале .cpp)
#include <deque>
#include <vector>
#include <queue>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <algorithm>

// Вспомогательная функция: суммирование байт mod 65536
static uint16_t sum_bytes_mod16(const uint8_t* data, size_t len) {
    uint32_t s = 0;
    for (size_t i = 0; i < len; ++i) s += data[i];
    return static_cast<uint16_t>(s & 0xFFFF);
}

FR_rt_state& fairino_robot::read() {
    static FR_rt_state last_state{};
    static std::deque<uint8_t> acc;
    static std::queue<FR_rt_state> store_buff;
    const size_t HEADER_MIN = 2 + 1 + 2; // frame_head(2) + frame_cnt(1) + data_len(2)
    const size_t CHECKSUM_SZ = 2;

    uint8_t tmp[2048];
    ssize_t n = recv(_socket_state, tmp, sizeof(tmp), 0);

    if (n == 0) {
        std::lock_guard<std::mutex> lg(_reconnect_mutex);
        _is_reconnect.store(true);
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"), "Socket closed by peer (recv==0)");
        return last_state;
    }
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return last_state;
        std::lock_guard<std::mutex> lg(_reconnect_mutex);
        _is_reconnect.store(true);
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"), "recv() failed: %s (%d)", strerror(errno), errno);
        return last_state;
    }

    // добавляем ровно n байт в аккумулятор
    acc.insert(acc.end(), tmp, tmp + static_cast<size_t>(n));

    // парсим циклом: ищем заголовок и работаем с переменной длиной
    while (true) {
        if (acc.size() < HEADER_MIN) break;

        // найти заголовок 0x5A5A
        size_t header_index = SIZE_MAX;
        for (size_t i = 0; i + 1 < acc.size(); ++i) {
            if (acc[i] == 0x5A && acc[i+1] == 0x5A) { header_index = i; break; }
        }
        if (header_index == SIZE_MAX) {
            // нет заголовка — оставляем 1 байт (на случай частичного заголовка) и ждём
            while (acc.size() > 1) acc.pop_front();
            break;
        }

        // отбросим мусор до заголовка
        if (header_index > 0) {
            for (size_t k = 0; k < header_index; ++k) acc.pop_front();
            if (acc.size() < HEADER_MIN) break;
        }

        if (acc.size() < HEADER_MIN) break;

        // прочитать data_len (little-endian) — acc[3], acc[4]
        uint16_t data_len = static_cast<uint16_t>(acc[3]) | (static_cast<uint16_t>(acc[4]) << 8);

        // полный размер кадра = header_min + data_len + checksum_size
        size_t frame_len = HEADER_MIN + static_cast<size_t>(data_len) + CHECKSUM_SZ; // = 7 + data_len

        if (acc.size() < frame_len) break; // ждем полный кадр

        // скопируем кадр для удобной работы
        std::vector<uint8_t> packet(frame_len);
        for (size_t i = 0; i < frame_len; ++i) packet[i] = acc[i];

        // вычислим контрольную сумму по всем байтам до поля check_sum (т.е. packet[0..frame_len-3], length = frame_len-2)
        uint32_t sum = 0;
        size_t payload_len_for_checksum = frame_len - CHECKSUM_SZ;
        for (size_t i = 0; i < payload_len_for_checksum; ++i) sum += packet[i];
        uint16_t calc = static_cast<uint16_t>(sum & 0xFFFF);

        uint16_t pkt_le = static_cast<uint16_t>(packet[frame_len - 2]) |
                          (static_cast<uint16_t>(packet[frame_len - 1]) << 8);
        uint16_t pkt_be = static_cast<uint16_t>(packet[frame_len - 1]) |
                          (static_cast<uint16_t>(packet[frame_len - 2]) << 8);

        if (calc == pkt_le || calc == pkt_be) {
            //валидный кадр
            FR_rt_state st{};
            size_t tocopy = std::min(sizeof(FR_rt_state), frame_len);
            std::memcpy(&st, packet.data(), tocopy);
            if (tocopy < sizeof(FR_rt_state)) {
                RCLCPP_WARN(rclcpp::get_logger("FrHardwareInterface"),
                            "Received frame shorter than FR_rt_state (%zu < %zu).", tocopy, sizeof(FR_rt_state));
            }
            if (store_buff.size() >= 10) store_buff.pop();
            store_buff.emplace(st);
            last_state = st;

            // удалить обработанные байты
            for (size_t i = 0; i < frame_len; ++i) acc.pop_front();
        } else {
            // неверная контрольная сумма — лог и ресинхронизация
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                        "fairino_robot: verification failed (calc: %u, pkt_le: %u, pkt_be: %u, data_len: %u, acc.size=%zu)",
                        (unsigned)calc, (unsigned)pkt_le, (unsigned)pkt_be, (unsigned)data_len, acc.size());
            acc.pop_front(); // сдвигаем окно на 1 байт и ищем следующий заголовок
        }
    } // while

    if (!store_buff.empty()) {
        last_state = store_buff.front();
        store_buff.pop();
    }
    return last_state;
}



void fairino_robot::write(double cmd[6])
{
    if (_control_mode != POSITION_CONTROL_MODE) return;

    XmlRpc::XmlRpcValue Args, result;

    // Args[0] = joint positions (6)
    Args[0].setSize(6);
    for (int i = 0; i < 6; ++i) Args[0][i] = static_cast<double>(cmd[i]);

    // Args[1] = tool vector (4) — zeros
    Args[1].setSize(4);
    for (int i = 0; i < 4; ++i) Args[1][i] = 0.0;

    // Args[2..6] = acc, vel, cmdT, filterT, gain
    Args[2] = 0.0;      // acc
    Args[3] = 0.0;      // vel
    Args[4] = 0.008;    // cmdT (8 ms)
    Args[5] = 0.0;      // filterT
    Args[6] = 0.0;      // gain

    // Args[7] = extra flag (int) to make total top-level args = 8
    Args[7] = static_cast<int>(0);

    try {
        if (!_xml_client_ptr->execute("ServoJ", Args, result)) {
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                        "fairino_robot: The instruction was not sent successfully");
            return;
        }

        // минимальный разбор результата: int | array[int] | struct{faultCode,faultString} | array[struct]
        int rc = 0;
        bool have_rc = false;

        if (result.getType() == XmlRpc::XmlRpcValue::TypeInt ||
            result.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            rc = static_cast<int>(result);
            have_rc = true;
        } else if (result.getType() == XmlRpc::XmlRpcValue::TypeArray && result.size() > 0) {
            XmlRpc::XmlRpcValue first = result[0];
            if (first.getType() == XmlRpc::XmlRpcValue::TypeInt ||
                first.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                rc = static_cast<int>(first);
                have_rc = true;
            } else if (first.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                if (first.hasMember("faultCode")) {
                    rc = static_cast<int>(first["faultCode"]);
                    have_rc = true;
                }
                if (first.hasMember("faultString")) {
                    std::string fs = static_cast<std::string>(first["faultString"]);
                    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                                "fairino_robot: ServoJ fault: %s", fs.c_str());
                }
            }
        } else if (result.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            if (result.hasMember("faultCode")) {
                rc = static_cast<int>(result["faultCode"]);
                have_rc = true;
            }
            if (result.hasMember("faultString")) {
                std::string fs = static_cast<std::string>(result["faultString"]);
                RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                            "fairino_robot: ServoJ fault: %s", fs.c_str());
            }
        }

        if (have_rc && rc != 0) {
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                        "fairino_robot: Received instruction feedback error code %d", rc);
        }

    } catch (const XmlRpc::XmlRpcException &e) {
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                    "fairino_robot: ServoJ XmlRpcException: %s", e.getMessage().c_str());
    } catch (const std::exception &e) {
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),
                    "fairino_robot: ServoJ exception: %s", e.what());
    }
}




}//end namespace
