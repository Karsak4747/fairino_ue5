#include "rclcpp/rclcpp.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ue_listener");

    uint8_t* data;
    key_t key = ftok("/dev/shm/data.conf", 1);
    int shmid = shmget(key, 256, 0666|IPC_CREAT);
    if(shmid == -1){
        RCLCPP_ERROR(node->get_logger(), "ERROR CREATING SHARED MEMORY SEGMENT");
    }else{
        data = (uint8_t*)shmat(shmid, NULL, 0);
        if(data == (void*)-1){
            RCLCPP_ERROR(node->get_logger(), "ERROR CREATING SHARED SEGMENT");
        }
    }
    int id = 0;
    const std::string msg = "Hello World";
    char buffer[msg.length()+1];
    RCLCPP_INFO(node->get_logger(), "Start Listening...");
    while(rclcpp::ok()){
        if(shmid != -1 && data != (void*)-1){
            int* q = (int*)data;
            int msg_id = *q;
            q++;
            if(msg_id > id){
                id = msg_id;
                char* c = (char*)q;
                for(int i = 0; i < msg.length() + 1; i++){
                    buffer[i] = *c;
                    c++;
                }

                RCLCPP_INFO(node->get_logger(), "I heard: '%s' - '%d'", buffer, msg_id);
            }
        }
    }

    shmdt(data);
    RCLCPP_INFO(node->get_logger(), "REALEASED SHARED MEMORY SEGMENT");
    return 0;
}
