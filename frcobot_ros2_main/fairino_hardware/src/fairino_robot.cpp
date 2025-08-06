#include "fairino_hardware/fairino_robot.hpp"

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

FR_rt_state& fairino_robot::read(){
//Call TCP communication, directly use the memcpy function to copy, no need to expand the data
    static FR_rt_state state_data;
    static std::queue<FR_rt_state> store_buff;//Used to store excess data in the buffer area, the length needs to be limited
    static char temp_buff[RT_PACKAGE_SIZE] = {0};
    static int future_recv = RT_PACKAGE_SIZE;
    int datalen = RT_PACKAGE_SIZE;
    uint16_t* head_ptr;
    uint8_t* checksum_ptr;
    uint16_t checksum = 0;
    while(datalen > 0){
        char recv_buff[future_recv] = {0};
        datalen = recv(_socket_state,recv_buff,sizeof(recv_buff),0);//Get data from the cache area
        head_ptr = (uint16_t*)(recv_buff);//Get the frame header
        checksum_ptr = (uint8_t*)(recv_buff);//And check pointer
        if(*head_ptr == 0x5A5A){//Detect header
            if(datalen == RT_PACKAGE_SIZE){//Sometimes the data at the end of the buffer is not a complete frame, so incomplete information needs to be saved for the next frame of data splicing.
                for(int i=0;i< datalen-2;i++){
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                }
                if(checksum == *((uint16_t*)checksum_ptr)){//Perform and verify
                    memcpy(&state_data,recv_buff,sizeof(recv_buff));
                    if(store_buff.size() < 10){
                        store_buff.emplace(state_data);//Put data in the queue
                        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Get complete data,size:%d",datalen);    
                    	}else{//If there are more than 10 data in the queue，then delete the data at the head of the queue and then insert the data at the end of the queue
                        	store_buff.pop();//Pop up the element of the team leader
                        	store_buff.emplace(state_data);
                    	}
                    }else{//And the verification does not pass, error information is lost, and no information is added to the queue
                        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Expected datasize: %d",datalen);
                        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:Status data and verification failed! Current data packet size:%d, and verify the calculation data:%d, and verify the read data:%d",datalen,checksum,*((uint16_t*)checksum_ptr));
                        state_data.check_sum = 0;    
                    }
                    checksum = 0;
            }else{//Situations where splicing is required, or the wrong data structure causes the length to be smaller than the expected value
                //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Obtain incomplete header data,size:%d",datalen);
                memset(temp_buff,0,sizeof(temp_buff));//Empty temporary storage variables
                memcpy(temp_buff,recv_buff,sizeof(recv_buff));//Temporary storage of incomplete data
                for(int i=0;i< datalen;i++){
		            checksum += *checksum_ptr; 
		            checksum_ptr++;           
                }
                future_recv = RT_PACKAGE_SIZE - datalen;
            }
        }else{
            //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Not obtained head,size:%d",datalen);
            if(datalen == -1){
                int error_code = 0;
                socklen_t len1 = sizeof(error_code);
                getsockopt(_socket_state,SOL_SOCKET,SO_ERROR,&error_code,&len1);//Get error code
                //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Abnormal status data!");    
                //break;//There is no data anymore, exit the loop
            }else{//Perform data stitching
                 for(int i=0;i< datalen-2;i++){//Perform and verify calculations
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                 }
                 if(checksum == *((uint16_t*)checksum_ptr)){//Perform and verify
                    memcpy(&temp_buff[RT_PACKAGE_SIZE-future_recv],recv_buff,sizeof(recv_buff));
                    memcpy(&state_data,temp_buff,sizeof(temp_buff));
                    //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Get spliced data，size: %d",datalen);
                    if(store_buff.size() < 10){
                        store_buff.emplace(state_data);//Put data in the queue
                    }else{//When the queue length is equal to 10, the element at the beginning of the team pops up, and the element is added at the end of the team.
                        store_buff.pop();
                        store_buff.emplace(state_data);
                    }
                }else{//和校验不通过
                    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: splicing status data and verification failed!");
                    state_data.check_sum = 0;    
                }
                future_recv = RT_PACKAGE_SIZE;
            }
            checksum = 0;
        }
    }//end while
    //The following is reading data from the queue
    if(!store_buff.empty()){//If the queue is not empty, then read the data, otherwise skip this time callback
        state_data = store_buff.front();
        store_buff.pop();
        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Status data: %d,%d",state_data.frame_head,(int)state_data.frame_cnt);    
    }
    return state_data;
}


void fairino_robot::write(double cmd[6]){
//Write servo j instruction through xmlrpc library
    if(_control_mode == POSITION_CONTROL_MODE){//Position control mode
        XmlRpc::XmlRpcValue Args, result;
        for(int i=0;i<6;i++){
            Args[0][i] = cmd[i];//Assignment position instruction
        }
        for(int i=0;i<4;i++){
            Args[1][i] = 0.;//Assignment position instruction
        }
        Args[2] = 0.;
        Args[3] = 0.;
        Args[4] = 0.008;//8ms
        Args[5] = 0.;
        Args[6] = 0.;
        try{
            //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"发送servoJ: %f,%f,%f,%f,%f,%f",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);    
            if (_xml_client_ptr->execute("ServoJ", Args, result)){
                if(int(result) != 0){
                    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: Received instruction feedback error code%d",int(result));    
                }
            }else{
                RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: The instruction was not sent successfully");    
            }
        }catch(XmlRpc::XmlRpcException e){
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot: There is an exception in the transmission of instructions!information:%s",e.getMessage().c_str());    
        }
    }else if(_control_mode == TORQUE_CONTROL_MODE){

    }
}


}//end namespace
