#include <iostream>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sys/select.h>
#include <serial/recv.h>
#include <serial/driver.h>
#include "ros/ros.h"
#include </home/snd/catkin_ws/src/serial/src/serial.h>


char tmp[1];
char tmp1[8];

uint16 pwm = 4000;

int flag = 0;
int flag1 = 0;
int flag2 = 0;

int32 Lposition_count = 0;
int32 Rposition_count = 0;

/**
	控制流程：
	1.发送复位指令
	2.等待500ms
	3.发送模式选择指令，使驱动器进入某种模式
	4.等待500ms
	5.在已经进入的模式下发送数据指令。(周期性发送本条指令，间隔最短为1ms，推荐间隔10ms)
**/
static void sleep_ms(unsigned int secs)
{
    struct timeval tval;
    tval.tv_sec=secs/1000;
    tval.tv_usec=(secs*1000)%1000000;
    select(0,NULL,NULL,NULL,&tval);

}


void driverdataCallback(const serial::driver::ConstPtr& msg)
{
    if((msg->position_data[4]-Lposition_count >= 0)&&(msg->position_data[4]-Lposition_count<=30))
        flag1 = 1;
    if((msg->position_data[5]-Rposition_count >= 0)&&(msg->position_data[5]-Rposition_count<=30))
        flag2 = 1;

    if((flag1 == 1)&&(flag2 == 1))
        flag = 1;    
}

/**
    sp0 :   左前轮
    sp1 :   右前轮
    sp2 :   左后轮
    sp3 :   右后轮
    sp4 :   左转向
    sp5 :   左转向
**/
void recvdataCallback(const serial::recv::ConstPtr& msg)
{
    flag = 0;
    flag1 = 0;
    flag2 = 0;
    //左前轮
    io_service iosev0;
    serial_port sp0(iosev0,"/dev/ttyS5");
    sp0.set_option(serial_port::baud_rate(115200));
    sp0.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp0.set_option(serial_port::parity(serial_port::parity::none));
    sp0.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp0.set_option(serial_port::character_size(8));


    //右前轮
    io_service iosev1;
    serial_port sp1(iosev1,"/dev/ttyS5");
    sp1.set_option(serial_port::baud_rate(115200));
    sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp1.set_option(serial_port::parity(serial_port::parity::none));
    sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp1.set_option(serial_port::character_size(8));

    //左后轮
    io_service iosev2;
    serial_port sp2(iosev2,"/dev/ttyS5");
    sp2.set_option(serial_port::baud_rate(115200));
    sp2.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp2.set_option(serial_port::parity(serial_port::parity::none));
    sp2.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp2.set_option(serial_port::character_size(8));

    //右后轮
    io_service iosev3;
    serial_port sp3(iosev3,"/dev/ttyS5");
    sp3.set_option(serial_port::baud_rate(115200));
    sp3.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp3.set_option(serial_port::parity(serial_port::parity::none));
    sp3.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp3.set_option(serial_port::character_size(8));

    //左转向
    io_service iosev4;
    serial_port sp4(iosev4,"/dev/ttyS5");
    sp4.set_option(serial_port::baud_rate(115200));
    sp4.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp4.set_option(serial_port::parity(serial_port::parity::none));
    sp4.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp4.set_option(serial_port::character_size(8));

    //右转向
    io_service iosev5;
    serial_port sp5(iosev5,"/dev/ttyS5");
    sp5.set_option(serial_port::baud_rate(115200));
    sp5.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp5.set_option(serial_port::parity(serial_port::parity::none));
    sp5.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp5.set_option(serial_port::character_size(8));


    cout<<"*********"<<endl;
    uint16 temp_velocity = 0;    //目标速度
    int32 temp_position = 0;   //目标位置
    

    velocity_position_mode[2]=(unsigned char)((pwm>>8)&0xff);
    velocity_position_mode[3]=(unsigned char)((pwm)&0xff);

    if((0 != (msg->temp_position[2]))||(0 != (msg->temp_position[3])))
    {
        LTvelocity_position_mode[4]=(unsigned char)(((msg->temp_velocity[2])>>8)&0xff);
        LTvelocity_position_mode[5]=(unsigned char)((msg->temp_velocity[2])&0xff);
        LTvelocity_position_mode[6]=(unsigned char)(((msg->temp_position[2])>>24)&0xff);
        LTvelocity_position_mode[7]=(unsigned char)(((msg->temp_position[2])>>16)&0xff);
        LTvelocity_position_mode[8]=(unsigned char)(((msg->temp_position[2])>>8)&0xff);
        LTvelocity_position_mode[9]=(unsigned char)((msg->temp_position[2])&0xff);

        RTvelocity_position_mode[4]=(unsigned char)(((msg->temp_velocity[3])>>8)&0xff);
        RTvelocity_position_mode[5]=(unsigned char)((msg->temp_velocity[3])&0xff);
        RTvelocity_position_mode[6]=(unsigned char)(((msg->temp_position[3])>>24)&0xff);
        RTvelocity_position_mode[7]=(unsigned char)(((msg->temp_position[3])>>16)&0xff);
        RTvelocity_position_mode[8]=(unsigned char)(((msg->temp_position[3])>>8)&0xff);
        RTvelocity_position_mode[9]=(unsigned char)((msg->temp_position[3])&0xff);
        
        write(sp4,buffer(LTvelocity_position_mode,10));
        write(sp5,buffer(RTvelocity_position_mode,10));
    }

    if((0 != (msg->temp_position[0]))||(0 != (msg->temp_position[1])))
    {
        Lvelocity_position_mode[4]=(unsigned char)(((msg->temp_velocity[0])>>8)&0xff);
        Lvelocity_position_mode[5]=(unsigned char)((msg->temp_velocity[0])&0xff);
        Lvelocity_position_mode[6]=(unsigned char)(((msg->temp_position[0])>>24)&0xff);
        Lvelocity_position_mode[7]=(unsigned char)(((msg->temp_position[0])>>16)&0xff);
        Lvelocity_position_mode[8]=(unsigned char)(((msg->temp_position[0])>>8)&0xff);
        Lvelocity_position_mode[9]=(unsigned char)((msg->temp_position[0])&0xff);

        Rvelocity_position_mode[4]=(unsigned char)(((msg->temp_velocity[1])>>8)&0xff);
        Rvelocity_position_mode[5]=(unsigned char)((msg->temp_velocity[1])&0xff);
        Rvelocity_position_mode[6]=(unsigned char)(((msg->temp_position[1])>>24)&0xff);
        Rvelocity_position_mode[7]=(unsigned char)(((msg->temp_position[1])>>16)&0xff);
        Rvelocity_position_mode[8]=(unsigned char)(((msg->temp_position[1])>>8)&0xff);
        Rvelocity_position_mode[9]=(unsigned char)((msg->temp_position[1])&0xff);
        
        write(sp0,buffer(Lvelocity_position_mode,10));
        write(sp2,buffer(Lvelocity_position_mode,10));
        write(sp1,buffer(Rvelocity_position_mode,10));
        write(sp3,buffer(Rvelocity_position_mode,10));
    }
    
}

/**
    速度位置模式Demo
**/
int main(int argc,char **argv)
{
    ros::init(argc,argv,"serial");

    io_service iosev;

    serial_port sp(iosev,"/dev/ttyS5");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));


    //初始化命令
    write(sp,buffer(init_buf,10));
    std::cout<<"Send init_buf over."<<std::endl;
    sleep_ms(500);

    mode_choice[2] = 0x05;  //选择速度位置模式
    write(sp,buffer(mode_choice,10));
    sleep_ms(500);

    configure_data[2] = 0x0A;   //反馈周期为10ms
    write(sp,buffer(configure_data,10));

    sleep_ms(500);

    cout<<"##########"<<endl;
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("recv_data",1000,recvdataCallback);
    ros::spin();

        /**
    temp_velocity = 400;
    temp_position = 80000;
    temp_pwm = 4000;

	velocity_position_mode[2]=(unsigned char)((temp_pwm>>8)&0xff);
	velocity_position_mode[3]=(unsigned char)((temp_pwm)&0xff);
	velocity_position_mode[4]=(unsigned char)((temp_velocity>>8)&0xff);
	velocity_position_mode[5]=(unsigned char)((temp_velocity)&0xff);
	velocity_position_mode[6]=(unsigned char)((temp_position>>24)&0xff);
	velocity_position_mode[7]=(unsigned char)((temp_position>>16)&0xff);
	velocity_position_mode[8]=(unsigned char)((temp_position>>8)&0xff);
	velocity_position_mode[9]=(unsigned char)(temp_position&0xff);
    write(sp,buffer(velocity_position_mode,10));
**/
    return 0;
}
