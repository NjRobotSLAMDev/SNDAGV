#include <iostream>
#include "ros/ros.h"
#include <serial/driver.h>
#include <string>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <sys/select.h>
#include </home/snd/catkin_ws/src/serial/src/serial.h>


char tmp[1];
unsigned char tmp1[8];

static void sleep_ms(unsigned int secs)
{
    struct timeval tval;
    tval.tv_sec=secs/1000;
    tval.tv_usec=(secs*1000)%1000000;
    select(0,NULL,NULL,NULL,&tval);

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"read_serial");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<serial::driver>("driver_data",1000);
    serial::driver driver_msg;
    
    io_service iosev;
    stringstream ss;
    serial_port sp(iosev,"/dev/ttyS5");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
    while(ros::ok())
    {
        sleep_ms(5);
        read(sp,buffer(tmp));
        if(tmp[0] == 0x23)
        {
            read(sp,buffer(tmp));
            if(tmp[0] == 0x0b)
            {
                read(sp,buffer(tmp1));
                now_current = (tmp1[0]<<8)|tmp1[1];
                cout<<"now_current:"<<now_current<<" mA"<<endl;
                now_velocity = (tmp1[2]<<8)|tmp1[3];
                cout<<"now_velocity:"<<now_velocity<<" RPM"<<endl;
                now_position = (tmp1[4]<<24)|(tmp1[5]<<16)|(tmp1[6]<<8)|(tmp1[7]);
                cout<<"now_position:"<<now_position<<" qc"<<endl;
                
                driver_msg.velocity_data[0] = now_velocity;
                driver_msg.current_data[0] = now_current;
                driver_msg.position_data[0] = now_position;
                chatter_pub.publish(driver_msg);
                ros::spinOnce();

                std::cout<<"---------------"<<std::endl;
            }
        }
    }
}
