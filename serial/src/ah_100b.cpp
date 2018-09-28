#include <iostream>
#include <iomanip>
#include <string>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sys/select.h>
#include <iomanip>
#include <serial/data.h>
#include <serial/recv.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include </home/snd/catkin_ws/src/serial/src/ah100b.h>


using namespace std;
using namespace boost::asio;


#define DL_CLASS_MINIAHRS   0x0F    //姿态数据类型码
#define DL_MINIAHRS_ATTITUDE_ADN_SENSORS    0x01    //姿态数据ID码
#define DATA_LINK_MAX_LENGTH    200 //datalink 数据包payload最长值

//DL Read Return Err
#define DL_NO_ERR   0X00
#define DL_UNKNOW_MESSAGE   0x01
#define DL_CHECKSUM_ERR 0x02
#define DL_PALYOAD_LENGTH_ERR   0x04

#pragma pack(1)
typedef struct
{
    unsigned char Flags;    //状态位，暂时不用处理
    float Euler[3];         //三个欧拉角
    float Gyro[3];          //经校准后，去bias后的角速度值，度制
    float Acc[3];           //经校准后，加速度单位g
    float Mag[3];           //经校准后的指南针值，无量纲
}sAHRSData;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char Header1;  //'T'
    unsigned char Header2;  //'M'
    unsigned char Class;
    unsigned char ID;
    unsigned short Length;
    unsigned char Payload[DATA_LINK_MAX_LENGTH];
    unsigned char CheckSum[2];
}sDataLink;
#pragma pack()


sAHRSData AHRSData;     //AHRS数据全局变量
sDataLink DataLink;     //接受数据结构

char buf1[57]="";
char buf2[1];

unsigned short CRC16(unsigned char *p,unsigned short length)
{
    unsigned short checksum = 0;
    for(;length>0;length--)
    {
        checksum = (checksum >> 8)^CRC16Table[(checksum&0xFF)^*p];
        p++;
    }
    return checksum;
}


int main(int argc,char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<serial::data>("ah100b_data",1000);

    ros::NodeHandle n1;
    ros::Publisher chatter_pub1 = n1.advertise<serial::recv>("recv_data",1000);

    io_service iosev;
    serial_port sp(iosev,"/dev/ttyUSB0");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    unsigned char *INT8UP;
    unsigned short CheckSum;

    //ros::init(argc,argv,"talker");

    serial::data send_msg;
    

    while(1)
    {
        //std::stringstream ss;
        float ss[3];
        int i = 0;
        read(sp,buffer(buf2));
        if(buf2[0] == 'T')
        {
            read(sp,buffer(buf2));
            if(buf2[0] == 'M')
            {
                read(sp,buffer(buf1));
                DataLink.Class = buf1[0]; //0X0F
                DataLink.ID = buf1[1];    //0X01
                DataLink.Length = buf1[2]|(((unsigned short)(buf1[3]))<<8);
                DataLink.CheckSum[0] = buf1[53];
                DataLink.CheckSum[1] = buf1[54];

                INT8UP = (unsigned char*)&buf1[0];
                if((DataLink.Length+4) > 53)
                    continue;
                CheckSum = CRC16(INT8UP,DataLink.Length+4);

                if((CheckSum>>8 == DataLink.CheckSum[1]) && ((CheckSum&0x00ff)==DataLink.CheckSum[0]))
                {
                    for(i = 0;i<48;i++)
                    {
                        DataLink.Payload[i] = buf1[i+4];
                    }
                }
                INT8UP = (unsigned char*)&AHRSData.Flags;
                for(i=0;i<DataLink.Length-1;i++)
                {
                    (*INT8UP) = DataLink.Payload[i];
                    INT8UP++;
                }

                send_msg.Euler[0] = AHRSData.Euler[0];
                send_msg.Euler[1] = AHRSData.Euler[1];
                send_msg.Euler[2] = AHRSData.Euler[2];
                send_msg.Gyro[0] = AHRSData.Gyro[0];
                send_msg.Gyro[1] = AHRSData.Gyro[1];
                send_msg.Gyro[2] = AHRSData.Gyro[2];
                send_msg.Acc[0] = AHRSData.Acc[0];
                send_msg.Acc[1] = AHRSData.Acc[1];
                send_msg.Acc[2] = AHRSData.Acc[2];
                send_msg.Mag[0] = AHRSData.Mag[0];
                send_msg.Mag[1] = AHRSData.Mag[1];
                send_msg.Mag[2] = AHRSData.Mag[2];
                chatter_pub.publish(send_msg);
                ros::spinOnce();
                std::cout<<"***************"<<std::endl;
            }
        }
    }
    return 0;
}
