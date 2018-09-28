#include <iostream>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sys/select.h>
#include </home/snd/catkin_ws/src/serial/src/serial.h>

int main(int argc,char** argv)
{
    io_service iosev;

    serial_port sp(iosev,"/dev/ttyS5");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    write(sp,buffer(init_buf,10));
    std::cout<<"Send init_buf over."<<std::endl;

}
