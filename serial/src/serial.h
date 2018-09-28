#ifndef __SERIAL_H__
#define __SERIAL_H__

using namespace std;
using namespace boost::asio;

typedef signed short int16;
typedef unsigned short uint16;
typedef signed int int32;
typedef unsigned int uint32;


uint16 temp_pwm = 0; //目标PWM
int16 temp_current = 0; //目标电流

int16 now_current = 0;  //当前电流
int16 now_velocity = 0; //当前速度
int32 now_position = 0; //当前位置

//复位指令
char init_buf[10] = {0x23,0x00,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
//回传指令
char buf[10] = {0x23,0x0F,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};

/**	
	模式选择：
		mdoel_choice[2]

	开环模式	0x01
	电流模式	0x02
	速度模式	0x03
	位置模式	0x04
	速度位置模式	0x05
	电流速度模式	0x06
	电流位置模式	0x07
	电流速度位置模式0x08
**/
char mode_choice[10]= {0x23,0x01,0x00,0x55,0x55,0x55,0x55,0x55,0x55,0x55};

/**
	开环模式：
		open_loop[2] open_loop[3]

	open_loop[2] = (unsigned char)((temp_pwm>>8)&0xff)
	open_loop[3] = (unsigned char)((temp_pwm)&0xff)
	temp_pwm取值范围为：-5000～+5000
		temp+pwm为“-”时：反转
			为“+”时：正转
**/
char open_loop[10] = {0x23,0x02,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55};

/**
	电流模式：
		current_mode[2] current_mode[3] current_mode[4] current_mode[5]		
	current_mode[2] = (unsigned char)((temp_pwm>>8)&0xff)
	current_mode[3] = (unsigned char)(temp_pwm&0xff)
	current_mode[4] = (unsigned char)((temp_current>>8)&0xff)
	current_mode[5] = (unsigned char)(temp_current&0xff)

	temp_pwm取值范围为：0～+5000
	temp_current取值范围为：-32768～+32767 单位为mA
		temp_current为“-”时：电机反转
			    为“+”时：电机正转
**/
char current_mode[10] = {0x23,0x03,0x00,0x00,0x00,0x00,0x55,0x55,0x55,0x55};

/**
	速度模式：
		velocity_mode[2] velocity_mode[3] velocity_mode[4] velocity_mode[5]
	
	velocity_mode[2] = (unsigned char)((temp_pwm>>8)&0xff)
	velocity_mode[3] = (unsigned char)(temp_pwm&0xff)
	velocity_mode[4] = (unsigned char)((temp_velocity>>8)&0xff)
	velocity_mode[5] = (unsigned char)(temp_velocity&0xff)
	
	temp_pwm取值范围为：0～+5000
	temp_velocity取值范围为：-32768～+32767 单位为RPM
		temp_velocity为“-”时：电机反转
			     为“+”时：电机正传
**/
char velocity_mode[10] = {0x23,0x05,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00};

/**
	位置模式
	position_mode[2] position_mode[3] position_mode[6] position_mode[7] position_mode[8] position_mode[9]

	position_mode[2] = (unsigned char)((temp_pwm>>8)&0xff)
	position_mode[3] = (unsigned char)(temp_pwm&0xff)
	position_mode[6] = (unsigned char)((temp_position>>24)&0xff)
	position_mode[7] = (unsigned char)((temp_position>>16)&0xff)
	position_mode[8] = (unsigned char)((temp_position>>8)&0xff)
	position_mode[9] = (unsigned char)(temp_position&0xff)

	temp_pwm取值范围：0～+5000
	temp_position取值范围：-2147483648～+2147483647 单位为qc
		temp_position为“-”时：电机反转
			     为“+”时：电机正转
	temp_position为1000时，正转1圈吧。
**/
char position_mode[10] = {0x23,0x05,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00};

/**
	速度位置模式：
	velocity_position_mode[2]~velocity_position_mode[9]
	发送间隔不能小于2ms，建议10ms
	
	velocity_position_mode[2]=(unsigned char)((temp_pwm>>8)&0xff)
	velocity_position_mode[3]=(unsigned char)((temp_pwm)&0xff)
	velocity_position_mode[4]=(unsigned char)((temp_velocity>>8)&0xff)
	velocity_position_mode[5]=(unsigned char)((temp_velocity)&0xff)
	velocity_position_mode[6]=(unsigned char)((temp_position>>24)&0xff)
	velocity_position_mode[7]=(unsigned char)((temp_position>>16)&0xff)
	velocity_position_mode[8]=(unsigned char)((temp_position>>8)&0xff)
	velocity_position_mode[9]=(unsigned char)(temp_position&0xff)
	
	temp_pwm取值范围：0～+5000
	temp_velocity取值范围：0～+32767	单位为RPM
	temp_position取值范围：-2147483648～+2147483647单位为qc
	
	电动机方向由temp_position决定
**/
char velocity_position_mode[10] = {0x23,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char Lvelocity_position_mode[10] = {0x23,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char Rvelocity_position_mode[10] = {0x23,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

char LTvelocity_position_mode[10] = {0x23,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char RTvelocity_position_mode[10] = {0x23,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/**
	电流速度模式：
	current_velocity_mode[2]~current_velocity_mode[5]
	
	current_velocity_mode[2] = (unsigned char)((temp_current>>8)&0xff)
	current_velocity_mode[3] = (unsigned char)((temp_current)&0xff)
	current_velocity_mode[4] = (unsigned char)((temp_velocity>>8)&0xff)
	current_velocity_mode[5] = (unsigned char)((temp_velocity)&0xff)

	temp_current取值范围：0～+32767		单位为mA
	temp_velocity取值范围：-32768~+32767	单位为RPM
	
	电机方向由temp_velocity决定
**/

/**
	电流位置模式：
	current_position_mode[2] current_position_mode[3] current_position_mode[6]~current_position_mode[9]

	current_position_mode[2] = (unsigned char)((temp_current>>8)&0xff)
	current_position_mode[3] = (unsigned char)((temp_current)&0xff)
	current_position_mode[6] = (unsigned char)((temp_position>>24)&0xff)
	current_position_mode[7] = (unsigned char)((temp_position>>16)&0xff)
	current_position_mode[8] = (unsigned char)((temp_position>>8)&0xff)
	current_position_mode[9] = (unsigned char)((temp_position)&0xff)

	temp_current取值范围为：0～+32767	单位为mA

	temp_position取值范围为：-2147483648～+2147483647	单位为qc

	电机方向由temp_velocity决定
**/
char current_position_mode[10] = {0x23,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/**
	配置指令：
	1.可以让驱动器周期性发送实时电流、速度、位置等信息
	2.可以决定CTRL1和CTRL2端口作为左右限位功能后，以某个固定时间间隔对外发送2个端口电平状态
	
	configure_data[2] : 发送电流速度位置的周期，0x00为不发送，单位为ms
	configure_data[3] : 发送CTL端口电平状态的周期，0x00为不发送，单位为ms
**/
char configure_data[10] = {0x23,0x0A,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55};

/**
	数据反馈：
	驱动器对外发送电流、速度、位置等信息
	
	data_feedback[2]~data_feedback[9]
	
	data_feedback[2] = (unsigned char)((real_current>>8)&0xff) 
	data_feedback[3] = (unsigned char)((real_current)&0xff) 
	data_feedback[4] = (unsigned char)((real_velocity>>8)&0xff) 
	data_feedback[5] = (unsigned char)((real_velocity)&0xff) 
	data_feedback[6] = (unsigned char)((real_position>>24)&0xff) 
	data_feedback[7] = (unsigned char)((real_position>>16)&0xff) 
	data_feedback[8] = (unsigned char)((real_position>>8)&0xff) 
	data_feedback[9] = (unsigned char)((real_position)&0xff) 

	real_current 单位为mA
	real_velocity 单位为RPM
	real_velocity 单位为qc
**/
//char data_feedback[10] = {0x23,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char data_feedback[10];

/**
	左右限位定时反馈：

	1.通过RS232配置指令，对CTL，CTL2端口电平发送，设置了不为0的周期
	2.驱动器进入了任意一种运动模式

	limit_feedback[2] limit_feedback[3]
**/
char limit_feedback[10] = {0x23,0x0C,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55};

char recv_buf[10];
//char buf[10] =      {0x23,0x0F,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};


#endif
