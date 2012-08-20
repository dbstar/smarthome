#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <string.h>
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <time.h>
#include <semaphore.h>

#include "common.h"
#include "serial.h"
#include "sqlite.h"
#include "instruction.h"

static int g_serialfd;
static sem_t s_sem_serial;

static int UART0_Open(char* port);
static void UART0_Close(int fd);
static int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
static int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len);
static int UART0_Send(int fd, unsigned char *send_buf,int data_len);

int serial_int(void)
{
	if(-1==sem_init(&s_sem_serial, 0, 1)){
		DEBUG("s_sem_insert_insts init failed\n");
		return -1;
	}
	
#if 0
	g_serialfd = UART0_Open("/dev/ttyUSB0"); //pc虚拟机linux测试，使用usb转串口，不经过开发板，直接发往串口设备
#else
	g_serialfd = UART0_Open("/dev/ttyS1"); //打开串口，返回文件描述符
#endif

	if(g_serialfd<0){
		DEBUG("open serial failed\n");
		return -1;
	}
	if(0 > UART0_Set(g_serialfd,115200,0,8,1,'N')){
		DEBUG("Set Port Exactly failed!\n");
		UART0_Close(g_serialfd);
		g_serialfd = -1;
		return -1;
	}
	
	DEBUG("serial module init success, serialfd=%d\n", g_serialfd);
	return 0;
}

/*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
static int UART0_Open(char* port)
{
	int fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
	if (-1 == fd)
	{
		perror("Can't Open Serial Port");
		DEBUG("strerror(errno): [%d]%s\n", errno,strerror(errno));
		return(-1);
	}
	//恢复串口为阻塞状态                               
	if(fcntl(fd, F_SETFL, 0) < 0)
	{
		DEBUG("fcntl failed!\n");
		return(-1);
	}     
	else
	{
		DEBUG("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	}
#if 0
	//测试是否为终端设备    
	if(0 == isatty(STDIN_FILENO))
	{
		DEBUG("standard input is not a terminal device\n");
		return(-1);
	}
	else
	{
		DEBUG("isatty success!\n");
	}
#endif

	DEBUG("fd = %d\n",fd);
	return fd;
}
/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/

static void UART0_Close(int fd)
{
	if(fd>0){
		close(fd);
		DEBUG("close serial fd %d\n", fd);
	}
	else
		DEBUG("can NOT close fd %d\n", fd);
}

/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
static int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
	int   i;
	int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
	int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};
	
	struct termios options;
	
	/*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
	*/
	if  ( tcgetattr( fd,&options)  !=  0)
	{
		perror("SetupSerial 1");    
		return(-1); 
	}
	
	//设置串口输入波特率和输出波特率
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if  (speed == name_arr[i])
		{             
			cfsetispeed(&options, speed_arr[i]); 
			cfsetospeed(&options, speed_arr[i]);  
		}
	}     
	
	//修改控制模式，保证程序不会占用串口
	options.c_cflag |= CLOCAL;
	//修改控制模式，使得能够从串口中读取输入数据
	options.c_cflag |= CREAD;
	
	//设置数据流控制
	switch(flow_ctrl)
	{
	case 0 ://不使用流控制
		options.c_cflag &= ~CRTSCTS;
		break; 
	case 1 ://使用硬件流控制
		options.c_cflag |= CRTSCTS;
		break;
	case 2 ://使用软件流控制
		options.c_cflag |= IXON | IXOFF | IXANY;
		break;
	}
	//设置数据位
	//屏蔽其他标志位
	options.c_cflag &= ~CSIZE;
	switch (databits)
	{  
	case 5    :
		options.c_cflag |= CS5;
		break;
	case 6    :
		options.c_cflag |= CS6;
		break;
	case 7    :    
		options.c_cflag |= CS7;
		break;
	case 8:    
		options.c_cflag |= CS8;
		break;  
	default:   
		fprintf(stderr,"Unsupported data size\n");
		return (-1); 
	}
	//设置校验位
	switch (parity)
	{  
	case 'n':
	case 'N': //无奇偶校验位。
		options.c_cflag &= ~PARENB; 
		options.c_iflag &= ~INPCK;    
		break; 
	case 'o':  
	case 'O'://设置为奇校验    
		options.c_cflag |= (PARODD | PARENB); 
		options.c_iflag |= INPCK;             
		break; 
	case 'e': 
	case 'E'://设置为偶校验  
		options.c_cflag |= PARENB;       
		options.c_cflag &= ~PARODD;       
		options.c_iflag |= INPCK;      
		break;
	case 's':
	case 'S': //设置为空格 
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break; 
	default:  
		fprintf(stderr,"Unsupported parity\n");    
		return (-1); 
	} 
	// 设置停止位 
	switch (stopbits)
	{  
	case 1:   
		options.c_cflag &= ~CSTOPB; break; 
	case 2:   
		options.c_cflag |= CSTOPB; break;
	default:   
		fprintf(stderr,"Unsupported stop bits\n"); 
		return (-1);
	}
	
	//修改输出模式，原始数据输出
	options.c_oflag &= ~OPOST;
	
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
	//options.c_lflag &= ~(ISIG | ICANON);
	
	cfmakeraw(&options);
	
	//设置等待时间和最小接收字符
	options.c_cc[VTIME] = 1; 
	options.c_cc[VMIN] = 4;
	
	//如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
	tcflush(fd,TCIFLUSH);
	
	//激活配置 (将修改后的termios数据设置到串口中）
	if (tcsetattr(fd,TCSANOW,&options) != 0)  
	{
		perror("com set error!\n");  
		return (-1); 
	}
	return (0); 
}

/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符    
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
static int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len)
{
	int len,fs_sel;
	fd_set fs_read;
	
	struct timeval time;
	
	FD_ZERO(&fs_read);
	FD_SET(fd,&fs_read);
	
	time.tv_sec = 0;
	time.tv_usec = 200000;
	
	//使用select实现串口的多路通信
	fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
	if(fs_sel>0)
	{
		if(FD_ISSET(fd, &fs_read)){
			len = read(fd,rcv_buf,data_len);
			return len;
		}
		else{
			DEBUG("other fd is can be read, but not serial\n");
			return 0;
		}
	}
	else if(0==fs_sel)
	{
		DEBUG("serial read waiting timeout\n");
		return (0);
	}
	else{	//fs_sel < 0
		DEBUG("serial read failed\n");
		return (-1);
	}
}
/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符    
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
static int UART0_Send(int fd, unsigned char *send_buf,int data_len)
{
	int len = 0;
	
	len = write(fd,send_buf,data_len);
	if (len == data_len )
	{
		return len;
	}     
	else   
	{
		tcflush(fd,TCOFLUSH);
		return -1;
	}
}

static time_t s_last_serial_cmd_time = 0;
INSTRUCTION_RESULT_E sendto_serial(unsigned char *buf, unsigned int len)
{
	if(NULL==buf || len<=0){
		DEBUG("params some error\n");
		return -1;
	}

	sem_wait(&s_sem_serial);
	
	/*
	间隔1s，确保串口操作安全
	*/
	if( (time(NULL)-s_last_serial_cmd_time) < 1 ){
		DEBUG("too pressing cmd, sleep 1s to relex serial\n");
		sleep(1);
	}
	
	int i = 0;
	DEBUG("len %d chars\n", len);
	printf("----------------------------------------\n");
	for(i=0;i<len;i++)
		printf(" %02x", buf[i]);
	printf("\n----------------------------------------\n");
	
	int ret = UART0_Send(g_serialfd, buf, len);
	
	s_last_serial_cmd_time = time(NULL);
	sem_post(&s_sem_serial);

	if(ret==len)
		return 0;
	else
		return -1;
}

int recvfrom_serial(unsigned char *buf, unsigned int buf_size)
{
	if(NULL==buf || buf_size<=0){
		DEBUG("params some error\n");
		return -1;
	}
	
	int ret = -1;
	sem_wait(&s_sem_serial);
	usleep(200000);
	int has_read = 0;
	unsigned char *p_readbuf = buf;
	int i = 0;
	int len = 0;
	for(i=0;i<SERIAL_RECV_RETRY;i++){
		len = UART0_Recv(g_serialfd, p_readbuf+has_read, buf_size-1-has_read);
		if(len > 0)
		{
			DEBUG("read %d bytes for this time,(",len);
			int j = 0;
			for(j=0;j<len;j++)
				printf(" %02x", *(p_readbuf+has_read+j));
				
			has_read += len;
			
			printf("), and total read %d bytes\n", has_read);
			
			if(has_read>12 && 0x16==buf[has_read-1])	// 大于12就算是读完毕了
				break;
		}
		else if(0==len)
		{
			DEBUG("cannot receive data for %d try\n", i+1);
		}
		else{
			break;
		}
	}
	DEBUG("has read total %d bytes\n", has_read);
	if(has_read>12)	// 大于12就算是读完毕了
		ret = has_read;
	else
		ret = -1;
		
	printf("===========================================\n");
	for(i=0;i<has_read;i++)
		printf(" %02x", buf[i]);
	printf("\n===========================================\n");
	
	tcflush(g_serialfd, TCIOFLUSH);
	sem_post(&s_sem_serial);
	
	return ret;
}


#ifdef TEST_SERIAL_CMD_ONLY_ONCE
void serial_fd_close(void)
{
	UART0_Close(g_serialfd);
	g_serialfd = -1;
}
#endif


/***sendSerial() brief send data to serial port
 * param length[in], file descriptor of serial port to be send data
 *
 * retval int, return value--true if true,false failed.
 ***/
 /*
BOOL_E sendSerial(int length)
{
	int l_fd=-1;								///temporary file descriptor

	if(write(l_fd,g_dataTrans.st_sendSerial,length)!=length)
	{
		ERROROUT("write() error!");
		tcflush(l_fd,TCOFLUSH);						///???bug(maybe cause some problem)???flushes data written but not transmitted.
		return false;
	}


	return true;
}*/

