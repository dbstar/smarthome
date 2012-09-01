#include <stdio.h>      /*��׼�����������*/
#include <stdlib.h>     /*��׼�����ⶨ��*/
#include <string.h>
#include <unistd.h>     /*Unix ��׼��������*/
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h>      /*�ļ����ƶ���*/
#include <termios.h>    /*PPSIX �ն˿��ƶ���*/
#include <errno.h>      /*����Ŷ���*/
#include <time.h>
#include <semaphore.h>

#include "common.h"
#include "serial.h"
#include "sqlite.h"
#include "instruction.h"

static int g_serialfd;
static sem_t s_sem_serial;

static time_t s_last_serial_cmd_time = 0;
static int s_serial_failed_count = 0;

static int UART0_Open(char* port);
static void UART0_Close(int fd);
static int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
static int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len);
static int UART0_Send(int fd, unsigned char *send_buf,int data_len);

static int serial_reset()
{
	sem_wait(&s_sem_serial);
	
	int ret = -1;
	
	if(g_serialfd>0)
		serial_fd_close();
	
	usleep(500000);
	
#if 0
	g_serialfd = UART0_Open("/dev/ttyUSB0"); //pc�����linux���ԣ�ʹ��usbת���ڣ������������壬ֱ�ӷ��������豸
#else
	g_serialfd = UART0_Open("/dev/ttyS1"); //�򿪴��ڣ������ļ�������
#endif

	if(g_serialfd<0){
		DEBUG("open serial failed\n");
		ret = -1;
	}
	else{
		if(0 > UART0_Set(g_serialfd,115200,0,8,1,'N')){
			DEBUG("Set Port Exactly failed!\n");
			UART0_Close(g_serialfd);
			g_serialfd = -1;
			ret = -1;
		}
		else{
			s_serial_failed_count = 0;
			DEBUG("serial module reset success, serialfd=%d\n", g_serialfd);
			ret = 0;
		}
	}
	sem_post(&s_sem_serial);
	
	return ret;
}

int serial_int(void)
{
	if(-1==sem_init(&s_sem_serial, 0, 1)){
		DEBUG("s_sem_insert_insts init failed\n");
		return -1;
	}
	
	g_serialfd = -1;
	return serial_reset();
}

/*******************************************************************
* ���ƣ�                  UART0_Open
* ���ܣ�                �򿪴��ڲ����ش����豸�ļ�����
* ��ڲ�����        fd    :�ļ�������     port :���ں�(ttyS0,ttyS1,ttyS2)
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
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
	//�ָ�����Ϊ����״̬                               
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
	//�����Ƿ�Ϊ�ն��豸    
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
* ���ƣ�                UART0_Close
* ���ܣ�                �رմ��ڲ����ش����豸�ļ�����
* ��ڲ�����        fd    :�ļ�������     port :���ں�(ttyS0,ttyS1,ttyS2)
* ���ڲ�����        void
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
* ���ƣ�                UART0_Set
* ���ܣ�                ���ô�������λ��ֹͣλ��Ч��λ
* ��ڲ�����        fd        �����ļ�������
*                              speed     �����ٶ�
*                              flow_ctrl   ����������
*                           databits   ����λ   ȡֵΪ 7 ����8
*                           stopbits   ֹͣλ   ȡֵΪ 1 ����2
*                           parity     Ч������ ȡֵΪN,E,O,,S
*���ڲ�����          ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
static int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
	int   i;
	int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
	int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};
	
	struct termios options;
	
	/*tcgetattr(fd,&options)�õ���fdָ��������ز������������Ǳ�����options,�ú��������Բ��������Ƿ���ȷ���ô����Ƿ���õȡ������óɹ�����������ֵΪ0��������ʧ�ܣ���������ֵΪ1.
	*/
	if  ( tcgetattr( fd,&options)  !=  0)
	{
		perror("SetupSerial 1");    
		return(-1); 
	}
	
	//���ô������벨���ʺ����������
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if  (speed == name_arr[i])
		{             
			cfsetispeed(&options, speed_arr[i]); 
			cfsetospeed(&options, speed_arr[i]);  
		}
	}     
	
	//�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���
	options.c_cflag |= CLOCAL;
	//�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ����ж�ȡ��������
	options.c_cflag |= CREAD;
	
	//��������������
	switch(flow_ctrl)
	{
	case 0 ://��ʹ��������
		options.c_cflag &= ~CRTSCTS;
		break; 
	case 1 ://ʹ��Ӳ��������
		options.c_cflag |= CRTSCTS;
		break;
	case 2 ://ʹ�����������
		options.c_cflag |= IXON | IXOFF | IXANY;
		break;
	}
	//��������λ
	//����������־λ
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
	//����У��λ
	switch (parity)
	{  
	case 'n':
	case 'N': //����żУ��λ��
		options.c_cflag &= ~PARENB; 
		options.c_iflag &= ~INPCK;    
		break; 
	case 'o':  
	case 'O'://����Ϊ��У��    
		options.c_cflag |= (PARODD | PARENB); 
		options.c_iflag |= INPCK;             
		break; 
	case 'e': 
	case 'E'://����ΪżУ��  
		options.c_cflag |= PARENB;       
		options.c_cflag &= ~PARODD;       
		options.c_iflag |= INPCK;      
		break;
	case 's':
	case 'S': //����Ϊ�ո� 
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break; 
	default:  
		fprintf(stderr,"Unsupported parity\n");    
		return (-1); 
	} 
	// ����ֹͣλ 
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
	
	//�޸����ģʽ��ԭʼ�������
	options.c_oflag &= ~OPOST;
	
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//�Ҽӵ�
	//options.c_lflag &= ~(ISIG | ICANON);
	
	cfmakeraw(&options);
	
	//���õȴ�ʱ�����С�����ַ�
	options.c_cc[VTIME] = 1; 
	options.c_cc[VMIN] = 4;
	
	//�����������������������ݣ����ǲ��ٶ�ȡ ˢ���յ������ݵ��ǲ���
	tcflush(fd,TCIFLUSH);
	
	//�������� (���޸ĺ��termios�������õ������У�
	if (tcsetattr(fd,TCSANOW,&options) != 0)  
	{
		perror("com set error!\n");  
		return (-1); 
	}
	return (0); 
}

/*******************************************************************
* ���ƣ�                  UART0_Recv
* ���ܣ�                ���մ�������
* ��ڲ�����        fd                  :�ļ�������    
*                              rcv_buf     :���մ��������ݴ���rcv_buf��������
*                              data_len    :һ֡���ݵĳ���
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
*******************************************************************/
static int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len)
{
	int len,fs_sel;
	fd_set fs_read;
	
	struct timeval time;
	
	FD_ZERO(&fs_read);
	FD_SET(fd,&fs_read);
	
	time.tv_sec = 0;
	time.tv_usec = 100000;
	
	//ʹ��selectʵ�ִ��ڵĶ�·ͨ��
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
* ���ƣ�                  UART0_Send
* ���ܣ�                ��������
* ��ڲ�����        fd                  :�ļ�������    
*                              send_buf    :��Ŵ��ڷ�������
*                              data_len    :һ֡���ݵĸ���
* ���ڲ�����        ��ȷ����Ϊ1�����󷵻�Ϊ0
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

static int sendto_serial(unsigned char *buf, unsigned int len)
{
	if(NULL==buf || len<=0){
		DEBUG("params some error\n");
		return -1;
	}
	
	int i = 0;
	printf("---------------------------------------- %d chars\n", len);
	for(i=0;i<len;i++)
		printf(" %02x", buf[i]);
	printf("\n----------------------------------------\n");
	
	int ret = UART0_Send(g_serialfd, buf, len);
	
	if(ret==len)
		return 0;
	else
		return -1;
}

/*
ģ��strstr����һ������в�����һ�������������ִ������ַ����п��ܴ���'\0'�����Բ���ֱ����strstr
Ŀǰ���ü򵥵�ƥ�䣬û���Ż������Ҫ��ѯ�Ĵ��Ƚϳ�����˷�������ȡ��
����ֵΪƥ�䴮son_buf�Ŀ�ͷ��dad_buf�е�index����0��ʼ��
*/
static int ascinasc(unsigned char *dad_buf, unsigned int dad_len, unsigned char *son_buf, unsigned int son_len)
{
	if(0==dad_len || 0==son_len)
		return -1;
	
	int i = 0, j = 0;
	for(i=0;i<(dad_len-son_len);i++){
		j = 0;
		for(j=0;j<son_len;j++){
			if(dad_buf[i+j]!=son_buf[j]){
				//DEBUG("dad_buf[%d]=0x%02x, son_buf[%d]=0x%02x	break\n", i+j, dad_buf[i+j], j, son_buf[j]);
				break;
			}
			else
				;//DEBUG("dad_buf[%d]=0x%02x, son_buf[%d]=0x%02x	==\n", i+j, dad_buf[i+j], j, son_buf[j]);
				
		}
		if(j==son_len)
			return i;
	}
	
	return -1;
}

static int recvfrom_serial(unsigned char *buf, unsigned int buf_size, unsigned char *distinguish_cmd)
{
	if(NULL==buf || buf_size<=0){
		DEBUG("params some error\n");
		return -1;
	}
	
	int ret = -1;
	int has_read = 0;
	unsigned char *p_readbuf = buf;
	int i = 0;
	int len = 0;
	int catch_valid_cmd_head_flag = 0;
	
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
		}
		else if(0==len)
		{
			DEBUG("cannot receive data for %d try\n", i+1);
		}
		else{
			break;
		}
		
//	����Ϊ��̬��⣬�����������ѭ������ߴ�����Ӧ�ٶȡ�
		if(0==catch_valid_cmd_head_flag){
			if(has_read>SERIAL_RESPONSE_LEN_MIN){
				int p = ascinasc(buf, has_read, distinguish_cmd, 8);
				if(p>=0){
					catch_valid_cmd_head_flag = 1;
				}
			}
		}
		else
		{
			if(catch_valid_cmd_head_flag>=3)
				break;
				
			catch_valid_cmd_head_flag ++;
		}
		
		DEBUG("catch_valid_cmd_head_flag=%d, has_read=%d\n", catch_valid_cmd_head_flag, has_read);
		if(has_read>64)	// ����̫��Ĵ���û�����壬�������Ƚ��յ�һЩ����Ĵ��ڷ����������Ϊ64Ҳ���Եõ�3�顣
			break;
	}
	DEBUG("has read total %d bytes\n", has_read);
	if(has_read>SERIAL_RESPONSE_LEN_MIN)
		ret = has_read;
	else
		ret = -1;
		
	tcflush(g_serialfd, TCIOFLUSH);
	
	return ret;
}

/*
������������������ش��Ŀ�ͷһ���ǷǷ�ֵ��
��1���Լ̵����պϲ���Ϊ��
���ͣ�68 20 11 12 21 06 36 68 04 09 56 16 33 33 33 33 44 44 44 81 16
���أ�56 7b 16 68 20 11 06 02 41 56 68 81 06 43 c3 33 33 33 33 f9 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 06 00 41 56 68 81 06 43 c3 33 3b 33 33 ff 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 06 00 41 56 68 81 06 43 c3 33 33 33 33 f7 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16 68 20 11
ʵ�������õ��ǣ�68 20 11 12 21 06 36 68 c5 03 e9 04 56 7b 16����ʾͨ�Ŵ���

��2����֤�����Ƿ����
���ͣ�68 20 11 12 21 06 36 68 07 00 77 16
���أ�68 99 99 99 99 99 99 68 c5 03 e9 99 99 49 16 68 20 11 12 21 06 36 68 87 06 53 44 45 54 39 69 cf 16
ʵ�������õ��ǣ�68 20 11 12 21 06 36 68 87 06 53 44 45 54 39 69 cf 16
��ȫ��ȷ���ǣ�	68 20 11 12 21 06 36 68 87 06 33 33 33 33 33 33 cf 16

��3���̵����պ�
���ͣ�68 20 11 12 21 09 51 68 04 09 56 16 33 33 33 33 44 44 44 9f 16
���أ�68 20 11 12 21 09 51 68 c5 03 e9 04 56 99 16 68 20 11 12 21 09 51 68 84 02 89 89
��һ��˵��ͨ��ʧ�ܣ��ڶ�����ִ�гɹ������е���ٵ����������ܻ�ȡ����ȷ������أ�

����Ҫ���һ�����⣺�����������ͨ��ʧ����ô�죿Ӧ�ùرմ��ڣ�����smarthomeӦ�á�
*/
int serial_access(unsigned char *buf, unsigned int buf_len, unsigned int buf_size)
{
	int i = 0;
	int read_len = -1;
	
	sem_wait(&s_sem_serial);
	/*
	���1s��ȷ�����ڲ�����ȫ����ʵ���1s��̫׼ȷ����ȫ�п��ܲ���1s������ͨ��
	*/
	if( (time(NULL)-s_last_serial_cmd_time) < 1 ){
		DEBUG("too pressing cmd, sleep 1s to relex serial\n");
		sleep(1);
	}
	
	// ָ��ʶ��68 a0 a1 a2 a3 a4 a5 68
	unsigned char distinguish_cmd[32];
	memset(distinguish_cmd, 0, sizeof(distinguish_cmd));
	memcpy(distinguish_cmd, buf, 8);	//ʶ���: 68 20 11 12 21 06 36 68
	
	if(0!=sendto_serial(buf, buf_len)){
		DEBUG("send to serial failed\n");
		read_len = -1;
	}
	else{
		memset(buf, 0, buf_size);
		usleep(200000);
		read_len = recvfrom_serial(buf, buf_size, distinguish_cmd);
	}
	
	if(read_len>8)	// �Ϸ��ķ��ش�����Ҫ����ʶ��Σ�����8B�����磺68 20 11 12 21 06 36 68
	{
		int p = ascinasc(buf, read_len, distinguish_cmd, 8);
		if(p<0)
		{
			DEBUG("this recv buf from serial can not be distinguished, p=%d\n", p);
			read_len = -1;
		}
		else if(p>0){
			read_len -= p;
			for(i=0; i<read_len; i++){
				buf[i] = buf[p+i];
			}
		}
	
		printf("=========================================== read_len = %d, p=%d\n", read_len, p);
		for(i=0;i<read_len;i++)
			printf(" %02x", buf[i]);
		printf("\n===========================================\n");
	}
	else{
		DEBUG("recv from serial fialed, read_len=%d\n", read_len);
	}
	
	if(read_len<=8){
		read_len = -1;
		s_serial_failed_count++;
	}
	
	s_last_serial_cmd_time = time(NULL);

	sem_post(&s_sem_serial);
	
	if(s_serial_failed_count>2)
		serial_reset();
	
	return read_len;
}

void serial_fd_close(void)
{
	UART0_Close(g_serialfd);
	g_serialfd = -1;
}


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

