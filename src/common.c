#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "common.h"
#include "instruction.h"

/* 
���ܣ���ָ���ַ�����ָ��λ�õ��ִ�������ָ���Ľ��ƽ���ת�����õ�long������
���룺	str				����ԭʼ�ַ��������Բ������ֿ�ͷ
		str_len			����ԭʼ�ַ�������
		start_position	����ָ��ת������ʼλ�ã�ԭʼ�ַ����Ŀ�ͷλ�ö���Ϊ0
		appoint_len		����ָ����Ҫת���ĳ���
		base			����ת���Ľ��ƣ�ȡֵ��strtolһ��
���أ�ʧ�ܷ���-1���������صõ���long int����
*/
int appoint_str2int(char *str, unsigned int str_len, unsigned int start_position, unsigned int appoint_len, int base)
{
	if(NULL==str || str_len<(start_position+appoint_len) || appoint_len>64 || (base<0 && 36<base)){
		DEBUG("some arguments are invalid\n");
		return -1;
	}

	char tmp_str[65];
	int ret_int = 0;
	
	memset(tmp_str, 0, sizeof(tmp_str));
	strncpy(tmp_str, str+start_position, appoint_len);
	ret_int = strtol(tmp_str, NULL, base);//atoi(tmp_str);
//	DEBUG("tmp_str=%s, will return with 0x%x==%d, origine str=%s, start at %d, aspect len %d\n", tmp_str,ret_int,ret_int, str, start_position, appoint_len);
	return ret_int;
}

/*
���ܣ�	�Ժ���Ϊ��λ��������
���룺	���ߵĺ�����
���أ�	���ޣ�
*/
void ms_sleep(unsigned int ms)
{
	if(ms<=0)
		return;
	struct timeval timeout;
	timeout.tv_sec=ms/1000;
	timeout.tv_usec=(ms%1000)*1000;			///ms
	select(0,NULL,NULL,NULL,&timeout);
}

/*
���ܣ�	������ֵ����װ��srand()��rand()��ϣ�����ʱ��usec%10000��Ϊ���ӣ���ֱ�Ӳ���sec���ӡ�����������жȸ�һЩ
���룺	���ޣ�
���أ�	�޷�������ֵ������rand()�ķ���ֵ
*/
unsigned int randint()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	
	srand((unsigned int)(tv.tv_usec)%10000);
	return rand();
}

/* 
	��localtime()�Ļ����϶�ʱ������У��������ϵͳ����δ��ʱ��������localtime�ķ���ֵ����GMTʱ�䣬��Ч��gmtime()��
	Ϊ�˹�ܴ����⣬�ֹ���ʱ�������У����
	
	�ر�Ҫע�⣺ֻ�����ϵͳδ����ʱ����Ϣ��������в��ȡ�����������������
*/
int timezone_repair(void)
{
	return 0;
	
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);
	
	int timezone = (TIMEZONE_EMENDATION*60 - tz.tz_minuteswest)/60;
	if(timezone<(-23))
		timezone = -23;
	else if(timezone>23)
		timezone = 23;
	
//	DEBUG("minuteswest=%d, dsttime=%d, repair with timezone=%d\n", tz.tz_minuteswest, tz.tz_dsttime, timezone);
	return timezone;
}

/*
������time_t time(time_t *timer)�����Ϸ�װ�����ĺ�����Ŀ���ǽ�ʱ��У���ͷ�����ʱ��У��ͳһ����
�����õ��Է�����Ϊ׼�ı���ʱ�䡣
һ������time_t���ͣ���time.h�ж��壺typedef long     time_t;    ʱ��ֵtime_t Ϊ�����͵ı�����
	��Ȼtime_tʵ�����ǳ����ͣ���δ����ĳһ�죬��һ��ʱ��㣨һ����1970��1��1��0ʱ0��0�룩
	����ʱ��������������ʱ�䣩�������˳��������ܱ�ʾ�����ķ�Χ��ô�죿
	��time_t�������͵�ֵ��˵��������ʾ��ʱ�䲻������2038��1��18��19ʱ14��07�롣
	Ϊ���ܹ���ʾ����Զ��ʱ�䣬һЩ����������������64λ��������������������������ʱ�䡣
	����΢����Visual C++�в�����__time64_t������������������ʱ�䣬
	��ͨ��_time64()�������������ʱ�䣨������ͨ��ʹ��32λ�ֵ�time()��������
	�����Ϳ���ͨ�����������ͱ���3001��1��1��0ʱ0��0�루��������ʱ��㣩֮ǰ��ʱ�䡣
�����˺����Ļ�����time()�������ڴ˻����ϣ����Ϸ�����ʱ��У����ʱ��У����
���������÷���time()���ơ�
*/
time_t time_get(time_t *timer)
{
	time_t timep;
	time(&timep);
	timep += smart_power_difftime_get();
	timep += 60*60*timezone_repair();
	
	if(NULL!=timer)
		*timer = timep;
	
	return timep;
}

/*
Ŀ¼��ʼ������������ȱ��Ŀ¼����Ȩ�޵����ļ�����ʧ��
*/
int dir_exist_ensure(char *dir)
{
	if(0!=access(dir, F_OK)){
		ERROROUT("dir %s is not exist\n", dir);
		if(0!=mkdir(dir, 0777)){
			ERROROUT("create dir %s failed\n", dir);
			return -1;
		}
		else{
			DEBUG("create dir %s success\n", dir);
			return 0;
		}
	}
	else{
		DEBUG("dir %s is exist\n", dir);
		return 0;
	}
}

/*
��ø���ʱ����ʱ����������1970��1��1��0ʱ0��0�뿪ʼ���㣩���������Ϊ0�����ý�����ʱ��������
���磺����1342524240��ʾUTCʱ��Tue Jul 17 11:24:00 2012����ô������Tue Jul 17 00:00:00 2012��Ӧ������
���ñ���ʱ��
*/
int zero_sec_get(time_t appoint_secs)
{
	time_t appoint_sec;
	time_t day_sec_0;	// ������ʱ������
	struct tm appoint_tm;
	
	if(0==appoint_secs)
		time_get(&appoint_sec);
	else
		appoint_sec = appoint_secs;
	
	localtime_r(&appoint_sec, &appoint_tm);
	
	appoint_tm.tm_hour = 0;
	appoint_tm.tm_min = 0;
	appoint_tm.tm_sec = 0;
	day_sec_0 = mktime(&appoint_tm);
	
	return day_sec_0;
}
