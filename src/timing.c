/*
����ʱ���ʹ�ñȽϷ���

һ��ʱ���������
	1��	ͨ��time()��gettimeofday֮��ĺ�����ȡ���ı���ʱ�ӣ�
	2��	ͨ���������·�ʱ�����������������������ʱ������õ�ϵͳʱ���ڣ�
		���Ǽ�¼ʱ����ͱ���ʱ��ʱ���ƫ�����smart_power_difftime_get()�õ���
	3��	ʹ�ñ���ʱ�䣨����ʱ�䣬����ʱ����ʱ����Ҫ��ʱ��ƫ�ͨ��Ϊ8��60��60�롣
		2012-08-17 ע��Ŀǰʱ���Ѿ���ȷ�����Դ˲��ֲ���ҪУ������У��0���ɡ�

����ʱ��ʹ�÷�ʽ������
	1��	��ʱ������ʱ�����ɷ��������ɵģ����ն���Ϊ������ʱ���ᣩ��ԭ����¼�����ݿ��time���ݱ��ڡ�
	2��	��ʱ���ص��ע�����뵱ǰʱ���Ĳ�࣬���磺50s��ִ��ĳ����
		ֻ��Ҫ��ʱ���յ�ʱ����ͳһ��timerע�������ʱ���յ�ʱ����ͳһ�����ɣ�����Ҫ����ʱ�����⡣
	
	3��	ֻ���ڹ�ע��ĳ�족�����ܼ��������漰�����족�Ĺ���ʱ������Ҫʱ��У����
		Ŀǰ�Ѿ����˷�װ��time_get�����ڡ�

������ʱ������
	1��	��ʱ�����ʱ�侫��Ϊ�룻
	2��	Ϊ�˽���ʱ�����ıȽ�ͨ�ã�������usec����ʱ�����struct timeval���͡�

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <time.h>
#include "timing.h"
#include "instruction.h"
#include "sqlite.h"
#include "socket.h"

#define REPORT_ACTPOWER_PREFIX	"#0000000000#02#0201#80#"
#define REPORT_POWER_PREFIX		"#0000000000#02#0301#84#"

static TIMER_S	g_timers[TIMER_NUM];
static int		g_timers_time_rectify_flag = 0;	// ���ڿ��ܴ����ȳ�ʼ����һ��timer��������������·�ʱ�䣬���¶�ʱ����׼�������Ҫ�ڷ������·�ʱ��ʱ��У��timer������ʱ��
static sem_t	s_sem_timer;

static int	 g_wday_code[] = {0x80, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
static int g_power_inquire_min = -1;
static int g_power_consumption_inquire_hour = -1;
static struct tm g_power_consumption_upload_tm;
static struct tm g_power_upload_tm;
static int g_refresh_timing_task_mday = -1;
char *wday[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

// the timer_callback funtion can not be blocked!!!
// this function CAN NOT use sem_wait/sem_post, because of timing_task_refresh_callback
/*
arg1	�����豸��typeID����ʶ���������
arg2	�������·���ʱ����ʱ������ʱ�䣬��ʶ��ʱ����
		������Ҳ�Ǽ�¼�����ݿ�time���ݱ��ڣ���typeIDһ����Ϊ�˵�����������ı�ʶ��
		ע�����ʱ���Ƕ�ʱ����ʼʱ�䣬��ʱ������ʱ����Ҫ���м��㡣
*/
static int inner_timer_regist(struct timeval *tv, TIMER_TYPE_E type,int arg1, int arg2, int (*timer_callback)(struct timeval *tv_datum, int arg1, int arg2))
{
	int i = 0;
	int timer_is_exist = 0;
	struct timeval tv_now;
	
	if(tv->tv_sec<0 || (0==tv->tv_sec && tv->tv_usec<500000) || NULL==timer_callback){
		DEBUG("timer regist failed: timer(%ld:%ld) is invalid to regist, or callback is NULL\n", tv->tv_sec, tv->tv_usec);
		return -1;
	}
	
	if(-1==gettimeofday(&tv_now, NULL)){
		ERROROUT("gettimeofday failed\n");
		// do something here, such as funciton time() but not return with -1
		return -1;
	}
	tv_now.tv_sec += smart_power_difftime_get();
	DEBUG("regist a timer, aim at %ld, sec=%ld, arg1=%d, arg2=%d, type=%d, callback=%p\n", (tv_now.tv_sec+tv->tv_sec), tv->tv_sec,arg1,arg2,type,timer_callback);
	
	for(i=0; i<TIMER_NUM; i++){
		if(-1!=g_timers[i].id){
			printf("[%d]timer.sec=%ld,arg1=%d, arg2=%d, type=%d, callback=%p\n",
					i,g_timers[i].tv_timer.tv_sec,g_timers[i].arg1,g_timers[i].arg2,g_timers[i].type,g_timers[i].callback);
			
			/*
			��ʱ������ֻ��Ҫ�ȶԵ�����typeID��arg1������ʱ����ʱ�䣨arg2����
			������ʱ��(tv_now.tv_sec+tv->tv_sec)���п���ƫ������1�룬������Ϊ�ж�����
			*/
			if(TIMER_TYPE_TASK==type){
				if(	timer_callback==g_timers[i].callback 
					&& arg1==g_timers[i].arg1
					&& arg2==g_timers[i].arg2
					&& type==g_timers[i].type){
					DEBUG("this timer is already exist, no need to regist it\n");
					timer_is_exist = 1;
					break;
				}
			}
			else if(TIMER_TYPE_MANUAL==type){
				if(	(tv_now.tv_sec+tv->tv_sec)==(g_timers[i].tv_timer.tv_sec)
					&& timer_callback==g_timers[i].callback 
					&& arg1==g_timers[i].arg1
					&& arg2==g_timers[i].arg2
					&& type==g_timers[i].type){
					DEBUG("this timer is already exist, no need to regist it\n");
					timer_is_exist = 1;
					break;
				}
			}
			else{	// TIMER_TYPE_INNER
				DEBUG("this type(%d) should be processed dirctly in timer_poll\n", type);
				return -1;
			}
		}
	}
	DEBUG("timer_is_exist=%d, now(server time, without timezone) %s\n", timer_is_exist, asctime(localtime(&(tv_now.tv_sec))));
	if(0==timer_is_exist){
		for(i=0; i<TIMER_NUM; i++){
			if(-1==g_timers[i].id){
				g_timers[i].id = i;
				g_timers[i].tv_timer.tv_sec = tv_now.tv_sec+tv->tv_sec;
				g_timers[i].tv_timer.tv_usec = tv_now.tv_usec+tv->tv_usec;
				g_timers[i].type = type;
				g_timers[i].arg1 = arg1;
				g_timers[i].arg2 = arg2;
				g_timers[i].callback = timer_callback;
				DEBUG("regist at g_timers[%d], aim sec=%ld, callback=%p\n", i, g_timers[i].tv_timer.tv_sec, g_timers[i].callback);
				break;
			}
		}
	}

	if(TIMER_NUM==i)
		i = -1;

	return i;
}

int timer_regist(struct timeval *tv, TIMER_TYPE_E type,int arg1, int arg2, int (*timer_callback)(struct timeval *tv_datum, int arg1, int arg2))
{
	int ret = -1;
	int (*timer_cb)(struct timeval *tv_datum, int arg1, int arg2) = timer_callback;
	sem_wait(&s_sem_timer);
	ret = inner_timer_regist(tv, type, arg1, arg2, timer_cb);
	sem_post(&s_sem_timer);

	return ret;
}


int timer_unregist(int *timer_id)
{
	if(*timer_id<0 || *timer_id>TIMER_NUM)
		return -1;
	
	sem_wait(&s_sem_timer);
	g_timers[*timer_id].id = -1;
	*timer_id = -1;				// avoid from multi unrigist
	sem_post(&s_sem_timer);
	return 0;
}

int timing_init(void)
{
	DEBUG("timing arrarys init\n");
	if(-1==sem_init(&s_sem_timer, 0, 1)){
		DEBUG("s_sem_timer init failed\n");
		return -1;
	}

	int i = 0;
	sem_wait(&s_sem_timer);
	for(i=0; i<TIMER_NUM; i++){
		g_timers[i].id = -1;
		g_timers[i].callback = NULL;
	}
	sem_post(&s_sem_timer);

	g_power_inquire_min = -1;
	g_power_consumption_inquire_hour = -1;

	g_power_consumption_upload_tm.tm_hour = -1;
	g_power_consumption_upload_tm.tm_min = 55+randint(5.0);
	
	g_power_upload_tm.tm_hour = -1;
	g_power_upload_tm.tm_min = randint(5.0);

	DEBUG("g_power_consumption_upload_tm.tm_min=%d, g_power_upload_tm.tm_min=%d\n", g_power_consumption_upload_tm.tm_min, g_power_upload_tm.tm_min);

	timing_task_refresh();
	return 0;
}

static int timing_task_activation_callback(char **result, int row, int column, void *receiver)
{
	if(row<1){
		DEBUG("select from table 'time' for 0 line\n");
		return -1;
	}
	DEBUG("select from table 'time' by typeID, row=%d, column=%d\n", row, column);
	int i=0;
	INSTRUCTION_S timing_inst;
	for(i=1; i<row+1; i++){
		timing_inst.type_id = atoi(result[i*column]);
		timing_inst.type = atoi(result[i*column+1]);
		timing_inst.arg1 = atoi(result[i*column+2])>>8 & 0xff;
		timing_inst.arg2 = atoi(result[i*column+2]) & 0xff;
		timing_inst.alterable_flag = 0x00;
		timing_inst.insert_flag = 1;
		memset(timing_inst.alterable_entity, 0, sizeof(timing_inst.alterable_entity));
		instruction_dispatch(&timing_inst);
	}
	return -1;
}

static int timing_task_ring_callback(struct timeval *tv_datum, int type_id, int absolute_sec)
{
	DEBUG("timing task ring, type_id=%d, absolute_sec=%d\n", type_id, absolute_sec);
	int (*sqlite_callback)(char **,int,int,void *) = timing_task_activation_callback;

	char sqlite_cmd[128];
	memset(sqlite_cmd, 0, sizeof(sqlite_cmd));
	sprintf(sqlite_cmd,"SELECT typeID,cmdType,controlVal FROM time WHERE typeID=%d AND controlTime=%d;", type_id, absolute_sec);
	INSTRUCTION_RESULT_E ret = sqlite_read(sqlite_cmd, NULL, sqlite_callback);
	if(ret>RESULT_OK){
		ret = RESULT_OK;
	
		/*
		Ŀǰ��timer�������ڲ����Ķ�ʱ��������ֱ�ӵ��ò���״̬�ϱ�
		*/
		char typeIDs[256];
		snprintf(typeIDs, sizeof(typeIDs), "%d", type_id);
		sockets_status_report(typeIDs);
	}
	return -1;	// -1: invalide the timer after call
}

static int timing_task_refresh_callback(char **result, int row, int column, void *receiver)
{
	DEBUG("row=%d, column=%d, receiver addr=%p\n", row, column, receiver);
	
	int i = 0;
	time_t now_sec;
	time_t today_sec_0;	// ������ʱ������
	time_t timer_sec = 0;
	int (*timer_callback)(struct timeval *tv_datum, int type_id, int absolute_sec) = timing_task_ring_callback;
	struct tm now_tm;
	struct timeval tv;
	struct tm timer_tm;
	
	/*
	Ƶ�ʣ�00��ʾָ��ĳ�죻01��ʾÿ�죻04��ʾÿ���ġ�������
	*/
	int frequency_wday;
	
	/*
	�����漰�����족���жϣ����ͨ��time_get��ȡ��������ʱ��+������ʱ����У��+ʱ��У������ʱ��
	*/
	time_get(&now_sec);
	today_sec_0 = zero_sec_get(now_sec);
	
	localtime_r(&now_sec, &now_tm);
	DEBUG("fresh timing task at (server time): %s\n",asctime(&now_tm));

	sem_wait(&s_sem_timer);
	for(i=0;i<TIMER_NUM;i++){
		if(TIMER_TYPE_TASK==g_timers[i].type)
			g_timers[i].id = -1;
	}
	
	for(i=1;i<row+1;i++)
	{
		timer_sec = atoi(result[i*column+1]);
	
		/*
		��ʱ������ж��漰�����족����ʱ�����뿼��ʱ����
		���ڷ������·���ʱ���Ѿ��Ƿ�����ʱ����ʱ�䣬��δ����ʱ���������Ҫ����ʱ�������뵱ǰʱ��now_sec�ȶ�
		*/
		timer_sec += 60*60*timezone_repair();
		localtime_r(&timer_sec, &timer_tm);
		
		frequency_wday = atoi(result[i*column+2]);
		
		DEBUG("typeID=%s, frequency=0x%02x, controlTime=%s, aim at %s", result[i*column], frequency_wday, result[i*column+1], asctime(localtime(&timer_sec)));
		
		if(0x00==frequency_wday){				// check whether appoint day
			if(now_tm.tm_mday==timer_tm.tm_mday){			// it is today
				DEBUG("regist timer for (server time): %s", asctime(&timer_tm));
				tv.tv_sec = timer_sec-now_sec;
				tv.tv_usec = 0;
				inner_timer_regist(&tv, TIMER_TYPE_TASK,atoi(result[i*column]), atoi(result[i*column+1]),timer_callback);
			}
			else
				DEBUG("no using time task, now_tm.tm_mday=%d, timer_tm.tm_mday=%d\n", now_tm.tm_mday, timer_tm.tm_mday);
		}
		
		/*
		�������ָ��ĳ��Ķ�ʱ��������ʱ��timer_secָ��������ʼ��ʱ�䡣
		*/
		else if(	(0x01 & frequency_wday)							// every day
					|| (g_wday_code[now_tm.tm_wday] & frequency_wday) )		// match wday
		{
			/*
			�ԡ��족Ϊ��׼���鿴�������ڽ����Ƿ���Ҫ�������ȶԵ��ǽ�����ʱ��������
			*/
			if(zero_sec_get(timer_sec)<=today_sec_0)
			{
				time_t tmp_timer_sec = timer_sec;
				DEBUG("regist timer aim at %ld sec, %s\n", timer_sec, asctime(localtime(&tmp_timer_sec)));
				
				/*
				�õ�timer�������ڡ��족��ʱ�Ĳ�࣬���磺����3:30��ʱ����Ӧ�õ�3:30��Ӧ������
				*/
				time_t timer_sec_on_day = timer_sec - zero_sec_get(timer_sec);
				
				/*
				�õ���ǰʱ���������ʱ�Ĳ�࣬���磺��ǰΪ3:00��Ӧ�õ�3:00��Ӧ������
				*/
				time_t now_sec_on_day = now_sec - today_sec_0;
				
				tv.tv_sec = timer_sec_on_day - now_sec_on_day;	// �����timer�͵�ǰʱ��Ĳ�ֵ��
				tv.tv_usec = 0;
				inner_timer_regist(&tv, TIMER_TYPE_TASK,atoi(result[i*column]), atoi(result[i*column+1]),timer_callback);
			}
			else
				DEBUG("this timing task no need to start today\n");
		}
		else
			DEBUG("this frequency can not be processed: 0x%02x\n", frequency_wday);
	}
	sem_post(&s_sem_timer);
	
	return 0;
}

int timing_task_refresh(void)
{
	INSTRUCTION_RESULT_E ret = RESULT_OK;
	int (*sqlite_callback)(char **,int,int,void *) = timing_task_refresh_callback;
	time_t now_sec;
	struct tm now_tm;
	
	time_get(&now_sec);
	localtime_r(&now_sec, &now_tm);

	DEBUG("refresh timing task at (repair with server time and timezone) %d %02d %02d  %s  %02d:%02d:%02d\n",(1900+now_tm.tm_year), (1+now_tm.tm_mon), now_tm.tm_mday,
		wday[now_tm.tm_wday], now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec);
	
	char sqlite_cmd[128];

	memset(sqlite_cmd, 0, sizeof(sqlite_cmd));
	sprintf(sqlite_cmd,"DELETE FROM time WHERE frequency=0 AND controlTime<%d;",(int)now_sec);
	ret = sqlite_execute(sqlite_cmd);
	if(ret!=RESULT_OK)
		DEBUG("delete overdue timer failed\n");
	
	memset(sqlite_cmd, 0, sizeof(sqlite_cmd));
	sprintf(sqlite_cmd,"SELECT typeID,controlTime,frequency FROM time;");
	ret = sqlite_read(sqlite_cmd, NULL, sqlite_callback);
	if(ret>=RESULT_OK)
	{
		time_get(&now_sec);
		localtime_r(&now_sec, &now_tm);
		g_refresh_timing_task_mday = now_tm.tm_mday;
	}
	if(ret>RESULT_OK)
		ret = RESULT_OK;
	
	return ret;
}

// ���������·�ʱ��ʱ������Ҫ��մ˱�ǣ�ʹ�ö�ʱ���ܹ����Ѿ�ע���timer����ʱ������У����
int time_rectify_flag_reset(void)
{
	g_timers_time_rectify_flag = 0;

	return 0;
}

// �Թ��ʺ͵����Ĳ�ѯ�漰���ܶ��豸�Ĵ��ڽ�������ҪһЩʱ�䡣Ϊ�˱���timerģ��ļ�ʱ��Ӧ������Щ����Ĳ���ѹ�뵽instruction�̡߳�
int timer_poll(void)
{
	int i = 0;
	int ret_callback = -1;
	int (*callbackfun)(struct timeval *, int arg1, int arg2);
	int (*sqlite_callback)(char **,int,int,void *) = power_inquire_callback;
	INSTRUCTION_RESULT_E ret = RESULT_OK;
	time_t now_sec;
	struct tm now_tm;
	INSTRUCTION_S insert_inst;
	char sqlite_cmd[SQLITECMDLEN];
	char entity[ALTERABLE_ENTITY_SIZE];	// 4096

	sem_wait(&s_sem_timer);
	
	/*
	�������·�ʱ��ʱ��Ҫ��ʱ�Ա���timerʱ�����У��
	*/
	if(0==g_timers_time_rectify_flag && 0!=smart_power_difftime_get() ){
		for(i=0;i<TIMER_NUM;i++){
			if(-1!=g_timers[i].id)
				g_timers[i].tv_timer.tv_sec += smart_power_difftime_get();
		}
		DEBUG("retify the aim time with %d secs in g_timers, because of the difference between server and local\n", smart_power_difftime_get());
		g_timers_time_rectify_flag = 1;
	}
	
	struct timeval tv_now;
	if(-1==gettimeofday(&tv_now, NULL)){
		ERROROUT("gettimeofday failed\n");
		// do something here, such as funciton time(), but not return with -1
		sem_post(&s_sem_timer);
		return -1;
	}
	tv_now.tv_sec += smart_power_difftime_get();
//	DEBUG("now_sec=%ld, aim_sec=%ld\n", tv_now.tv_sec, g_timers[i].tv_timer.tv_sec);
	for(i=0; i<TIMER_NUM; i++){
		if(	-1!=g_timers[i].id
			&& 	NULL!=g_timers[i].callback
			&&	(tv_now.tv_sec > g_timers[i].tv_timer.tv_sec
				||	(tv_now.tv_sec == g_timers[i].tv_timer.tv_sec
					&&	tv_now.tv_usec >= g_timers[i].tv_timer.tv_usec)) ){
			DEBUG("g_timers[%d]=%d is ring at %d, call its callback function. ptr=%p\n", i, (int)(g_timers[i].tv_timer.tv_sec), (int)(tv_now.tv_sec), g_timers[i].callback);
			callbackfun = g_timers[i].callback;
			ret_callback = callbackfun(&(g_timers[i].tv_timer), g_timers[i].arg1, g_timers[i].arg2);
			if(-1==ret_callback)
				g_timers[i].id = -1;
		}
	}
	sem_post(&s_sem_timer);
	
	time(&now_sec);
	now_sec += smart_power_difftime_get();
	localtime_r(&now_sec, &now_tm);
	if(g_power_inquire_min!=now_tm.tm_min && 0==now_tm.tm_min%5 && 0!=now_tm.tm_min%10){
		DEBUG("timer to insert a instruction to inquire power at %d %02d %02d - %02d:%02d:%02d\n", 
			(1900+now_tm.tm_year), (1+now_tm.tm_mon),now_tm.tm_mday,now_tm.tm_hour + timezone_repair(), now_tm.tm_min, now_tm.tm_sec);
		g_power_inquire_min = now_tm.tm_min;
		insert_inst.type_id = 0x000000;
		insert_inst.type = 0x02;
		insert_inst.arg1 = 0x02;
		insert_inst.arg2 = 0x01;
		insert_inst.alterable_flag = 0x00;
		insert_inst.insert_flag = 1;

		instruction_insert(&insert_inst);
	}
	if(g_power_consumption_inquire_hour!=now_tm.tm_hour && 50==now_tm.tm_min){
		DEBUG("timer to insert a instruction to inquire power consumption at %d %02d %02d - %02d:%02d:%02d\n", 
			(1900+now_tm.tm_year), (1+now_tm.tm_mon),now_tm.tm_mday,now_tm.tm_hour + timezone_repair(), now_tm.tm_min, now_tm.tm_sec);
		g_power_consumption_inquire_hour = now_tm.tm_hour;
		insert_inst.type_id = 0x000000;
		insert_inst.type = 0x02;
		insert_inst.arg1 = 0x03;
		insert_inst.arg2 = 0x01;
		insert_inst.alterable_flag = 0x00;
		insert_inst.insert_flag = 1;
		instruction_insert(&insert_inst);
	}
#if 1
	if(g_power_consumption_upload_tm.tm_hour!=now_tm.tm_hour && now_tm.tm_min==g_power_consumption_upload_tm.tm_min)
#else	// only for test!!!!
	//DEBUG("now_tm.tm_min=%d, now_tm.tm_min(mod 10)=%d, g_power_consumption_upload_tm.tm_min=%d\n", now_tm.tm_min, (now_tm.tm_min)%10, g_power_consumption_upload_tm.tm_min);
	if(6==((now_tm.tm_min)%10) && now_tm.tm_min!=g_power_consumption_upload_tm.tm_min)
#endif
	{
		DEBUG("timer to insert a cmd to upload power consumption at %d %02d %02d - %02d:%02d:%02d\n", 
			(1900+now_tm.tm_year), (1+now_tm.tm_mon),now_tm.tm_mday,now_tm.tm_hour + timezone_repair(), now_tm.tm_min, now_tm.tm_sec);
		DEBUG("g_power_consumption_upload_tm.tm_hour=%d, g_power_consumption_upload_tm.tm_min=%d\n", g_power_consumption_upload_tm.tm_hour,g_power_consumption_upload_tm.tm_min);
		g_power_consumption_upload_tm.tm_hour = now_tm.tm_hour;
		g_power_consumption_upload_tm.tm_min = 55+randint(5.0);
		
		/*ȡ�������ϱ���û����ɵĶ�����Ȼ�����Ǵӡ�1���ָ�Ϊ��0�����ϲ��������ϱ�����Ҳ���ڽ�����Ϊ��1�����������¼�¼�ɵ������*/
		/*һ������£���������ǿն���*/
		smart_power_active_reported_clear(CMD_ACTIVE_REPORTED_POWER);
		snprintf(sqlite_cmd, sizeof(sqlite_cmd), "UPDATE power SET status=0 WHERE status=1;");
		sqlite_execute(sqlite_cmd);
		
		snprintf(sqlite_cmd, sizeof(sqlite_cmd), "SELECT typeID, hourTime, data FROM power WHERE status=0;");
//		DEBUG("active_power_sum addr=%p\n", sqlite_callback);
		snprintf(entity, sizeof(entity), "%s", REPORT_POWER_PREFIX);
		ret = sqlite_read(sqlite_cmd, entity, sqlite_callback);
		if(ret>RESULT_OK){
			/*���Ѿ���ѯ�ļ�¼���Ϊ��1���������ϱ��ɹ���ʧ�ܺ󣬽����±�ǣ������ظ�����©�ϱ�*/
			/*��׼������Ӧ���ǽ���ѯ��status��λ����һ�����ݿ⡰��������ɣ���ȷ��״̬ת��ͬ��*/
			/*����һ��©����������ڼ�¼̫�ർ��һ��alterable_entity�޷��������д��ϱ���¼��������ȴһ���Զ����ϱ�ǡ�1�����Ӷ����¶���ļ�¼�����ⶪ��*/
			snprintf(sqlite_cmd, sizeof(sqlite_cmd), "UPDATE power SET status=1 WHERE status=0;");
			sqlite_execute(sqlite_cmd);
			snprintf(	entity+strlen(entity), sizeof(entity)-strlen(entity), 
					"#");
			cmd_insert(entity, CMD_ACTIVE_REPORTED_POWER);
			ret = RESULT_OK;
		}
		else
			DEBUG("no record for power consumption to upload\n");
	}
	if(g_power_upload_tm.tm_hour!=now_tm.tm_hour && now_tm.tm_min==g_power_upload_tm.tm_min){
		DEBUG("timer to insert a cmd to upload power at %d %02d %02d - %02d:%02d:%02d\n", 
			(1900+now_tm.tm_year), (1+now_tm.tm_mon),now_tm.tm_mday,now_tm.tm_hour + timezone_repair(), now_tm.tm_min, now_tm.tm_sec);
		DEBUG("g_power_upload_tm.tm_hour=%d, g_power_upload_tm=%d\n", g_power_upload_tm.tm_hour, g_power_upload_tm.tm_min);
		g_power_upload_tm.tm_hour = now_tm.tm_hour;
		g_power_upload_tm.tm_min = randint(5.0);
		
		smart_power_active_reported_clear(CMD_ACTIVE_REPORTED_ACTPOWER);
		snprintf(sqlite_cmd, sizeof(sqlite_cmd), "UPDATE actpower SET status=0 WHERE status=1;");
		sqlite_execute(sqlite_cmd);
		
		snprintf(sqlite_cmd, sizeof(sqlite_cmd), "SELECT typeID, hourTime, data FROM actpower WHERE status=0;");
//		DEBUG("active_power_sum addr=%p\n", sqlite_callback);
		snprintf(entity, sizeof(entity), "%s", REPORT_ACTPOWER_PREFIX);
		ret = sqlite_read(sqlite_cmd, entity, sqlite_callback);
		if(ret>RESULT_OK){
			/*��׼������Ӧ���ǽ���ѯ��status��λ����һ�����ݿ⡰��������ɣ���ȷ��״̬ת��ͬ��*/
			snprintf(sqlite_cmd, sizeof(sqlite_cmd), "UPDATE actpower SET status=1 WHERE status=0;");
			sqlite_execute(sqlite_cmd);
			snprintf(	entity+strlen(entity), sizeof(entity)-strlen(entity), 
					"#");
			cmd_insert(entity, CMD_ACTIVE_REPORTED_ACTPOWER);
			ret = RESULT_OK;
		}
		else
			DEBUG("no record for power to upload\n");
	}
	if(g_refresh_timing_task_mday!=now_tm.tm_mday && 0==now_tm.tm_hour){
		g_refresh_timing_task_mday = now_tm.tm_mday;
		timing_task_refresh();
	}
	
	return ret;
}

void timing_mainloop(void)
{
	struct timeval s_time = {0,500000};	// actually, we need 50ms timer, not 1.05s. here is testing case
	int ret = -1;
	int error_notice_count = 0;
	
	while(1){
		s_time.tv_sec = 0;	// warning: some system will clear the timeout setting after select action
		s_time.tv_usec = 300000;
		ret = select(0, NULL, NULL, NULL, &s_time);
		if(ret<0){
			if(error_notice_count>100000){
				DEBUG("make a sleep by select failed, use function sleep(1) directly\n");
				error_notice_count = 0;
			}
			error_notice_count++;
			
			sleep(1);
		}
		else if(0==ret){
			timer_poll();
		}
	}
}
