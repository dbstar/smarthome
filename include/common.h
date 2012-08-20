//ȫ��ʹ�õ�ͷ�ļ����������������ļ���������

#ifndef __COMMON_H__
#define __COMMON_H__
#include <errno.h>

#define DEBUG(x...) do{printf("[%s:%s:%d] ", __FILE__, __FUNCTION__, __LINE__);printf(x);}while(0)

#define ERROROUT(ERRSTR...) \
			do{printf("[%s:%s:%d] ", __FILE__, __FUNCTION__, __LINE__);\
			if(errno!=0) printf("[err note: %s]",strerror(errno));\
			printf(ERRSTR);}while(0)

#define MIN_LOCAL(a,b) ((a)>(b)?(b):(a))

typedef enum{
	BOOL_FALSE = 0,
	BOOL_TRUE
}BOOL_E;

#define MAXCMD 5									//max cmd/msg number
#define MAXLEN 4096									//the size of the transmission
#define MAXSLEEP 64									//the max sleep time of reconnect

#define WORKSPACE_SURFIX	"/data/dbstar/smarthome/"
#define DATABASE_DIR 		WORKSPACE_SURFIX"database/"
#define DATABASE			DATABASE_DIR"smarthome.db"	//sqlite3 database
#define SETTING_BASE		WORKSPACE_SURFIX"settings/base.ini"	//serverconfig path on ppc
#define SQLITECMDLEN 256							//the max number byte contained in "cmdStr"---command string--sqlite.cpp
#define WAITRES	600									//wait 600*myDelay(5) ms, if no response, return "#ff ff ff ffff#ff#ff ff#ff#"
#define SYNCTIME 10									//sync time
#define CMD_SIZE		4096
#define ALTERABLE_ENTITY_SIZE	4096

#define SN_DEFAULT_TEST			"89277089728430810813"		//WARNING: only for test, the lenght of sn is 20 currently
#define SW_VERSION				"20120816"
#define SMARTPOWER_SERVER_IP		"211.99.30.254"		// the ip of baidu.com is "61.135.169.105"
#define	SMARTPOWER_SERVER_PORT		(9999)

#define PREFIX_REGIST				"#rs#"
#define PREFIX_EXCEPTION_RESONSE	"#ff#"
#define PREFIX_REBOOT				"#reboot#"
#define PREFIX_DOWN_CMD				"#tt#"
#define PREFIX_UP_CMD				"#cc#"
#define PREFIX_ACTIVE_REPORT		"#re#"
#define PREFIX_ALARM				"#am#"		// in document, this is "#arm#"

#define CMD_KEYWORD_SYNC			"sync"
#define SMART_POWER_CMD_NUM			32

#define FIFO_STR_SIZE				32
#define FIFO_DIR					WORKSPACE_SURFIX"fifo/"
#define FIFO_2_SOCKET				FIFO_DIR"fifo_2_socket"
#define FIFO_2_INSTRUCTION			FIFO_DIR"fifo_2_instruction"
#define FIFO_SOCKET_SELF			FIFO_DIR"fifo_socket_self"

#define DEV_TTY_0		"/dev/ttyS0"
#define DEV_TTY_1		"/dev/ttyS1"
#define DEV_TTY_2		"/dev/ttyS2"
#define DEV_TTY_3		"/dev/ttyS3"

typedef enum{
	SOCKET_STATUS_EXCEPTION = -1,
	SOCKET_STATUS_CLOSED,
	SOCKET_STATUS_CREATE,
	SOCKET_STATUS_DISCONNECT,
	SOCKET_STATUS_CONNECTED,
	SOCKET_STATUS_REGISTING,
	SOCKET_STATUS_REGISTED,
	SOCKET_STATUS_UNREGIST
}SOCKET_STATUS_E;

typedef enum{
	CMD_STATUS_NULL = 0,
	CMD_STATUS_WRITING = 1,
	CMD_STATUS_WRITED = 64,
	CMD_STATUS_PROCESSING = 65,
	CMD_STATUS_PROCESSED = 129,
	CMD_STATUS_READING,
	CMD_STATUS_READED
}CMD_STATUS_E;

typedef enum{
	CMD_HEADER_UNDEFINED = -1,
	CMD_HEADER_INVALID = 0,
	CMD_HEADER_REGIST,
	CMD_HEADER_ISSUE,
	CMD_HEADER_REPORTED,
	CMD_HEADER_ALARM,
	CMD_HEADER_REBOOT,
	CMD_HEADER_SYNC,
	CMD_SYNC_DEVS,
	CMD_SYNC_MODL,
	CMD_SYNC_TIME,
	CMD_HEADER_INNER,
	CMD_ACTIVE_REPORTED_ACTPOWER,
	CMD_ACTIVE_REPORTED_POWER,
	CMD_INVALID
}CMD_HEADER_E;

// If CMD_ENTITY_MODEL_ARRAY, entity[0,1] means the model num. 
// So, it must has its largest num, perhaps 16 ??.
typedef enum{
	CMD_ENTITY_NORMAL_STR = 0,
	CMD_ENTITY_MODEL_ARRAY
}CMD_ENTITY_TYPE_E;

typedef struct{
	int type_id;
	int cmd_type;
	int control_val;
	int control_time;
	int frequency;
	char remark[64];
	int model_id;
}MODEL_S;

typedef struct{
	CMD_STATUS_E		status;	//�հף�������ϣ�������ϣ�
	unsigned int		id;		//ͨ������¸����������Ψһid����ͬ��ָ�
	CMD_HEADER_E		type;
	unsigned int		send_try;
	char				serv_str[128];
	char				entity[CMD_SIZE];
	CMD_ENTITY_TYPE_E	entity_type;
}SMART_POWER_CMD_S;

typedef enum{
	INSTRUCTION_UNDEFINED = 0,
	INSTRUCTION_CTRL,		// make sure this value is 1, i.e. "01"
	INSTRUCTION_INQUIRE,	// make sure this value is 2, i.e. "02"
	INSTRUCTION_OP,			// make sure this value is 3, i.e. "03"
	INSTRUCTION_OTHER		// make sure this value is 4, i.e. "04"
}INSTRUCTION_TYPE_E;


typedef struct{
	int 				type_id;		// actual using 6B
	char				reserve[5];		// actual using 4B
	INSTRUCTION_TYPE_E	type;
	int					arg1;			// parhaps need char[3] for two chars and one '\n'
	int					arg2;			// parhaps need char[3] for two chars and one '\n'
	int					alterable_flag;	// parhaps need char[3] for two chars and one '\n'
	char				alterable_entity[ALTERABLE_ENTITY_SIZE];
	int					insert_flag;
	int					index_in_cmds;
}INSTRUCTION_S;

typedef struct{
	time_t	server_time;
	double	difference_time;	// server_time - stb_time
}SMART_POWER_TIME_S;



typedef enum{
	CMD_ARRAY_OP_R,
	CMD_ARRAY_OP_PROCESS,
	CMD_ARRAY_OP_W
}CMD_ARRAY_OP_E;

typedef enum{
	ERR_TIMEOUT = -8,
	ERR_FORMAT = -7,
	ERR_FORMATPRO = -6,
	ERR_SOCKET = -5,
	ERR_DATABASE = -4,
	ERR_MEMORY = -3,
	ERR_SERIAL = -2,
	ERR_OTHER = -1,			//	usually, it means "&ff"
	RESULT_OK = 0,			// usually, it means "&00"
	RESULT_ALTERABLE_ENTITY_FILL_OK = 1		//	usually, it means the alterable_entity is filled
}INSTRUCTION_RESULT_E;

typedef struct{
	int		type_id;
	int		location_id;
	int		icon_id;
	int		oper_id;
	char	socket_id[17];
	char	room_name[9];
	char	dev_name[41];
	double	power_cumulation;
	int 	power_inquire_status;
	int		socket_status;
	int		first_equipment;
	int		active_inquire_status;
}EQUIPMENT_S;

#define EQUIPMENT_NUM	256
#define SERIAL_CMD_SIZE	256


typedef enum{
	SMART_SOCKET_ACTION_UNDEFINED = -1,
	SMART_SOCKET_ACTIVE_POWER_CONSUMPTION_READ = 0,	// �й��ܵ���
	SMART_SOCKET_VOLTAGE_READ,											// ��ѹ
	SMART_SOCKET_POWER_CURRENT_READ,								// ����
	SMART_SOCKET_ACTIVE_POWER_READ,									// �й�����
	SMART_SOCKET_REACTIVE_POWER_READ,								// �޹�����
	SMART_SOCKET_POWER_COEFFICIENT_READ,						// ��������
	SMART_SOCKET_RELAY_DISCONNECT,									// �̵����Ͽ�
	SMART_SOCKET_RELAY_CONNECT,											// �̵����պ�
	SMART_SOCKET_RELAY_STATUS_READ,									// �̵���״̬
	SMART_SOCKET_ADDR_CONFIRM,											// ��ַȷ��
	SMART_SOCKET_INSTRUCTION_INVALID,								// ָ�����
	SMART_SOCKET_COMMUNICATION_FAILD								// ͨѶʧ��
}SMART_SOCKET_ACTION_E;

typedef enum{
	SMART_SOCKET_RELAY_STATUS_OFF = 0,
	SMART_SOCKET_RELAY_STATUS_ON,
	SMART_SOCKET_RELAY_STATUS_UNKNOWN
}SMART_SOCKET_RALAY_STATUS_E;

// the length of msg string can not loong than 32
#define MSGSTR_INSTRUCTION_SELF	"msgstr_instruction_self"
#define MSGSTR_2_INSTRUCTION		"msgstr_2_instruction"
#define MSGSTR_2_SOCKET					"msgstr_2_socket"
#define MSGSTR_SOCKET_SELF			"msgstr_socket_self"

#define INSTRUCTION_INSERT_NUM (64)


typedef enum{
	TIMER_TYPE_MANUAL = 0,	// ȫ�ֹ����ƣ���unrigist���٣�
	TIMER_TYPE_INNER,		// �ڲ��Զ�ʵ�ֵģ�����ע�ᵽtimer�����С�1��ÿ��*5����ʱ��ѯ���ʣ�2��ÿ��50����ʱ��ѯ������3��ÿ��55����00ʱ�ϱ����ʣ�4��ÿ��00����05ʱ�ϱ����ʣ�5��ÿ����ʱˢtiming task
	TIMER_TYPE_TASK			// �������·��Ķ�ʱ���񣬰�����ʱģʽ�������ڡ�ÿ����ʱ����ˢ��
}TIMER_TYPE_E;

#define WORK_NORMAL_NOT_TEST	// �������г���Ӧ���ſ��˺�
//#define TEST_SERIAL_CMD_ONLY_ONCE	// ���ڵ��Դ���ʱʹ�ã�ÿ��ִ�п�ִ�г���ֻ�Դ��ڲ���һ��write-read���
//#define HEARTBEAT_SUPPORT		// �Ƿ���������Ŀǰ�ݲ�����

#define SERIAL_RECV_RETRY		(16)	// �Ӵ��ڽ�������ԵĴ��������ֵ��select�ĳ�ʱʱ�乲ͬ�����˴��ڽ���ʱ�ķ�Ӧ�ٶ�
#define TIMEZONE_EMENDATION		(8)	// ����ʱ�䣬��0ʱ��ʱ������ϼ�8��Сʱ��

int appoint_str2int(char *str, unsigned int str_len, unsigned int start_position, unsigned int appoint_len, int base);
void myDelay(int delay);

void ms_sleep(unsigned int ms);
unsigned int randint();
int timezone_repair(void);
int dir_exist_ensure(char *dir);
int zero_sec_get(time_t appoint_secs);
time_t time_get(time_t *timer);

#endif

