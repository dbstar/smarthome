#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>

#include "common.h"
#include "equipment.h"
#include "sqlite.h"

static EQUIPMENT_S g_equipments[EQUIPMENT_NUM];
static sem_t s_sem_equipment;

/*
���ܣ�	����sqlite���ݿ��ѯ�ص�������ѯ�������g_equipments������
���룺	result	����Ϊ������飨����ά������׺Ϊһά��
		row		������ѯ��������
		column	������ѯ��������
			ǰ���������ĺ��������֮���ο�sqlite API
�����	receiver�������������ڴ��ַ���京��;���Ӧ�ó������
���أ�	0�����ɹ�����������ʧ�ܣ�
*/
static int equipment_sqlite_callback(char **result, int row, int column, void *receiver)
{
	DEBUG("sqlite callback, row=%d, column=%d, receiver addr: %p\n", row, column, receiver);
	if(row<1){
		DEBUG("no record in table, return\n");
		return 0;
	}
	int i = 0;
	sem_wait(&s_sem_equipment);
	
	for(i=0;i<EQUIPMENT_NUM;i++)
		g_equipments[i].type_id = -1;

	for(i=1;i<row+1 && i<=EQUIPMENT_NUM;i++)
	{
		g_equipments[i-1].type_id = atoi(result[i*column]);
		g_equipments[i-1].location_id = atoi(result[i*column+1]);
		g_equipments[i-1].icon_id = atoi(result[i*column]+2);
		g_equipments[i-1].oper_id = atoi(result[i*column]+3);
		strncpy(g_equipments[i-1].socket_id, result[i*column+4], sizeof(g_equipments[i-1].socket_id)-1);
		strncpy(g_equipments[i-1].room_name, result[i*column+5], sizeof(g_equipments[i-1].room_name)-1);
		strncpy(g_equipments[i-1].dev_name, result[i*column+6], sizeof(g_equipments[i-1].dev_name)-1);
		g_equipments[i-1].power_cumulation = atoi(result[i*column]+7);

		DEBUG("[row %d/%d]type_id=0x%06x, localtion_id=0x%02x, icon_id=0x%02x, oper_id=0x%04x, socket_id=%s, room_name=%s, dev_name=%s, power_cumulation=%lf\n", 
			i,row,g_equipments[i-1].type_id,g_equipments[i-1].location_id,g_equipments[i-1].icon_id,g_equipments[i-1].oper_id,g_equipments[i-1].socket_id,g_equipments[i-1].room_name,g_equipments[i-1].dev_name,g_equipments[i-1].power_cumulation );
	}
	sem_post(&s_sem_equipment);

	if(i>EQUIPMENT_NUM){
		DEBUG("equipment array is full!!!\n");
	}
	return 0;
}

/*
���ܣ�	�豸�����ʼ���������ź�����ʼ��
���أ�	0�����ɹ���-1����ʧ��
*/
int equipment_init(void)
{
	if(-1==sem_init(&s_sem_equipment, 0, 1)){
		DEBUG("s_sem_equipment init failed\n");
		return -1;
	}

	return equipment_refresh();
}

/*
���ܣ�	�����ݿ��е��豸�б�����������ڴ��С������������豸�������豸��ɾ���豸������ô˺�����
���أ�	0�����ɹ���-1����ʧ��
*/
int equipment_refresh(void)
{
	char sqlite_cmd[SQLITECMDLEN];
	int (*sqlite_callback)(char **, int, int, void *) = equipment_sqlite_callback;
	
	sem_wait(&s_sem_equipment);
	int i = 0;
	for(i=0;i<EQUIPMENT_NUM;i++)
		g_equipments[i].type_id = -1;
	sem_post(&s_sem_equipment);
	
	memset(sqlite_cmd, 0, sizeof(sqlite_cmd));
	sprintf(sqlite_cmd,"SELECT typeID,locationID,iconID,operID,socketID,roomName,devName,elecData FROM devlist;");
	INSTRUCTION_RESULT_E ret = sqlite_read(sqlite_cmd, NULL, sqlite_callback);
	if(ret<RESULT_OK)
		ret = -1;
	else
		ret = 0;

	return ret;
}

/*
���ܣ�	��ȡָ��typeID���豸��Ϣ
���룺	type_id			����ָ����typeID
�����	tmp_equipment	������ȡ���豸�ṹ�壬�ɵ������ṩ�ռ�
���أ�	0�����ɹ���-1����ʧ��
*/
int equipment_get(int type_id, EQUIPMENT_S *tmp_equipment)
{
	if(NULL==tmp_equipment || 0==type_id)
	{
		DEBUG("some arguments are invalid\n");
		return -1;
	}

	int i = 0;
	int ret = -1;
	sem_wait(&s_sem_equipment);
	DEBUG("search equipment with type_id=0x%06x=%d\n", type_id, type_id);
	for(i=0; i<EQUIPMENT_NUM; i++){
		if(g_equipments[i].type_id==type_id){
			memcpy(tmp_equipment, &g_equipments[i], sizeof(EQUIPMENT_S));
			ret = 0;
			break;
		}
	}		
	sem_post(&s_sem_equipment);

	return ret;
}

/*
���ܣ�	��ȡ�����豸����Ϣ��������equipment���飬�������ʹ�ñȽ��ر�
�����	tmp_equipments��������equipment���ݣ��������������豸����Ϣ����СҪ��g_equipments��Ȼ�����ɵ������ṩ�ռ�
���أ�	0�����ɹ���-1����ʧ��
*/
int equipments_get(void *tmp_equipments)
{
	if(NULL==tmp_equipments)
	{
		return -1;
	}

	sem_wait(&s_sem_equipment);
	memcpy(tmp_equipments, &g_equipments, sizeof(g_equipments));
	sem_post(&s_sem_equipment);

	return 0;
}

