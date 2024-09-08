/*
 * galvanometer.h
 *
 *  Created on: 2024. 9. 8.
 *      Author: User
 */

#ifndef SRC_COMMON_HW_INCLUDE_GALVANO_H_
#define SRC_COMMON_HW_INCLUDE_GALVANO_H_


#include "hw_def.h"

#ifdef _USE_HW_GALVANO

#define GALVANO_PKT_BUF_MAX        HW_GALVANO_PKT_BUF_MAX



typedef struct
{
  uint8_t header;
  uint8_t length;
  uint8_t inst;
  uint8_t check;
  uint32_t param;
} galvano_packet_t;

enum
{
  GALVANO_INST_STATUS = 0x01,
  GALVANO_INST_READ = 0x02,
  GALVANO_INST_WRITE = 0x03,
};


typedef struct _galvano_driver_t
{
  bool isInit;
  bool isOpen;

  bool (*open)(uint8_t ch, uint32_t baud);
  bool (*close)(uint8_t ch);
  uint32_t (*available)(uint8_t ch);
  uint32_t (*write)(uint8_t ch, uint8_t* p_data, uint32_t length);
  uint8_t (*read)(uint8_t ch);
  bool (*flush)(uint8_t ch);
} galvano_driver_t;

typedef struct _galvano_t
{
  galvano_driver_t driver;

  bool 		 isOpen;
  uint8_t	 ch;
  uint32_t       baud;
  uint32_t 	 pre_time;
  uint32_t       state;

  galvano_packet_t packet;
  uint8_t        packet_buf[GALVANO_PKT_BUF_MAX];
} galvano_t;


typedef struct _galvano_status_t
{
  uint32_t       current_angle;
  bool			 flagGUI;
}galvano_status_t;

bool galvanoLoadDriver(galvano_t* p_galvano, bool (*load_func)(galvano_driver_t*));
bool galvanoOpen(uint8_t ch, uint32_t baud);
bool galvanoIsOpen(galvano_t *p_galvano);
bool galvanoClose(galvano_t *p_galvano);




bool galvanoSendInst(galvano_t *p_galvano, uint8_t inst, uint8_t *param);
bool galvanoReceivePacket(galvano_t *p_galvano);
bool galvanoProcessPKT(galvano_t *p_galvano, uint8_t rx_data);

bool galvanoInit(void);

void setRotation(galvano_status_t *p_galvano, uint32_t angle); //파라미터 값만큼 회전하며, 반환값은 현재각도+이동한 값이다.
uint32_t getAngle(galvano_status_t *p_galvano); // 반환값이 현재 각도의 값이다.
void zeroAngle(galvano_status_t *p_galvano); // 현재 각도를 0으로 설정.
void getStatus(galvano_status_t *p_galvano); // display status (angle, speed, direction)

void galvanoGUIRun();

#endif

#endif /* SRC_COMMON_HW_INCLUDE_GALVANO_H_ */
