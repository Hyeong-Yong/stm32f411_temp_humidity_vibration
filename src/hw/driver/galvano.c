/*
 * galvano.c
 *
 *  Created on: 2022. 1. 8.
 *      Author: HYJH
 */

#include "galvano.h"
#include "cli.h"
#include "galvano/galvano_uart.h"

#define STX	0xFF

#ifdef _USE_HW_GALVANO

enum state
{
  STATE_HEADER=0,
  STATE_LENGTH,
  STATE_INST,
  STATE_PARAM_1,
  STATE_PARAM_2,
  STATE_PARAM_3,
  STATE_PARAM_4,
  STATE_CHECK,
};
enum index
{
  INDEX_HEADER=0,
  INDEX_LENGTH,
  INDEX_INST,
  INDEX_PARAM_1,
  INDEX_PARAM_2,
  INDEX_PARAM_3,
  INDEX_PARAM_4,
  INDEX_CHECK,
};


enum instruction
  {
    INST_SET_ANGLE =0x01,
    INST_SET_ZERO = 0x04,
    INST_GET_ANGLE = 0x05,
  };

galvano_packet_t packet;
galvano_t galvano;
galvano_status_t galvano_status;

static bool checksumPacket(galvano_t* p_galvano);
#ifdef _USE_HW_CLI
  static void cligalvano(cli_args_t *args);
#endif

bool galvanoLoadDriver(galvano_t* p_galvano, bool (*load_func)(galvano_driver_t*))
  {
    bool ret;

    ret = load_func(&p_galvano->driver);
    return ret;
  }
bool galvanoOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;
  if (galvano.driver.isInit ==false)
    {
      return false;
    }

  galvano.ch=ch;
  galvano.baud=baud;
  galvano.isOpen=galvano.driver.open(ch, baud);
  galvano.pre_time=millis();
  galvano.state=STATE_HEADER;

  ret = galvano.isOpen;
  return ret;
}
bool galvanoIsOpen(galvano_t *p_galvano)
{
  return p_galvano->isOpen;
}
bool galvanoClose(galvano_t *p_galvano)
{
  bool ret= true;

  return ret;
}


// cli 다시 검토 및 send함수 작성
// 항상 galvanoUart 오픈 또는 선택시 galvanoUart 오픈

bool galvanoSendInst(galvano_t *p_galvano, uint8_t inst, uint8_t *param);
bool galvanoReceivePacket(galvano_t *p_galvano)
{
  bool ret= false;
  uint8_t rx_data;
  uint32_t pre_time;

  if (p_galvano->isOpen == false)
    {
      return ret;
    }

  pre_time=millis();
  while (p_galvano->driver.available(p_galvano->ch)>0)
    {
      rx_data = p_galvano->driver.read(p_galvano->ch);
      ret = galvanoProcessPKT(p_galvano, rx_data);

      if (ret == true)
	{
	  break;
	}

      if (millis() - pre_time >= 50)
	{
	  break;
	}
    }
  return ret;
}
bool galvanoProcessPKT(galvano_t *p_galvano, uint8_t rx_data)
{
  bool ret =false;

  if ( millis() - p_galvano->pre_time > 100)
    {
      p_galvano->state = STATE_HEADER;
    }
  p_galvano->pre_time= millis();

  switch (p_galvano->state)
  {
    case STATE_HEADER:

      if (rx_data == STX)
	{
	  p_galvano->packet_buf[INDEX_HEADER]= rx_data;
	  p_galvano->state = STATE_LENGTH;
	}
      break;

    case STATE_LENGTH:
      p_galvano->packet_buf[INDEX_LENGTH]= rx_data;
      p_galvano->state= STATE_INST;
      break;

    case STATE_INST:
      p_galvano->packet_buf[INDEX_INST]= rx_data;
      p_galvano->state = STATE_PARAM_1;
      break;

    case STATE_PARAM_1:
      p_galvano->packet_buf[INDEX_PARAM_1] = rx_data;
      p_galvano->state= STATE_PARAM_2;
      break;

    case STATE_PARAM_2:
      p_galvano->packet_buf[INDEX_PARAM_2] = rx_data;
      p_galvano->state= STATE_PARAM_3;
      break;

    case STATE_PARAM_3:
      p_galvano->packet_buf[INDEX_PARAM_3] = rx_data;
      p_galvano->state= STATE_PARAM_4;
      break;

    case STATE_PARAM_4:
      p_galvano->packet_buf[INDEX_PARAM_4] = rx_data;
      p_galvano->state= STATE_CHECK;
      break;

    case STATE_CHECK:
      p_galvano->packet_buf[INDEX_CHECK] = rx_data;
      if (checksumPacket(&galvano) == true)
	{
	  packet.header = p_galvano->packet_buf[INDEX_HEADER];
	  packet.inst   = p_galvano->packet_buf[INDEX_INST];
	  packet.length = p_galvano->packet_buf[INDEX_LENGTH];
	  packet.inst   = p_galvano->packet_buf[INDEX_INST];
	  packet.param  = p_galvano->packet_buf[INDEX_PARAM_1] << 0;
	  packet.param |= p_galvano->packet_buf[INDEX_PARAM_2] << 8;
	  packet.param |= p_galvano->packet_buf[INDEX_PARAM_3] << 16;
	  packet.param |= p_galvano->packet_buf[INDEX_PARAM_4] << 24;
	  packet.check  = p_galvano->packet_buf[INDEX_CHECK];
	  ret=true;
	  galvano_status.flagGUI=true;
	  p_galvano->state = STATE_HEADER;
	}
      else
	{
	  p_galvano->state = STATE_HEADER;
	}
      break;
  }
  return ret;
}


bool checksumPacket(galvano_t* p_galvano)
{
  bool ret = false;
  uint32_t sum = 0, total = 0;
  for (int i = 0; i < INDEX_CHECK+1; i++)
    {
      sum += p_galvano->packet_buf[i];
    }
  total = sum;
  total = total & 0xFF;
  total = (~total + 1) & 0xFF;
  total += sum;
  total = total & 0xFF;
  if (total == 0)
    {
      return ret = true;
    }
  return ret;
}

bool galvanoInit(void)
{
  bool ret= true;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  galvano_status.current_angle = 0;
#ifdef _USE_HW_CLI
  cliAdd("galvano", cligalvano);
#endif

  galvanoLoadDriver(&galvano, galvanoUartDriver);

  return ret;
}

uint32_t getAngle(galvano_status_t* p_galvano) // 반환값이 현재 각도의 값이다.
{
  return p_galvano->current_angle;
}
void setZeroAngle(galvano_status_t* p_galvano) // 현재 각도를 0으로 설정.
{
  p_galvano->current_angle =0;
}

#ifdef _USE_HW_CLI
void getStatus(galvano_status_t* p_galvano) // display status (angle, speed, direction)
{
  cliPrintf("Current angle     : %d\n", p_galvano->current_angle);
}
#endif

#ifdef _USE_HW_CLI
void cligalvano(cli_args_t *args)
{
  bool ret = false;
  if (args->argc == 2 && args->isStr(0, "rotation") == true) //"galvano", "rot", degree:10
    {
      uint32_t degree;
      degree = (uint32_t)args->getData(1);
      setRotation(&galvano_status, degree);
      cliPrintf("Current angle is %d\n", galvano_status.current_angle);
      ret= true;
    }

  if (args->argc == 1 && args->isStr(0, "zero") == true) //"galvano", "zero"
    {
      setZeroAngle(&galvano_status);
      cliPrintf("Current angle is %d\n", galvano_status.current_angle);
      ret = true;
    }

  if (args->argc == 1 && args->isStr(0, "info") == true) //"galvano", "info"
    {
      getStatus(&galvano_status);
      ret = true;
    }



  if (ret != true)
    {
      cliPrintf("galvano rotation degree[0~360]\n");
      cliPrintf("galvano zero\n");
      cliPrintf("galvano info\n");
    }
}
#endif

void galvanoGUIRun()
{
  /*INSTUCTION
0x01 : rotate angle
0x04 : set zeroAngle
0x05 : get angle
0x55 : open */

  if (galvano.driver.available(_DEF_GALVANO1) > 0)
    {
      galvanoReceivePacket(&galvano);
    }

  if (galvano_status.flagGUI == true)
    {
      switch (packet.inst)
      {
	case INST_SET_ANGLE:
	  setRotation(&galvano_status, packet.param);
	break;

	case INST_SET_ZERO:
	  galvano_status.current_angle = 0;
	break;

      }
      galvano_status.flagGUI = false;
    }
}

#endif
