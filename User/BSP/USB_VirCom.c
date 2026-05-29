/**
 * @file USB_VirCom.c
 * @author sethome
 * @brief 虚拟串口数据发送
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "usbd_cdc_if.h"
#include "USB_VirCom.h"
#include "crc8_crc16.h"
#include "Stm32_time.h"
#include "fifo.h"
void VirCom_send(uint8_t data[], uint16_t len)
{
  if (CDC_Transmit_HS(data, len) == 1) // 判断发送是否忙
  {
    // USB忙碌时将数据转入缓冲区

    fifo_s_puts(&USB_send_fifo, (char *)data, (int)len);
  }
}

void VirCom_rev(uint8_t data[], uint16_t len)
{
  // if(data[0]==0xA5){
  //   Global.Auto.input.Auto_control_online=100;
  //   decodeMINIPCdata(&fromMINIPC,data,len);
  //   MINIPC_to_STM32();
  // }
}

#include "stdio.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  VirCom_send((uint8_t *)&ch, 1);

  return ch;
}
