#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "bsp_referee_uart.h"
#include "remote_control.h"

//??????(???)
unpack_data_t referee_unpack_obj;

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_referee_rc_data_t referee_rc_data_t;

volatile uint32_t referee_uart_rx_cnt = 0;

extern volatile uint32_t rc_update_cnt;
extern RC_ctrl_t rc_ctrl;
extern RC_GET_t rc_sent;

void referee_data_solve(uint8_t *frame);
static void referee_rc_data_process(RC_ctrl_t *rc_ctrl, ext_referee_rc_data_t *referee_data);

void init_referee_struct_data(void)
{
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  memset(&referee_rc_data_t, 0, sizeof(ext_referee_rc_data_t));
}
int temp;
char byte_global = 0;
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;
  temp = fifo_s_used(&referee_fifo);

  while (fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    byte_global = byte;
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
}

void referee_data_solve(uint8_t *frame)
{
  uint16_t cmd_id = 0;

  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id)
  {
  case REFEREE_RC_DATA_ID:
  {
    memcpy(&referee_rc_data_t, frame + index, sizeof(ext_referee_rc_data_t));

    referee_rc_data_process(&rc_ctrl, &referee_rc_data_t);
  }

  default:
  {
    break;
  }
  }
}

// typedef __packed struct // 0x0304 Í¼´«Ò£¿ØÆ÷Êý¾Ý
// {
//   int16_t mouse_x;
//   int16_t mouse_y;
//   int16_t mouse_z;
//   int8_t left_button_down;
//   int8_t right_button_down;
//   uint16_t keyboard_value;
//   uint16_t reserved;
// } ext_referee_rc_data_t;

static void referee_rc_data_process(RC_ctrl_t *rc_ctrl, ext_referee_rc_data_t *referee_data)
{
  
  rc_ctrl->mouse.x =referee_data->mouse_x;
  rc_ctrl->mouse.y =referee_data->mouse_y;
  rc_ctrl->mouse.z =referee_data->mouse_z;
  rc_ctrl->mouse.press_l =referee_data->left_button_down;
  rc_ctrl->mouse.press_r =referee_data->right_button_down;

  rc_ctrl->key.v =referee_data->keyboard_value;
}