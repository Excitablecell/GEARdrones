#include "nlink_iot_frame0.h"

#include "nlink_utils.h"

#pragma pack(1)
typedef struct
{
  uint32_t uid;
  uint8_t cnt;
  nint24_t dis;
  nint24_t aoa_angle_horizontal;
  nint24_t aoa_angle_vertical;
} iot_frame0_raw_node_t;

typedef struct
{
  uint8_t frame_header;
  uint8_t function_mark;
  uint32_t uid;
  iot_frame0_raw_node_t nodes[IOT_FRAME0_NODE_COUNT];
  uint8_t reserved1[6];
  uint8_t sum;
} iot_raw_frame0_t;
#pragma pack()

static uint8_t UnpackData(const uint8_t *data, size_t data_length)
{
  const iot_raw_frame0_t *frame0 = (const iot_raw_frame0_t *)data;
  if (frame0->frame_header != g_iot_frame0.frame_header ||
      frame0->function_mark != g_iot_frame0.function_mark ||
      data_length < g_iot_frame0.fixed_part_size)
  {
    return 0;
  }

  if (g_iot_frame0.fixed_part_size != data_length)
  {
    return 0;
  }

  if (!NLINK_VerifyCheckSum(data, data_length))
  {
    return 0;
  }

  g_iot_frame0.uid = frame0->uid;
  for (int i = 0; i < IOT_FRAME0_NODE_COUNT; ++i)
  {
    g_iot_frame0.nodes[i].uid = frame0->nodes[i].uid;
    g_iot_frame0.nodes[i].cnt = frame0->nodes[i].cnt;
    g_iot_frame0.nodes[i].dis =
        NLINK_ParseInt24(frame0->nodes[i].dis) / 1000.0f;
    g_iot_frame0.nodes[i].aoa_angle_horizontal =
        NLINK_ParseInt24(frame0->nodes[i].aoa_angle_horizontal) / 1000.0f;
    g_iot_frame0.nodes[i].aoa_angle_vertical =
        NLINK_ParseInt24(frame0->nodes[i].aoa_angle_vertical) / 1000.0f;
  }
  return 1;
}

iot_frame0_t g_iot_frame0 = {.fixed_part_size = 160,
                               .frame_header = 0x55,
                               .function_mark = 0x01,
                               .UnpackData = UnpackData};
