#ifndef NLINK_IOT_FRAME0_H
#define NLINK_IOT_FRAME0_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "nlink_typedef.h"

#define IOT_FRAME0_NODE_COUNT 10
  typedef struct
  {
    uint32_t uid;
    uint8_t cnt;
    float dis;
    float aoa_angle_horizontal;
    float aoa_angle_vertical;
  } iot_frame0_node_t;

  typedef struct
  {
    const size_t fixed_part_size;
    const uint8_t frame_header;
    const uint8_t function_mark;
    uint32_t uid;
    iot_frame0_node_t nodes[IOT_FRAME0_NODE_COUNT];
    uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
  } iot_frame0_t;

  extern iot_frame0_t g_iot_frame0;

#ifdef __cplusplus
}
#endif
#endif // NLINK_IOT_FRAME0_H
