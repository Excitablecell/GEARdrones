#include "nlink_linktrack_nodeframe0.h"
#include "nlink_linktrack_nodeframe1.h"
#include "nlink_tofsense_frame0.h"
#include "nlink_tofsensem_frame0.h"
#include "nlink_utils.h"

#pragma pack(1)
typedef struct
{
  uint8_t a;
  uint8_t b;
  uint32_t c;
  double d;
  uint8_t e;
} pack_test_t;
#pragma pack()

void parseTofsensemData(const uint8_t *data, size_t data_length)
{
  if (g_ntsm_frame0.UnpackData(data, data_length))
  {
    printf("TOFSense-M Frame0 %d Pixel data unpack successfully:\r\n",
           g_ntsm_frame0.pixel_count);
    printf("id:%d, system_time:%d\r\n", g_ntsm_frame0.id,
           g_ntsm_frame0.system_time);
    for (int i = 0; i < g_ntsm_frame0.pixel_count; ++i)
    {
      ntsm_frame0_pixel_t *pixel = &g_ntsm_frame0.pixels[i];
      printf("pixel %d: dis:%f, dis_status:%d, signal_strength:%d\r\n", i,
             pixel->dis, pixel->dis_status, pixel->signal_strength);
    }
  }
  else
  {
    printf("parse error\n");
  }
}
int main()
{
  {
    uint32_t check = 1;
    if (*(uint8_t *)(&check) != 1)
    {
      printf("Error: this code must run in little endian.");
      return EXIT_FAILURE;
    }
  }

  if (sizeof(pack_test_t) != 15)
  {
    printf("Error: Pack do not work, pack size:%zu. Contact us for support",
           sizeof(pack_test_t));
    return EXIT_FAILURE;
  }

  uint8_t data[1024];
  size_t data_length;
  {
    const char *string = "57 00 ff 00 c2 45 00 00 80 02 00 00 08 00 ff e6";
    data_length = NLink_StringToHex(string, data);
    if (g_nts_frame0.UnpackData(data, data_length))
    {
      printf("TOFSense Frame0 data unpack successfully:\r\n");
      printf("id:%d, distance:%f\r\n", g_nts_frame0.result.id,
             g_nts_frame0.result.dis);
    }
  }
  {
    const char *string =
        "55 02 42 00 01 00 d1 2c c3 88 02 02 00 09 00 11 22 33 44 55 66 77 "
        "88 99 02 02 25 00 11 12 23 22 32 44 34 54 55 65 67 76 67 87 77 99 "
        "aa a2 13 45 57 65 56 56 56 56 57 78 43 33 34 44 44 44 44 46 76 0d";

    data_length = NLink_StringToHex(string, data);
    if (g_nlt_nodeframe0.UnpackData(data, data_length))
    {
      nlt_nodeframe0_result_t *result = &g_nlt_nodeframe0.result;
      printf("LinkTrack NodeFrame0 data unpack successfully:\r\n");
      printf("valid_node_count:%d\r\n", result->valid_node_count);
      for (int i = 0; i < result->valid_node_count; ++i)
      {
        nlt_nodeframe0_node_t *node = result->nodes[i];
        printf("role:%d, id:%d, data_length:%d\r\n", node->role, node->id,
               node->data_length);
      }
    }
  }
  {
    const char *string =
        "55 03 44 00 03 00 e8 80 00 00 00 86 00 00 01 51 01 e8 01 f2 02 02 92 "
        "09 48 13 02 02 00 5f 0b 00 86 09 00 9b ff ff c9 37 8a 34 06 ee "
        "37 3f aa 02 02 93 09 00 45 09 00 c4 fc ff 8d 09 00 66 09 00 c4 fc "
        "ff 8e";
    data_length = NLink_StringToHex(string, data);
    if (g_nlt_nodeframe1.UnpackData(data, data_length))
    {
      nlt_nodeframe1_result_t *result = &g_nlt_nodeframe1.result;
      printf("LinkTrack NodeFrame1 data unpack successfully:\r\n");
      printf("id:%d, system_time:%d, valid_node_count:%d\r\n", result->id,
             result->system_time, result->valid_node_count);
      for (int i = 0; i < result->valid_node_count; ++i)
      {
        nlt_nodeframe1_node_t *node = result->nodes[i];
        printf("role:%d, id:%d, x:%f, y:%f\r\n", node->role, node->id,
               node->pos_3d[0], node->pos_3d[1]);
      }
    }
  }

  {
    const char *string =
        "57 01 ff 00 7d 08 00 00 10 e0 2e 00 05 7b 79 e0 2e 00 05 17 6f e0 2e "
        "00 05 ea 58 20 4e 00 05 aa 8d 60 6d 00 05 ce 94 90 65 00 05 ba 88 a8 "
        "61 00 05 54 78 a8 61 00 05 4d 45 d0 84 00 ff 26 22 60 6d 00 ff 31 26 "
        "a8 61 00 ff c1 26 d8 59 00 ff 27 1e d8 59 00 ff ed 0f 08 52 00 ff 3a "
        "13 08 52 00 ff ca 11 38 4a 00 ff 4b 0f ff ff ff ff ff ff 72";
    data_length = NLink_StringToHex(string, data);
    parseTofsensemData(data, data_length);
  }

  {
    const char *string =
        "57 01 ff 00 17 56 00 00 40 1d 00 00 05 a1 84 20 00 00 05 cf 69 20 00 "
        "00 05 9e 75 21 00 00 05 99 73 20 00 00 05 b0 72 1f 00 00 05 73 51 1f "
        "00 00 05 7c c3 1e 00 00 ff 57 40 21 00 00 05 9a d5 22 00 00 05 71 dc "
        "26 00 00 05 ee e7 22 00 00 05 d6 de 23 00 00 05 6a ef 22 00 00 05 f8 "
        "e5 22 00 00 05 84 9b 20 00 00 ff 23 30 27 00 00 05 74 5b 29 00 00 05 "
        "b6 70 27 00 00 05 e2 5c 27 00 00 05 04 4e 27 00 00 05 ec 4c 24 00 00 "
        "05 57 4d 23 00 00 ff f0 3a 21 00 00 ff ff 20 2b 00 00 ff 3c 25 29 00 "
        "00 ff 6f 2e 29 00 00 ff de 26 28 00 00 ff a7 22 26 00 00 ff 57 23 23 "
        "00 00 ff 91 27 23 00 00 ff 05 23 21 00 00 ff 34 15 27 00 00 ff 61 13 "
        "27 00 00 ff eb 18 24 00 00 ff 8a 18 24 00 00 ff 86 19 22 00 00 ff 13 "
        "1c 22 00 00 ff a1 1e 21 00 00 ff 3b 18 1f 00 00 ff 47 0c 23 00 00 ff "
        "ff 0b 22 00 00 ff 35 0e 20 00 00 ff 8b 10 20 00 00 ff 80 11 20 00 00 "
        "ff e8 11 20 00 00 ff 22 10 1e 00 00 ff 7d 0b 1e 00 00 ff 0b 0b 1e 00 "
        "00 ff 4a 09 1e 00 00 ff b7 0a 1f 00 00 ff 2d 0d 1e 00 00 ff 1c 0f 1d "
        "00 00 ff 06 10 1e 00 00 ff 21 0d 1e 00 00 ff 89 0b 17 00 00 ff 4b 17 "
        "1c 00 00 ff 9d 09 1c 00 00 ff 43 0d 1d 00 00 ff d5 0c 1b 00 00 ff aa "
        "0e 1c 00 00 ff 29 0f 1a 00 00 ff 57 0d 18 00 00 ff d1 0f 13 00 00 05 "
        "a4 5c ff ff ff ff ff ff 05";
    data_length = NLink_StringToHex(string, data);
    parseTofsensemData(data, data_length);
  }

  return EXIT_SUCCESS;
}
