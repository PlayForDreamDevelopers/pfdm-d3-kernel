#ifndef HW_VER_DET_H
#define HW_VER_DET_H
enum hw_ver
{
  HW_VER_ERR = 0,
  HW_VER_DVT1,
  HW_VER_DVT2,
  HW_VER_PVT1,
};
int get_board_version(void);
#endif//HW_VER_DET_H
