#include "zmc432_driver/zmotion_ecat.h"
#include "zmc432_driver/zmotion.h"
#include "zmc432_driver/zmcaux.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>

static void MyDelayMs(int DelayTiem, int* pOutTime)
{
  usleep(DelayTiem * 1000);
  if (pOutTime)
  {
    *pOutTime = *pOutTime - DelayTiem;
  }
}

int ZAux_BusCmd_SlotScan(ZMC_HANDLE handle, int SlotId, int* pOutTime)
{
  uint32 puiread;
  uint8 pbifExcuteDown;
  char ReceBuff[256] = {0};
  char cmdbuff[2048] = {0};
  int Iresult = 0;
  int ScanOkFlag = 0;
  sprintf(cmdbuff, "SLOT_STOP(%d)", SlotId);
  ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  MyDelayMs(200, pOutTime);
  sprintf(cmdbuff, "SLOT_SCAN(%d) ?return", SlotId);
  Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  if ((Iresult == 0) || (Iresult == 20003) || (Iresult == 3402))
  {
    ReceBuff[2] = 0;
    if (0 == strcmp("-1", ReceBuff))
    {
      ScanOkFlag = 1;
      return ScanOkFlag;
    }
    else if (ReceBuff[0] != 0)
    {
      ScanOkFlag = 0;
      return ScanOkFlag;
    }
    else
    {
      MyDelayMs(500, pOutTime);
    }
    while (*pOutTime > 0)
    {
      Iresult += ZMC_ExecuteGetReceive(handle, ReceBuff, 1000, &puiread,
                                       &pbifExcuteDown);
      if ((ReceBuff[0] != 0) &&
          ((Iresult == 0) || (Iresult == 20003) || (Iresult == 3402)))
      {
        ReceBuff[2] = 0;
        if (0 == strcmp("-1", ReceBuff))
        {
          ScanOkFlag = 1;
          break;
        }
        else
        {
          ScanOkFlag = 0;
          break;
        }
      }
      MyDelayMs(50, pOutTime);
    }
    return ScanOkFlag;
  }
  return 0;
}

int32 __stdcall ZAux_BusCmd_EcatScan(ZMC_HANDLE handle, int SlotId,
                                     EcatInitInfoSet EcatInfo, int ApiOutTime)
{
  int32 Iresult = 0;
  int OutTime = ApiOutTime;
  int32 EcatScanOutTime = ApiOutTime;
  int32 IdleOutTime = 100;
  float ScanNodeNum = 0;
  uint32 puiread;
  uint8 pbifExcuteDown;
  char ReceBuff[256];
  char cmdbuff[2048];

  if (EcatInfo.InitStructFlag == 1)
  {
    EcatInfo.LocalAxisId = 0;
    EcatInfo.LocalAxisNum = 0;
    EcatInfo.DriveAxisStart = 0;
    EcatInfo.DriveAxisNum = -1;
    EcatInfo.DriveIoStara = 256;
    EcatInfo.DriveIoSpa = 16;
    memset(EcatInfo.DrivePdoMode, 12, sizeof(EcatInfo.DrivePdoMode));
    EcatInfo.DriveEnable = 1;
    EcatInfo.EcatNodeNum = -1;
    memset(EcatInfo.NodeIoId, 0, sizeof(EcatInfo.NodeIoId));
    memset(EcatInfo.NodeAIoId, 0, sizeof(EcatInfo.NodeAIoId));
    EcatInfo.SysClockMode = 1;
    memset(EcatInfo.DcOffsetFlag, 0, sizeof(EcatInfo.DcOffsetFlag));
    memset(EcatInfo.DcOffsetTime, 0, sizeof(EcatInfo.DcOffsetTime));
  }

  int ElmoVender = 0x9a;
  int ElmoDevice = 0x30924;

  Iresult = ZAux_Direct_Rapidstop(handle, 2);

  uint16 VirtualAxiseNum = 0;
  uint8 MotionAxisNum = 0;
  uint8 IoNum[4];
  ZAux_GetSysSpecification(handle, &VirtualAxiseNum, &MotionAxisNum, IoNum);

  for (int i = 0; i < VirtualAxiseNum; i++)
  {
    Iresult += ZAux_Direct_SetAxisAddress(handle, i, 0);
    Iresult += ZAux_Direct_SetAxisEnable(handle, i, 0);
    Iresult += ZAux_Direct_SetAtype(handle, i, 0);
    int Idle = 0;
    while (IdleOutTime > 0)
    {
      Iresult = ZAux_Direct_GetIfIdle(handle, i, &Idle);
      if (Idle == -1)
      {
        break;
      }
      MyDelayMs(10, &OutTime);
      IdleOutTime -= 10;
    }
  }

  for (int i = 0; i < EcatInfo.LocalAxisNum; i++)
  {
    Iresult += ZAux_Direct_SetAxisAddress(handle, EcatInfo.LocalAxisId + i,
                                          (-1 << 16) + i);
    Iresult += ZAux_Direct_SetAtype(handle, EcatInfo.LocalAxisId + i, 1);
  }
  if (EcatInfo.SysClockMode == 1)
  {
    Iresult += ZAux_Execute(handle, "SYSTEM_ZSET = SET_BIT(7, SYSTEM_ZSET)",
                            ReceBuff, 256);
  }
  else
  {
    Iresult += ZAux_Execute(handle, "SYSTEM_ZSET = CLEAR_BIT(7, SYSTEM_ZSET)",
                            ReceBuff, 256);
  }

  if (ERR_OK != Iresult)
  {
    return Iresult;
  }

  int ScanOkFlag = 0;
  for (int i = 0; i < 3; i++)
  {
    ScanOkFlag = ZAux_BusCmd_SlotScan(handle, SlotId, &OutTime);
    Iresult = 0;
    if (1 == ScanOkFlag) break;
  }
  if (ScanOkFlag == 1)
  {
    sprintf(cmdbuff, "?NODE_COUNT(%d)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ScanNodeNum = std::atoi(ReceBuff);
    for (int i = 0; i < ScanNodeNum; i++)
    {
      if (EcatInfo.DcOffsetFlag[i] == 1)
      {
        sprintf(cmdbuff, "?NODE_INFO(%d,%d, 0)", SlotId, i);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        int Drive_Vender = std::atoi(ReceBuff);

        sprintf(cmdbuff, "?NODE_INFO(%d,%d, 1)", SlotId, i);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        int Drive_Device = std::atoi(ReceBuff);

        sprintf(cmdbuff, "ZML_INFO(19, %d, %d) = SERVO_PERIOD * %f * 1000",
                Drive_Vender, Drive_Device, EcatInfo.DcOffsetTime[i]);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
    }
  }
  for (int i = 0; i < 3; i++)
  {
    ScanOkFlag = ZAux_BusCmd_SlotScan(handle, SlotId, &OutTime);
    Iresult = 0;
    if (1 == ScanOkFlag) break;
  }
  if (1 == ScanOkFlag)
  {
    sprintf(cmdbuff, "?NODE_COUNT(%d)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ScanNodeNum = std::atoi(ReceBuff);
    if (EcatInfo.EcatNodeNum >= 0)
    {
      if ((int)ScanNodeNum != EcatInfo.EcatNodeNum)
      {
        return WrongNodeNum;
      }
    }

    // simplified: full mapping omitted
    // application can implement additional configuration as needed
  }
  return 0;
}
