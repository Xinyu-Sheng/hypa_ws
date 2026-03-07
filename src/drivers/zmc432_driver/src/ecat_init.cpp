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

int32 __stdcall ZAux_BusCmd_EcatInit(ZMC_HANDLE handle, int SlotId,
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

    // --- begin full mapping & configuration (ported from Windows sample) ---
    // count total bus axes and optionally verify against DriveAxisNum
    int BusAxisNum = 0;
    int NodeAxisNum = 0;
    // get servo period for DC offset checks later
    int ServoPeriod = 0;
    Iresult += ZAux_Execute(handle, "?SERVO_PERIOD", ReceBuff, 256);
    ServoPeriod = std::atoi(ReceBuff);

    for (int i = 0; i < ScanNodeNum; i++)
    {
      sprintf(cmdbuff, "?NODE_AXIS_COUNT(%d,%d)", SlotId, i);
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      NodeAxisNum = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 0)", SlotId, i);
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      int Drive_Vender = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 1)", SlotId, i);
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      int Drive_Device = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 3)", SlotId, i);
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      // alias not used in current implementation

      // DC offset verification
      if (EcatInfo.DcOffsetFlag[i] == 1)
      {
        int ZmlInfo, NodeInfo;
        sprintf(cmdbuff, "?ZML_INFO(19,%d,%d)", Drive_Vender, Drive_Device);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        ZmlInfo = std::atoi(ReceBuff);

        sprintf(cmdbuff, "?NODE_INFO(%d,%d,19)", SlotId, i);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        NodeInfo = std::atoi(ReceBuff);
        if (((ServoPeriod * 1000 * EcatInfo.DcOffsetTime[i] - ZmlInfo) > 5) ||
            (ZmlInfo != NodeInfo))
        {
          return DcShiftSetFailu;
        }
      }

      // iterate each axis in the node
      for (int j = 0; j < NodeAxisNum; j++)
      {
        // address mapping and axis type
        Iresult += ZAux_Direct_SetAxisAddress(
            handle, EcatInfo.DriveAxisStart + BusAxisNum, BusAxisNum + 1);
        Iresult += ZAux_Direct_SetAtype(
            handle, EcatInfo.DriveAxisStart + BusAxisNum, 65);

        // PDO configuration
        if ((Drive_Device == ElmoDevice) && (Drive_Vender == ElmoVender))
        {
          // special Elmo handling
          Iresult += ZAux_Execute(
              handle, "SYSTEM_ZSET = CLEAR_BIT(7, SYSTEM_ZSET)", ReceBuff, 256);
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = -1",
                  EcatInfo.DriveAxisStart, BusAxisNum);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          sprintf(cmdbuff, " NODE_PROFILE(%d,%d) = -1", SlotId, i);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          // Elmo custom PDO via SDO writes (omitted here, user may add if
          // needed)
        }
        else if ((Drive_Device == 0x1ab0) && (Drive_Vender == 0x41B))
        {
          // 正运动脉冲扩展卡
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = 0",
                  EcatInfo.DriveAxisStart, BusAxisNum);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        }
        else
        {
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = %d",
                  EcatInfo.DriveAxisStart, BusAxisNum,
                  EcatInfo.DrivePdoMode[BusAxisNum]);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          // IO mapping for modes that use hardware limits/origin
          if ((4 == EcatInfo.DrivePdoMode[BusAxisNum]) ||
              (5 == EcatInfo.DrivePdoMode[BusAxisNum]) ||
              (12 == EcatInfo.DrivePdoMode[BusAxisNum]))
          {
            int StartIdTemp =
                EcatInfo.DriveIoStara + EcatInfo.DriveIoSpa * (BusAxisNum);
            sprintf(cmdbuff, " DRIVE_IO(%d + %d) = %d", EcatInfo.DriveAxisStart,
                    BusAxisNum, StartIdTemp);
            Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
            Iresult += ZAux_Direct_SetRevIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp, 1);
            Iresult += ZAux_Direct_SetFwdIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp + 1);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp + 1, 1);
            Iresult += ZAux_Direct_SetDatumIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp + 2);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp + 2, 1);
          }
          else if (EcatInfo.DrivePdoMode[BusAxisNum] < 4)
          {
            int TempVar = 0;
            Iresult += ZAux_Direct_GetRevIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, &TempVar);
            if (TempVar >= EcatInfo.DriveIoStara)
            {
              Iresult += ZAux_Direct_SetRevIn(
                  handle, EcatInfo.DriveAxisStart + BusAxisNum, -1);
              Iresult += ZAux_Direct_SetInvertIn(handle, TempVar, 0);
            }
            Iresult += ZAux_Direct_GetFwdIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, &TempVar);
            if (TempVar >= EcatInfo.DriveIoStara)
            {
              Iresult += ZAux_Direct_SetFwdIn(
                  handle, EcatInfo.DriveAxisStart + BusAxisNum, -1);
              Iresult += ZAux_Direct_SetInvertIn(handle, TempVar, 0);
            }
            Iresult += ZAux_Direct_GetDatumIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, &TempVar);
            if (TempVar >= EcatInfo.DriveIoStara)
            {
              Iresult += ZAux_Direct_SetDatumIn(
                  handle, EcatInfo.DriveAxisStart + BusAxisNum, -1);
              Iresult += ZAux_Direct_SetInvertIn(handle, TempVar, 0);
            }
          }
        }

        // disable group for single-axis alarm handling
        sprintf(cmdbuff, " DISABLE_GROUP(%d)",
                EcatInfo.DriveAxisStart + BusAxisNum);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

        BusAxisNum++;
      }

      // node IO/AIO mapping
      if (EcatInfo.NodeIoId[i] >= 32)
      {
        sprintf(cmdbuff, "NODE_IO(%d, %d) = %d", SlotId, i,
                EcatInfo.NodeIoId[i]);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
      if (EcatInfo.NodeAIoId[i] > 0)
      {
        sprintf(cmdbuff, "NODE_AIO(%d, %d) = %d", SlotId, i,
                EcatInfo.NodeAIoId[i]);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
    }

    // verify drive axis num if provided
    if (EcatInfo.DriveAxisNum >= 0 && BusAxisNum != EcatInfo.DriveAxisNum)
    {
      return WrongAxisNum;
    }

    // 4. start bus and wait for response
    MyDelayMs(100, &OutTime);
    sprintf(cmdbuff, "SLOT_START(%d, 4)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    MyDelayMs(1000, &OutTime);
    EcatScanOutTime = OutTime;
    sprintf(cmdbuff, "SLOT_START(%d, 8)  ?return", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ReceBuff[2] = 0;
    if (0 == strcmp("-1", ReceBuff))
    {
      EcatScanOutTime = 0;
    }
    else
    {
      MyDelayMs(500, &OutTime);
      EcatScanOutTime = EcatScanOutTime - 500;
    }
    while (EcatScanOutTime > 0)
    {
      ZMC_ExecuteGetReceive(handle, ReceBuff, 1000, &puiread, &pbifExcuteDown);
      if (ReceBuff[0] != 0)
      {
        ReceBuff[2] = 0;
        if (0 == strcmp("-1", ReceBuff))
        {
          break;
        }
      }
      MyDelayMs(50, &OutTime);
      EcatScanOutTime = EcatScanOutTime - 50;
    }
    if (0 == strcmp("-1", ReceBuff))
    {
      MyDelayMs(3000, &OutTime);
      // clear alarms and enable axes if requested
      for (int Drivei = EcatInfo.DriveAxisStart;
           Drivei < (EcatInfo.DriveAxisStart + BusAxisNum); Drivei++)
      {
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=128 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=6 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=15 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
      }
      MyDelayMs(10, &OutTime);
      ZAux_Direct_Single_Datum(handle, 0, 0);
      MyDelayMs(200, &OutTime);
      Iresult += ZAux_Execute(handle, "WDOG=1", ReceBuff, 256);
      if (EcatInfo.DriveEnable == 1)
      {
        for (int Drivei = EcatInfo.DriveAxisStart;
             Drivei < (EcatInfo.DriveAxisStart + BusAxisNum); Drivei++)
        {
          ZAux_Direct_SetAxisEnable(handle, Drivei, 1);
          MyDelayMs(10, &OutTime);
        }
      }
      return Iresult;
    }
    else
    {
      return EcatStartFailu;
    }
  }
  // if we reach here, no driv ers were found
  return NotScanNode;
}
