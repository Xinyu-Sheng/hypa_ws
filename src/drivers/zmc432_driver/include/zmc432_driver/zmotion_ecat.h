#ifndef ZMOTION_ECAT_H
#define ZMOTION_ECAT_H

#include "zmotion.h"
#include "zmcaux.h"

#ifdef __cplusplus
extern "C"
{
#endif

  // 连接类型,
  enum EcatInitErrCode
  {
    WrongNodeNum = -1,     // 节点数目与设置的不一致
    WrongAxisNum = -2,     // 总线轴数与设置的不一致
    NotScanNode = -3,      // 超时时间内未扫描到节点
    EcatStartFailu = -4,   // 总线开启失败
    DcShiftSetFailu = -5,  // DC偏移设置失败
  };

  /**
   * EtherCAT 初始化参数结构。
   * 详细说明见 zmc432_driver README 或上位机示例。
   */
  struct EcatInitInfoSet
  {
    int InitStructFlag;  // 是否使用默认参数
    int LocalAxisId;
    int LocalAxisNum;
    int DriveAxisStart;
    int DriveAxisNum;
    int DriveIoStara;
    int DriveIoSpa;
    int DrivePdoMode[128];
    int DriveEnable;
    int EcatNodeNum;
    int NodeIoId[128];
    int NodeAIoId[128];
    int SysClockMode;
    int DcOffsetFlag[128];
    float DcOffsetTime[128];
  };

  /*******************************************************************************************************************
  Description:   //EtherCat总线扫描
  Input:         //卡链接handle、 槽位号、初始化信息、 超时时间
  Return:        //错误码  -1:节点数目不一致  -2:驱动器轴数对不上
  -3:超时时间内,未扫描到驱动器  -4:总线开启失败  -5：总线扫描失败 说明:
  //该接口会阻塞线程,阻塞时长为超时时间
  *******************************************************************************************************************/
  int32 __stdcall ZAux_BusCmd_EcatScan(ZMC_HANDLE handle, int SlotId,
                                       EcatInitInfoSet EcatInfo,
                                       int ApiOutTime);

#ifdef __cplusplus
}
#endif

#endif  // ZMOTION_ECAT_H
