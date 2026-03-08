#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "zmc432_driver/zmcaux.h"
#include "zmc432_driver/zmotion.h"
#include "zmc432_driver/zmotion_ecat.h"

/**************************************************************************************************
Description:   //延时函数
Input:         //延时时间ms:DelayTiem

Return:        //
说明:           //
***************************************************************************************************/
static void MyDelayMs(int DelayTiem, int *pOutTime)
{
  usleep(DelayTiem * 1000);
  if (pOutTime)
  {
    *pOutTime = *pOutTime - DelayTiem;
  }
}

/**************************************************************************************************
Description:   //总线扫描
Input:         //SlotId：要进行总线扫描的总线槽ID，Ecat总线槽ID是0
               //pOutTime：超时时间
Return:        //
说明:		   return =1 表示有正常扫描的驱动器
***************************************************************************************************/
int ZAux_BusCmd_SlotScan(ZMC_HANDLE handle, int SlotId, int *pOutTime)
{
  uint32 puiread;
  uint8 pbifExcuteDown;
  char ReceBuff[256] = {0};
  char cmdbuff[2048] = {0};
  int Iresult = 0;
  int ScanOkFlag = 0;
  // 停止总线
  sprintf(cmdbuff, "SLOT_STOP(%d)", SlotId);
  ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  printf("DEBUG SLOT_STOP executed, ReceBuff='%s'\n", ReceBuff);
  // 等待200ms
  MyDelayMs(200, pOutTime);
  // 扫描总线
  sprintf(cmdbuff, "SLOT_SCAN(%d) ?return", SlotId);
  Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  printf("DEBUG after SLOT_SCAN: Iresult=%d, ReceBuff='%s', condition=%d\n",
         Iresult, ReceBuff,
         ((Iresult == 0) || (Iresult == 20003) || (Iresult == 3402)));
  if ((Iresult == 0) || (Iresult == 20003) || (Iresult == 3402))
  {
    // 延时等待扫描结果
    ReceBuff[2] = 0;
    printf("DEBUG post trim ReceBuff='%s' first_byte=0x%02x\n", ReceBuff,
           (unsigned char)ReceBuff[0]);
    if (0 == strcmp("-1", ReceBuff))
    {
      ScanOkFlag = 1;
      return ScanOkFlag;
    }
    else if (ReceBuff[0] != 0)
    {
      printf("DEBUG ReceBuff[0]!=0 early, ReceBuff='%s', returning 0\n",
             ReceBuff);
      ScanOkFlag = 0;
      return ScanOkFlag;
    }
    else
    {
      printf("DEBUG ReceBuff empty, will wait 500ms\n");
      MyDelayMs(500, pOutTime);
    }
    while (*pOutTime > 0)
    {
      // 读取在线命令的应答， 对没有接收应答的命令有用
      Iresult += ZMC_ExecuteGetReceive(handle, ReceBuff, 1000, &puiread,
                                       &pbifExcuteDown);
      printf("DEBUG while recv: ReceBuff='%s', Iresult=%d, timeout=%d\n",
             ReceBuff, Iresult, *pOutTime);
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

/*****************************************************************************************************************************
Description:   //EtherCat总线扫描
Input:
//卡链接:handle、槽位号:SlotId、总线初始化信息:EcatInfo、超时时间:ApiOutTime
Return:        //错误码  小于0的错误码：参考枚举类型EcatInitErrCode
等于0：总线初始化成功  大于0：对多个zaux接口错误码的累加值 说明:
//该接口会阻塞线程,阻塞时长为超时时间
*****************************************************************************************************************************/
int32 __stdcall ZAux_BusCmd_EcatInit(ZMC_HANDLE handle, int SlotId,
                                     EcatInitInfoSet EcatInfo, int ApiOutTime)
{
  int32 Iresult = 0;                   // 接口返回值，会累加
  int OutTime = ApiOutTime;            // 整个接口的超时时间
  int32 EcatScanOutTime = ApiOutTime;  // 总线扫描超时时间
  int32 IdleOutTime = 100;             // 等待轴停止的超时时间
  float ScanNodeNum = 0;               // 扫描到的节点数
  uint32 puiread;
  uint8 pbifExcuteDown;
  char ReceBuff[256];
  char cmdbuff[2048];
  /* debug */
  printf("DEBUG ZAux_BusCmd_EcatInit slot=%d ApiOutTime=%d\n", SlotId,
         ApiOutTime);
  // EtherCAT 诊断：查询模块状态
  printf("[ecat_init] querying ECAT_STATUS...\n");
  sprintf(cmdbuff, "?ECAT_STATUS");
  int diag_ret = ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  printf("[ecat_init] ECAT_STATUS returned ret=%d resp='%s'\n", diag_ret,
         ReceBuff);
  // 变量定义
  float TableData = 0;
  int Drive_Vender, Drive_Device, Drive_Alias;

  // 是否初始化Ecat初始化数据
  if (EcatInfo.InitStructFlag == 1)
  {
    // 【1、本地脉冲轴配置：如果控制器本体脉冲轴有使用的可以配置】
    EcatInfo.LocalAxisId =
        0;  // 控制器本地脉冲轴起始轴号[默认参数0]，控制器本体脉冲不使用的配置成0即可。
    EcatInfo.LocalAxisNum =
        0;  // 控制器本地脉冲轴使用的数目[默认参数0]，控制器本体脉冲不使用的配置成0即可。
    // 【2、总线轴使用配置：用户可自定义配置】
    EcatInfo.DriveAxisStart =
        0;  // 总线轴起始编号[默认参数0]，配置后第1个驱动器的轴号为DriveAxisStart，第2个驱动器的轴号为DriveAxisStart+1，以此类推
    EcatInfo.DriveAxisNum =
        -1;  // 总线驱动器个数[默认参数-1] -1：表示不判断总线轴的个数
             // 大于等于0：表示会检测设置的轴数与扫描到的总线轴的个数是否一致
    EcatInfo.DriveIoStara =
        256;  // 总线驱动器IO映射到控制器上后当前驱动器IO的起始编号，IO起始地址需要是8的倍数，不能和其他IO地址冲突
    EcatInfo.DriveIoSpa =
        16;  // 一个驱动器映射多少个IO到控制器上，需要是8的倍数
    memset(
        EcatInfo.DrivePdoMode, 12,
        sizeof(
            EcatInfo
                .DrivePdoMode));  // 轴PDO模式，详情参考RTBasic手册的drive_profile的指令说明,默认设置成12表示需要监控驱动器的PDO
    EcatInfo.DriveEnable =
        1;  ////总线初始化后驱动器是否自动上使能，1自动上使能，0不上使能

    // 【3、总线节点的使用配置：用户可自定义配置】
    EcatInfo.EcatNodeNum =
        -1;  // ECAT网络中从站的数目 -1：表示不判断从站的数目
             // 大于等于0：表示会检测设置的从站数目与扫描到的从站个数是否一致
    memset(
        EcatInfo.NodeIoId, 0,
        sizeof(
            EcatInfo
                .NodeIoId));  // 各个节点的IO起始地址,数组下标即是节点ID，需要是8的倍数
    memset(
        EcatInfo.NodeAIoId, 0,
        sizeof(
            EcatInfo.NodeAIoId));  // 各个节点的AIO起始地址,数组下标即是节点ID
    // 【4、DC同步时钟和DC偏移：用户可自定义配置】
    EcatInfo.SysClockMode =
        1;  // 系统时钟模式 1：打开DC同步时钟， 0：关闭DC同步时钟
    memset(
        EcatInfo.DcOffsetFlag, 0,
        sizeof(
            EcatInfo
                .DcOffsetFlag));  // 第N个节点类型的驱动器是否需要打开DC偏移功能，0：关闭DC偏移，1：打开DC偏移
                                  // ,数组下标即是节点ID
    memset(
        EcatInfo.DcOffsetTime, 0,
        sizeof(
            EcatInfo
                .DcOffsetTime));  // 第N个节点类型的驱动器DC偏移的时间，单位是总线周期，设置0.5即是0.5个总线周期,数组下标即是节点ID
  }

  // 【伺服驱动器厂商ID】
  int ElmoVender = 0x9a;     // Elmo驱动器的厂商Id
  int ElmoDevice = 0x30924;  // Elmo驱动器的设备Id

  // ═══════════════════════════════════════════════════════════════════
  // 【步骤 1】清除当前设置：标注在 ZAux_Direct_Rapidstop 和轴参数重置循环处
  // 操作目标：控制器内核
  // 被改主体：虚拟轴配置、运动缓冲
  // 说明：紧急停止所有轴，清空旧运动指令和配置
  // ═══════════════════════════════════════════════════════════════════
  Iresult = ZAux_Direct_Rapidstop(handle, 2);

  // 读取控制器规格
  uint16 VirtualAxiseNum = 0;  // 虚拟轴数目
  uint8 MotionAxisNum = 0;     // 实轴数
  uint8 IoNum[4];              // Io数目
  ZAux_GetSysSpecification(handle, &VirtualAxiseNum, &MotionAxisNum, IoNum);

  // 【步骤 1 续】轴参数重置循环（第 130 行起）
  // 清空所有虚拟轴的配置：地址、使能、类型
  // 确保内存参数不干扰后续配置
  for (int i = 0; i < VirtualAxiseNum; ++i)
  {
    Iresult += ZAux_Direct_SetAxisAddress(handle, i, 0);
    Iresult += ZAux_Direct_SetAxisEnable(handle, i, 0);
    Iresult += ZAux_Direct_SetAtype(handle, i, 0);
    // 等待轴停止
    int Idle = 0;
    while (IdleOutTime > 0)
    {
      Iresult = ZAux_Direct_GetIfIdle(handle, i, &Idle);
      if (Idle == -1)
      {
        break;
      }
      MyDelayMs(10, &OutTime);
      IdleOutTime = IdleOutTime - 10;
    }
  }

  // 2、本地轴号重映射
  for (int i = 0; i < EcatInfo.LocalAxisNum; ++i)
  {
    Iresult += ZAux_Direct_SetAxisAddress(
        handle, EcatInfo.LocalAxisId + i,
        (-1 << 16) + i);  // 将本地轴0-->i映射到轴LocalAxisId-->LocalAxisId+i
    Iresult += ZAux_Direct_SetAtype(handle, EcatInfo.LocalAxisId + i, 1);
  }
  // 3、DC同步时钟的设置
  if (EcatInfo.SysClockMode == 1)
  {
    Iresult += ZAux_Execute(handle, "SYSTEM_ZSET = SET_BIT(7, SYSTEM_ZSET)",
                            ReceBuff, 256);  // 打开总线时钟优化
  }
  else
  {
    Iresult += ZAux_Execute(handle, "SYSTEM_ZSET = CLEAR_BIT(7, SYSTEM_ZSET)",
                            ReceBuff, 256);  // 关闭总线时钟优化
  }

  if (ERR_OK != Iresult)
  {
    printf("DEBUG ZAux_BusCmd_EcatInit early exit, Iresult=%d\n", Iresult);
    // 阶段错误码拦截！
    return Iresult;
  }

  // ═══════════════════════════════════════════════════════════════════
  // 【步骤 2 前置】总线状态完全复位
  // 操作目标：EtherCAT总线
  // 说明：确保每次调用都从干净状态开始，避免重复调用时从站状态机未复位
  // ═══════════════════════════════════════════════════════════════════
  sprintf(cmdbuff, "SLOT_STOP(%d)", SlotId);
  ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
  MyDelayMs(100, &OutTime);

  // ═══════════════════════════════════════════════════════════════════
  // 【步骤 2】SLOT_SCAN 物理扫描
  // 操作目标：控制器 + EtherCAT总线
  // 被改主体：总线设备表
  // 说明：检查网线连接，扫描网络上的所有从站（驱动器）设备
  // ═══════════════════════════════════════════════════════════════════
  int ScanOkFlag = 0;
  for (int i = 0; i < 3; ++i)
  {
    printf("DEBUG scan loop attempt %d\n", i);
    ScanOkFlag = ZAux_BusCmd_SlotScan(handle, SlotId, &OutTime);
    printf("DEBUG scan loop result ScanOkFlag=%d\n", ScanOkFlag);
    Iresult = 0;
    if (1 == ScanOkFlag)
      break;
  }
  if (ScanOkFlag == 1)  // 如果有扫描到驱动器
  {
    // 【节点数目判断】
    sprintf(cmdbuff, "?NODE_COUNT(%d)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ScanNodeNum = std::atoi(ReceBuff);
    printf("DEBUG scanned node count=%f\n", ScanNodeNum);
    for (int i = 0; i < ScanNodeNum; ++i)
    {
      // 判断是否需要设置DC偏移时间
      if (EcatInfo.DcOffsetFlag[i] == 1)
      {
        sprintf(cmdbuff, "?NODE_INFO(%d,%d, 0)", SlotId, i);  // 该节点的厂商ID
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        Drive_Vender = std::atoi(ReceBuff);

        sprintf(cmdbuff, "?NODE_INFO(%d,%d, 1)", SlotId,
                i);  // 该节点的设备编号
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        Drive_Device = std::atoi(ReceBuff);

        sprintf(cmdbuff, "ZML_INFO(19, %d, %d) = SERVO_PERIOD * %f * 1000",
                Drive_Vender, Drive_Device,
                EcatInfo.DcOffsetTime[i]);  // DC 偏移时间单位是ns
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
    }
  }
  // 5、DC偏移时间设置后，需要再次扫描总线驱动器
  for (int i = 0; i < 3; ++i)
  {
    ScanOkFlag = ZAux_BusCmd_SlotScan(handle, SlotId, &OutTime);
    Iresult = 0;
    if (1 == ScanOkFlag)
      break;
  }
  // 6、扫描到ECAT从站设备
  if (1 == ScanOkFlag)  // 如果有扫描到驱动器
  {
    // 【7、节点数目判断，看看是否少从站】
    sprintf(cmdbuff, "?NODE_COUNT(%d)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ScanNodeNum = std::atoi(ReceBuff);
    if (EcatInfo.EcatNodeNum >= 0)
    {
      // 判断节点数目是否正确
      if ((int)ScanNodeNum != EcatInfo.EcatNodeNum)
      {
        printf("DEBUG WrongNodeNum expected=%d actual=%d\n",
               EcatInfo.EcatNodeNum, (int)ScanNodeNum);
        // 节点数目不一致
        return WrongNodeNum;
      }
    }

    // 【8、总线轴个数判断，看看轴数是否可以对上】
    int BusAxisNum = 0;   // 总线轴个数
    int NodeAxisNum = 0;  // 当前节点轴个数
    if (EcatInfo.DriveAxisNum >= 0)
    {
      for (int i = 0; i < ScanNodeNum; i++)
      {
        sprintf(cmdbuff, "?NODE_AXIS_COUNT(%d,%d)", SlotId, i);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        NodeAxisNum = std::atoi(ReceBuff);
        BusAxisNum = BusAxisNum + NodeAxisNum;
      }
      // 判断轴数目是否正确
      if (BusAxisNum != EcatInfo.DriveAxisNum)
      {
        printf("DEBUG WrongAxisNum expected=%d actual=%d\n",
               EcatInfo.DriveAxisNum, BusAxisNum);
        // 驱动器轴数目不一致
        return WrongAxisNum;
      }
    }

    // 【9、IO映射和轴映射】
    // 总线轴总数，从0开始计数
    BusAxisNum = 0;
    int ServoPeriod = 0;
    Iresult += ZAux_Execute(handle, "?SERVO_PERIOD", ReceBuff,
                            256);  // 获取总线周期
    ServoPeriod = std::atoi(ReceBuff);
    // 遍历节点
    for (int i = 0; i < ScanNodeNum; i++)
    {
      sprintf(cmdbuff, "?NODE_AXIS_COUNT(%d,%d)", SlotId,
              i);  // 各个节点的轴数
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      NodeAxisNum = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 0)", SlotId,
              i);  // 该节点的厂商ID
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      Drive_Vender = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 1)", SlotId,
              i);  // 该节点的设备编号
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      Drive_Device = std::atoi(ReceBuff);

      sprintf(cmdbuff, "?NODE_INFO(%d,%d, 3)", SlotId,
              i);  // 该节点的设备拨码ID
      Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      Drive_Alias = std::atoi(ReceBuff);

      // 【DC偏移设置成功校验】
      if (EcatInfo.DcOffsetFlag[i] == 1)
      {
        int ZmlInfo, NodeInfo;
        sprintf(cmdbuff, "?ZML_INFO(19,%d,%d)", Drive_Vender, Drive_Device);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        ZmlInfo = std::atoi(ReceBuff);

        sprintf(cmdbuff, "?NODE_INFO(%d,%d,19)", SlotId, i);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        NodeInfo = std::atoi(ReceBuff);
        // DC偏移设置失败
        if (((ServoPeriod * 1000 * EcatInfo.DcOffsetTime[i] - ZmlInfo) > 5) ||
            (ZmlInfo != NodeInfo))
        {
          printf("DEBUG DcShiftSetFailu slot=%d node=%d\n", SlotId, i);
          return DcShiftSetFailu;  // DC偏移设置失败
        }
      }

      // 遍历节点的各个电机
      for (int j = 0; j < NodeAxisNum; j++)
      {
        // ═══════════════════════════════════════════════════════════════════
        // 【步骤 3】AXIS_ADDRESS 逻辑映射（第 247 行）
        // 操作目标：控制器虚拟轴表
        // 被改主体：轴 -> 驱动器映射表
        // 说明：把物理从站(i)的轴(j)绑定到控制器的虚拟轴(iaxis)
        // ═══════════════════════════════════════════════════════════════════
        Iresult += ZAux_Direct_SetAxisAddress(
            handle, EcatInfo.DriveAxisStart + BusAxisNum, BusAxisNum + 1);

        // ═══════════════════════════════════════════════════════════════════
        // 【步骤 4】设置 ATYPE 定义行为（第 249 行）
        // 操作目标：控制器虚拟轴
        // 被改主体：轴类型标志 (ATYPE)
        // 说明：65 = EtherCAT伺服轴（定义控制器如何理解此轴）
        // ═══════════════════════════════════════════════════════════════════
        Iresult += ZAux_Direct_SetAtype(
            handle, EcatInfo.DriveAxisStart + BusAxisNum, 65);

        // ═══════════════════════════════════════════════════════════════════
        // 【步骤 5】设置 DRIVE_PROFILE 配置 PDO（第 252 行起）
        // 操作目标：**驱动器从站**（不是控制器！）
        // 被改主体：PDO数据结构
        // 说明：定义控制器 <-> 驱动器的实时数据交换格式
        //      RxPDO(接收): 目标位置、速度、控制字
        //      TxPDO(发送): 实际位置、速度、状态
        // ═══════════════════════════════════════════════════════════════════
        if ((Drive_Device == ElmoDevice) && (Drive_Vender == ElmoVender))
        {
          // 如果是ELMO的驱动器的PDO配置

          // ELMO的驱动器需要关闭总线时钟优化
          Iresult += ZAux_Execute(
              handle, "SYSTEM_ZSET = CLEAR_BIT(7, SYSTEM_ZSET)", ReceBuff, 256);
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = -1",
                  EcatInfo.DriveAxisStart, BusAxisNum);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          sprintf(cmdbuff, " NODE_PROFILE(%d,%d) = -1", SlotId, i);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          // ELMO的驱动器需要自定义PDO
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c12, 0, 5,
                                          0);  // 禁用RxPDO,禁用后才可以修改内容
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c1c, 0, 5,
                                          0);  // 禁用TxPDO,禁用后才可以修改内容
          MyDelayMs(50, &OutTime);
          // 更新TXPDO列表
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 0, 5,
                                          0);  // 禁用0x1a07
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 1, 7,
                                          0x60410010);  // 状态字
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 2, 7,
                                          0x60770010);  // 当前力矩
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 3, 7,
                                          0x60640020);  // 反馈位置
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 4, 7,
                                          0x60fd0020);  // 驱动器输入
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 5, 7,
                                          0x60b90010);  // probe状态
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 6, 7,
                                          0x60ba0020);  // probe位置1
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 7, 7,
                                          0x60bb0020);  // probe位置2
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1a07, 0, 5,
                                          0x7);  // 启用分配
          // 更新RXPDO列表
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 0, 5,
                                          0);  // 禁用0x1607
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 1, 7,
                                          0x60400010);  // 控制字
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 2, 7,
                                          0x60710010);  // 周期力矩
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 3, 7,
                                          0x60ff0020);  // 周期速度
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 4, 7,
                                          0x607a0020);  // 目标位置
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 5, 7,
                                          0x60b80010);  // probe设置
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 6, 7,
                                          0x60720010);  // 力矩限制
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 7, 7,
                                          0x60600008);  // 控制模式
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1607, 0, 5,
                                          0x7);  // 启用分配
          // 1C12的配置
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c12, 0, 6,
                                          0x1607);  // RxPDO分配对象
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c12, 1, 5,
                                          1);  // 启用分配
          // 1C13的配置
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c13, 0, 6,
                                          0x1a07);  // TxPDO分配对象
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x1c13, 1, 5,
                                          1);  // 启用分配

          // 清除ELMO的驱动器报警
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6040, 0, 6,
                                          0);  // 状态初始化
          MyDelayMs(50, &OutTime);
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6040, 0, 6,
                                          7);  // 伺服shutdown
          MyDelayMs(50, &OutTime);
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6040, 0, 6,
                                          7);  // 伺服disable voltage
          MyDelayMs(50, &OutTime);
          Iresult += ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6040, 0, 6,
                                          15);  // 伺服fault reset
        }
        else if ((Drive_Device == 0x1ab0) && (Drive_Vender == 0x41B))
        {
          // 如果正运动的脉冲扩展卡
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = 0",
                  EcatInfo.DriveAxisStart, BusAxisNum);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        }
        else
        {
          // 驱动器PDO设置,驱动器默认设置-- -1 位置模式--0  速度模式--20+
          // 力矩模式--30+
          sprintf(cmdbuff, " DRIVE_PROFILE(%d + %d) = %d",
                  EcatInfo.DriveAxisStart, BusAxisNum,
                  EcatInfo.DrivePdoMode[BusAxisNum]);
          Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

          // ═══════════════════════════════════════════════════════════════════
          // 【步骤 6】设置 DRIVE_IO 映射（第 278 行）
          // 操作目标：**驱动器从站**（不是控制器！）
          // 被改主体：I/O方向标志（SETREVIN/SETFWDIN）
          // 说明：配置驱动器的 IO 极性（正反方向、限位等）
          // ═══════════════════════════════════════════════════════════════════
          if ((4 == EcatInfo.DrivePdoMode[BusAxisNum]) ||
              (5 == EcatInfo.DrivePdoMode[BusAxisNum]) ||
              (12 == EcatInfo.DrivePdoMode[BusAxisNum]))
          {
            int StartIdTemp =
                EcatInfo.DriveIoStara + EcatInfo.DriveIoSpa * (BusAxisNum);
            sprintf(cmdbuff, " DRIVE_IO(%d + %d) = %d", EcatInfo.DriveAxisStart,
                    BusAxisNum, StartIdTemp);
            Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
            // 设置负限位
            Iresult += ZAux_Direct_SetRevIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp, 1);
            // 设置正限位
            Iresult += ZAux_Direct_SetFwdIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp + 1);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp + 1, 1);
            // 设置负限位
            Iresult += ZAux_Direct_SetDatumIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, StartIdTemp + 2);
            Iresult += ZAux_Direct_SetInvertIn(handle, StartIdTemp + 2, 1);
          }
          else if (EcatInfo.DrivePdoMode[BusAxisNum] < 4)
          {
            int TempVar = 0;
            // 取消负限位的设置
            Iresult += ZAux_Direct_GetRevIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, &TempVar);
            if (TempVar >= EcatInfo.DriveIoStara)
            {
              Iresult += ZAux_Direct_SetRevIn(
                  handle, EcatInfo.DriveAxisStart + BusAxisNum, -1);
              Iresult += ZAux_Direct_SetInvertIn(handle, TempVar, 0);
            }
            // 取消正限位的设置
            Iresult += ZAux_Direct_GetFwdIn(
                handle, EcatInfo.DriveAxisStart + BusAxisNum, &TempVar);
            if (TempVar >= EcatInfo.DriveIoStara)
            {
              Iresult += ZAux_Direct_SetFwdIn(
                  handle, EcatInfo.DriveAxisStart + BusAxisNum, -1);
              Iresult += ZAux_Direct_SetInvertIn(handle, TempVar, 0);
            }
            // 取消设置原点
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

        // 每轴单独分组,轴报警只停自己
        sprintf(cmdbuff, " DISABLE_GROUP(%d)",
                EcatInfo.DriveAxisStart + BusAxisNum);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);

        BusAxisNum++;
      }

      /***************************************************************************************
          -------------------------------2：IO模块和AIO模块的设置------------------------------
      ***************************************************************************************/
      // ECAT节点数字量IO起始地址的映射
      if (EcatInfo.NodeIoId[i] >= 32)
      {
        sprintf(cmdbuff, "NODE_IO(%d, %d) = %d", SlotId, i,
                EcatInfo.NodeIoId[i]);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
      // ECAT节点模拟量IO起始地址的映射
      if (EcatInfo.NodeAIoId[i] > 0)
      {
        sprintf(cmdbuff, "NODE_AIO(%d, %d) = %d", SlotId, i,
                EcatInfo.NodeAIoId[i]);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
      }
      /***************************************************************************************
          --------------------------------3：特殊模块的设置----------------------------------
      ***************************************************************************************/
      // 正运动EIO24088脉冲扩展轴和EIO16084脉冲扩展轴轴类型的设置与脉冲模式的设置
      if ((Drive_Device == 0x1AB0) && (Drive_Vender == 0x41B))
      {
        for (int k = 0; k < NodeAxisNum; k++)
        {
          // 设置扩展脉冲轴ATYPE类型 ATYPE=1
          ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6011 + k * 0x800, 0, 5, 1);
          // 设置扩展脉冲轴INVERT_STEP脉冲输出模式  INVERT_STEP=0
          ZAux_BusCmd_SDOWrite(handle, SlotId, i, 0x6012 + k * 0x800, 0, 6, 0);
        }
      }
    }

    // 判断轴数目是否正确 (已在上面第一次循环中判断，此处不再重复)

    // ═══════════════════════════════════════════════════════════════════
    // 【步骤 7】SLOT_START 启动总线
    // 操作目标：EtherCAT总线
    // 被改主体：实时通信状态
    // 说明：切换为 OP 模式，开始周期循环
    //      Mode=4: PREOP (初始化)
    //      Mode=8: OP (正式运行，实时周期250us)
    // ═══════════════════════════════════════════════════════════════════
    MyDelayMs(100, &OutTime);
    // 【诊断：SLOT_START 前的状态检查】
    printf("[ecat_init] ===== SLOT_START 前诊断 =====\n");
    printf("[ecat_init] BusAxisNum=%d Iresult=%d\n", BusAxisNum, Iresult);
    printf("[ecat_init] 驱动器信息: Vender=0x%x Device=0x%x Alias=%d\n", Drive_Vender, Drive_Device, Drive_Alias);
    printf("[ecat_init] 执行 SLOT_START(0, 4) 进入 PREOP 模式...\n");
    sprintf(cmdbuff, "SLOT_START(%d, 4)", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    MyDelayMs(1000, &OutTime);
    EcatScanOutTime = OutTime;
    printf("[ecat_init] PREOP mode entered, Iresult after SLOT_START(0,4)=%d\n", Iresult);
    printf("[ecat_init] 等待 1000ms 后执行 SLOT_START(0, 8) 进入 OP 模式\n");
    sprintf(cmdbuff, "SLOT_START(%d, 8)  ?return", SlotId);
    Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
    ReceBuff[2] = 0;
    if (0 == strcmp("-1", ReceBuff))
    {
      printf("DEBUG SLOT_START returned -1 success\n");
      EcatScanOutTime = 0;
    }
    else
    {
      printf("[ecat_init] SLOT_START(%d, 8) response: ReceBuff='%s' strlen=%zu first_char_hex=%02x\n", SlotId, ReceBuff, strlen(ReceBuff), (unsigned char)ReceBuff[0]);
      MyDelayMs(500, &OutTime);
      EcatScanOutTime = EcatScanOutTime - 500;
    }
    while (EcatScanOutTime > 0)
    {
      // 读取在线命令的应答， 对没有接收应答的命令有用
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
      MyDelayMs(3000,
                &OutTime);  // 延迟3秒，等待驱动器时钟同步
      // ═══════════════════════════════════════════════════════════════════
      // 【步骤 8】WDOG = 1 启用看门狗（第 406 行）
      // 操作目标：控制器安全模块
      // 被改主体：看门狗使能标志
      // 说明：工业安全保护，监测系统心跳
      //      在轴使能前必须启用
      // 【步骤 9】DRIVE_CONTROLWORD 清驱动器故障（第 382 行）
      // 操作目标：**驱动器从站**（不是控制器！）
      // 被改主体：状态机控制字
      // 说明：通过状态机序列复位驱动器从故障态进入 Ready 状态
      //      0x80(Fault Clear) -> 0x06(Shutdown) -> 0x0F(Operation Enabled)
      // ═══════════════════════════════════════════════════════════════════
      for (int Drivei = EcatInfo.DriveAxisStart;
           Drivei < (EcatInfo.DriveAxisStart + BusAxisNum); ++Drivei)
      {
        /*sprintf(cmdbuff, "DRIVE_CLEAR(0) AXIS(%d)", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);*/

        // 伺服错误清除
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=128 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
        // 伺服shutdown
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=6 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
        // 伺服disable voltage
        /*sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=7 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);*/
        // 伺服fault reset
        sprintf(cmdbuff, "DRIVE_CONTROLWORD(%d)=15 ", Drivei);
        Iresult += ZAux_Execute(handle, cmdbuff, ReceBuff, 256);
        MyDelayMs(10, &OutTime);
      }
      MyDelayMs(10, &OutTime);
      // 清除控制器所有轴的错误状态
      ZAux_Direct_Single_Datum(handle, 0, 0);
      MyDelayMs(200, &OutTime);
      // 打开总线轴使能总开关
      Iresult += ZAux_Execute(handle, "WDOG=1", ReceBuff, 256);
      // ═══════════════════════════════════════════════════════════════════
      // 【步骤 11】轴使能 AXIS_ENABLE（第 411 行）
      // 操作目标：控制器虚拟轴
      // 被改主体：轴使能标志
      // 说明：电机正式通电，准备进入运动控制循环
      // ═══════════════════════════════════════════════════════════════════
      if (EcatInfo.DriveEnable == 1)
      {
        for (int Drivei = EcatInfo.DriveAxisStart;
             Drivei < (EcatInfo.DriveAxisStart + BusAxisNum); ++Drivei)
        {
          ZAux_Direct_SetAxisEnable(handle, Drivei,
                                    1);  // 总线轴通过这个指令上使能
          MyDelayMs(10, &OutTime);
        }
      }
      // ═══════════════════════════════════════════════════════════════════
      // 【步骤 12】初始化成功
      // 说明：系统进入正常工作循环
      //      • 控制器已配置完毕，虚拟轴与驱动器映射建立
      //      • EtherCAT 实时循环已启动（250us周期）
      //      • 所有驱动器进入 Operation Enabled 状态
      //      • PC端可通过 ZAux_Direct_MultiMovePvt() 下发轨迹命令
      //      • 控制器负责轨迹插补和多轴同步（DC时钟保证<1us）
      // ═══════════════════════════════════════════════════════════════════
      return Iresult;
    }
    else
    {
      printf("DEBUG returning EcatStartFailu\n");
      return EcatStartFailu;  // 总线开启失败
    }
  }
  // 未扫描到驱动器
  printf("DEBUG returning NotScanNode\n");
  return NotScanNode;
}