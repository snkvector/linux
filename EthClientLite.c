#include "EthClientLite.h"
#include "EthClientLite_MII.h"
#include "VStdLib.h"
#include "Eth_Priv.h"

uint8         EthClientLite_DutAddr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8         EthClientLite_TesterAddr[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8         EthClientLite_Broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct
{
   P2VAR(uint8, AUTOMATIC, ETHIF_APPL_DATA) EthClientLiteTxDataPtr;
   uint16 EthClientLiteTxLenByte;
   boolean EthClientTxConfirmed;
   boolean EthClientLiteIsValid;
   uint16 EthClientLiteFrameType;
} EthClientLite_TransmitStruct;

typedef struct EthClientLite_BufQueueTypeStruct
{
  Eth_DataType* BufPtr;
  struct EthClientLite_BufQueueTypeStruct* NextBuf;
  uint8 CtrlIdx;
} EthClientLite_BufQueueType;

uint8 EthClientLite_ModeDownActive = 0;
boolean EthClientLite_TrcvModeDownActive = 0;

#define ETHCLIENTLITE_TRANSMISSION_TRACE_SIZE ETH_TX_BUF_TOTAL
EthClientLite_TransmitStruct EthClientLite_TransmissionTrace[ETHCLIENTLITE_TRANSMISSION_TRACE_SIZE];

boolean EthTrcvClientLite_TrcvDumpActive = FALSE;
boolean EthTrcvClientLite_PeriodicSend = FALSE;
boolean EthTrcvClientLite_SetLoopback = FALSE;
boolean EthTrcvClientLite_GetPacketCnt = FALSE;
uint8 EthTrcvClientLite_DebugState = 0;

EthClientLite_BufQueueType* EthTrcvClientLite_Bufs2BeReleased = NULL_PTR;
EthClientLite_BufQueueType* EthTrcvClientLite_FreeBufStructs = NULL_PTR;
EthClientLite_BufQueueType EthTrcvClientLite_BufStruct[ETH_RX_BUF_TOTAL];

void EthClientLite_Test_UpdatePhysAddrFilter(Eth_FilterActionType Action, uint8* PhysAddr)
{
  /* set phys addr */
  Eth_UpdatePhysAddrFilter(0, PhysAddr, Action);
}

void EthClientLite_Test_SetPhysAddr(uint8* PhysAddr)
{
  /* set phys addr */
  Eth_SetPhysAddr(0, PhysAddr);
}

void EthClientLite_Test_SetControllerMode(uint8 ControllerMode)
{

  if (ControllerMode == ETH_MODE_DOWN)
  {
    EthClientLite_ModeDownActive = 1;
  }
}
void EthClientLite_Test_GetPhysAddr()
{
  uint8 PhysAddr[6];
  /* set phys addr */
  Eth_GetPhysAddr(0, PhysAddr);
}


void EthClientLite_Test_SetTransceiverMode(uint8 TransceiverMode)
{
  if (TransceiverMode == ETHTRCV_MODE_DOWN)
    {
      EthClientLite_TrcvModeDownActive = 1;
    }
}



void EthClientLite_RxIndicationMirroring(uint8 CtrlIdx, P2VAR(uint32, AUTOMATIC, AUTOMATIC) BufferPtr,
       uint16 LenByte )
{

}

void EthClientLite_TxConfirmationMirroring( uint8 CtrlIdx, uint8 BufIdx )
{

}

void EthClientLite_RxIndication( uint8 CtrlIdx, Eth_FrameType FrameType, boolean IsBroadcast, uint8* PhysAddrPtr,
  P2VAR(Eth_DataType, AUTOMATIC, AUTOMATIC) BufferPtr,
  uint16 LenByte )
{
  /* send back */
  /* ARP ping */
  P2VAR(Eth_DataType, AUTOMATIC, AUTOMATIC) BufPtr;
  P2VAR(uint8, AUTOMATIC, AUTOMATIC) BufPtrByte;
  uint8  BufIdx;
  uint16 LenByteTemp = LenByte;
  uint16 Idx;

#if (4 > ETHIF_SW_MAJOR_VERSION)
  uint16 FrameType = 0x0806;
#endif

#if (defined ETH_ENABLE_GET_PHYS_ADDR_LATCH) && (STD_ON == ETH_ENABLE_GET_PHYS_ADDR_LATCH)
  Eth_PhysAddrType SrcAddrLatch;
  Eth_PhysAddrType SrcAddrOrig;
  Eth_PhysAddrType DestAddr;
#endif

  if ((((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xA0) ||
      (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xB0) ||
      (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xC0))
  {
#if (4 <= ETHIF_SW_MAJOR_VERSION)
    /* ASR 4.1.1 API */
    if(E_OK == EthIf_ProvideTxBuffer(CtrlIdx, FrameType, 2, &BufIdx, &BufPtr, &LenByteTemp))
#else
    /* ASR 4.0.X API */
    if(E_OK == EthIf_ProvideTxBuffer(CtrlIdx, &BufIdx, &BufPtr, &LenByteTemp))
#endif  /* ETHIF_SW_MAJOR_VERSION */
    {
      BufPtrByte = (P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufPtr;

      if(LenByteTemp >= LenByte)
      {
#if (defined ETH_ENABLE_GET_PHYS_ADDR_LATCH) && (STD_ON == ETH_ENABLE_GET_PHYS_ADDR_LATCH)
        if (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xB0 ) /* ID is 0xB0. Test PLC Latch APIs */
        {
          Eth_GetSrcPhysAddrLatch(CtrlIdx, SrcAddrLatch);
          Eth_GetDestPhysAddrLatch(CtrlIdx, DestAddr);
          ETH_GETPHYSADDR(CtrlIdx, SrcAddrOrig);

          BufPtrByte[0] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[0];
          BufPtrByte[1] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[1];
          BufPtrByte[2] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[2];
          BufPtrByte[3] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[3];
          BufPtrByte[4] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4];
          BufPtrByte[5] = 1;

          for (Idx=0; Idx < sizeof(DestAddr); Idx++)
          {
            if (DestAddr[Idx] != SrcAddrOrig[Idx])
            {
              BufPtrByte[5] = 0;
              break;
            }
          }
          EthIf_Transmit(CtrlIdx, BufIdx, FrameType, FALSE, LenByte, &SrcAddrLatch[0]);

        }
        else
  #endif
        if ((((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xA0) ||
            (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xC0)) /* Mirror&Answer frame */
        {
          for (Idx=0; Idx < LenByte; Idx++)
          {
            BufPtrByte[Idx] = ((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[Idx];

          }
          if (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xA0)
          {
            EthIf_Transmit(CtrlIdx, BufIdx, FrameType, FALSE, LenByte, (P2VAR(uint8, AUTOMATIC, AUTOMATIC))&EthClientLite_TesterAddr[0]);
          }
          else
          {
            EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxDataPtr = (uint8*)BufPtr;
            EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxLenByte = LenByte;
            EthClientLite_TransmissionTrace[BufIdx].EthClientTxConfirmed = FALSE;
            EthClientLite_TransmissionTrace[BufIdx].EthClientLiteIsValid = TRUE;
            EthClientLite_TransmissionTrace[BufIdx].EthClientLiteFrameType = FrameType;
            EthIf_Transmit(CtrlIdx, BufIdx, FrameType, TRUE, LenByte, (P2VAR(uint8, AUTOMATIC, AUTOMATIC))&EthClientLite_TesterAddr[0]);
          }
        }
      }
    }
    else
    {
      /* Release buffer */
      (void)EthIf_Transmit(CtrlIdx, BufIdx, FrameType, FALSE, (uint16)0, (uint8*)&EthClientLite_TesterAddr[0]);
    }
  }
  else if ((((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[4] == 0xF0))
  {
    switch (((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[5])
    {
    case 0x01: /* Set phys address */
      EthClientLite_Test_SetPhysAddr(&((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[6]);
      break;
    case 0x03:
      EthClientLite_Test_SetControllerMode(((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[6]);
      break;
    case 0x05:
      EthClientLite_Test_GetPhysAddr();
      break;
    case 0x07:
      EthClientLite_Test_UpdatePhysAddrFilter(
        (Eth_FilterActionType)(((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[6]),
        &(((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[7]));
   break;
    case 0x10:
      EthClientLite_Test_SetTransceiverMode(((P2VAR(uint8, AUTOMATIC, AUTOMATIC))BufferPtr)[6]);
      break;
    }
  }
#if ((defined ETHIF_ENABLE_ZERO_COPY_EXTENSIONS) && (STD_ON == ETHIF_ENABLE_ZERO_COPY_EXTENSIONS))
  if (FrameType == 0xC0DE)
  {
    /* release buffer explicitly */
    if (NULL_PTR != EthTrcvClientLite_FreeBufStructs)
    {
      EthClientLite_BufQueueType* FreeBufStruct = EthTrcvClientLite_FreeBufStructs;
      EthTrcvClientLite_FreeBufStructs = EthTrcvClientLite_FreeBufStructs->NextBuf;

      FreeBufStruct->BufPtr = BufferPtr;
      FreeBufStruct->NextBuf = EthTrcvClientLite_Bufs2BeReleased;
      FreeBufStruct->CtrlIdx = CtrlIdx;
      EthTrcvClientLite_Bufs2BeReleased = FreeBufStruct;
    }
  }
#endif
}

void EthClientLite_TxConfirmation( uint8 CtrlIdx, uint8 BufIdx )
{

  uint8 NewBufIdx;
  uint32 i;
  uint8* BufPtr;
  uint16 LenByteTemp;

  /* Ethernet communication test */
  if (EthClientLite_TransmissionTrace[BufIdx].EthClientLiteIsValid == TRUE)
  {
    if ((EthClientLite_TransmissionTrace[BufIdx].EthClientTxConfirmed == FALSE) &&
        (EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxDataPtr[4] == 0xC0))
    {
      EthClientLite_TransmissionTrace[BufIdx].EthClientTxConfirmed = TRUE;
      LenByteTemp = EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxLenByte;
#if (4 <= ETHIF_SW_MAJOR_VERSION)
      /* ASR 4.1.1 API */
      if (E_OK == EthIf_ProvideTxBuffer(1, EthClientLite_TransmissionTrace[BufIdx].EthClientLiteFrameType,4, &NewBufIdx, (Eth_DataType**)&BufPtr, &LenByteTemp))
#else
      /* ASR 4.0.X API */
      if (E_OK == EthIf_ProvideTxBuffer(1, &NewBufIdx, (Eth_DataType**)&BufPtr, &LenByteTemp))
#endif  /* ETHIF_SW_MAJOR_VERSION */
      {
        if (LenByteTemp >= EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxLenByte)
        {
          for (i=0; i < EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxLenByte; i++)
          {
           BufPtr[i] = EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxDataPtr[i];
          }
          EthIf_Transmit(1, NewBufIdx, EthClientLite_TransmissionTrace[BufIdx].EthClientLiteFrameType, FALSE, EthClientLite_TransmissionTrace[BufIdx].EthClientLiteTxLenByte, (P2VAR(uint8, AUTOMATIC, AUTOMATIC))&EthClientLite_TesterAddr[0]);
        }
      }
    }
    EthClientLite_TransmissionTrace[BufIdx].EthClientLiteIsValid = FALSE;
  }
}

void EthClientLite_Cbk_TrcvLinkStateChange(uint8 CtrlIdx, EthTrcv_LinkStateType LinkState)
{

}

void EthClientLite_Init(void)
{
  uint8 Idx = 1;
  //uint8 PhysAddr[6] = {2,0,0,0,0,2};

  /* set phys addr */
  //Eth_SetPhysAddr(0, PhysAddr);

  EthTrcvClientLite_TrcvDumpActive = FALSE;
  EthTrcvClientLite_PeriodicSend = TRUE;//FALSE;

  EthTrcvClientLite_Bufs2BeReleased = NULL_PTR;
  while (ETH_RX_BUF_TOTAL > Idx)
  {
    EthTrcvClientLite_BufStruct[Idx].NextBuf = EthTrcvClientLite_FreeBufStructs;
    EthTrcvClientLite_FreeBufStructs = &EthTrcvClientLite_BufStruct[Idx];
    Idx++;
  }
}

void EthClientLite_MainFunction(void)
{
  static uint32 Counter = 0;
  static uint32 ModeCnt = 1;
  static uint8 PayloadIdx = 0;
  if (EthClientLite_ModeDownActive == 1)
  {
    /* set phys addr */
    Eth_SetControllerMode(0, (Eth_ModeType)ETH_MODE_DOWN);
    EthClientLite_ModeDownActive = 2;
  }
  else if (EthClientLite_ModeDownActive == 2)
  {
    Counter++;
    if (Counter == 800)
    {
      EthClientLite_ModeDownActive = 0;
      Eth_SetControllerMode(0, ETH_MODE_ACTIVE);
    }
  }
  else if (EthClientLite_TrcvModeDownActive == 1)
  {
    /* set phys addr */
    //ETHTRCV_SETTRANSCEIVERMODE(0, 0, (EthTrcv_ModeType)ETHTRCV_MODE_DOWN);
    EthClientLite_TrcvModeDownActive = 2;
  }
  else if (EthClientLite_TrcvModeDownActive == 2)
  {
    Counter++;
    if (Counter == 800)
    {
      EthClientLite_TrcvModeDownActive = 0;
     // ETHTRCV_SETTRANSCEIVERMODE(0, 0, ETHTRCV_MODE_ACTIVE);
    }
  }
  else
  {
    Counter = 0;
  }


  if (TRUE == EthTrcvClientLite_TrcvDumpActive)
  {
    //EthTrcvClientLite_TrcvDumpActive = FALSE;

    EthClientLite_MII_TransceiverRegisterDump();
  }

  if (TRUE == EthTrcvClientLite_PeriodicSend)
  {
    uint8 BufIdx;
    uint16 LenByte = 1000;
    Eth_DataType* Buf;
    uint32 Idx;
    uint32 Idx1;
    uint8 *RAM_Write_Addr = (uint8 *)0x3D800004;
    uint8 *RAM_Read_Addr = (uint8 *)0x3D900000;
    volatile uint8 *Write_Flag = (volatile uint8 *)0x3D800000;
    volatile uint8 *Read_flag = (volatile uint8 *)0x3D800001;
    uint8 RAM_Write_Buffer[100];
    uint8 RAM_Read_Buffer[100];
    static uint8 count=0;
    static uint8 start=0;
    
   if (start ==0)    
   {
     *Write_Flag =1;
     start = 1;
     count=0;
   }
  
    if (ModeCnt == 1)
    {
      if (BUFREQ_OK == EthIf_ProvideTxBuffer(0, 0xBAAD, 0, &BufIdx, &Buf, &LenByte))
      {
        for (Idx=0; Idx < 1000; Idx++)
        {
          ((uint8*)Buf)[Idx] = PayloadIdx++;
          // RAM_Write_Buffer[Idx] = Idx;
        }

       // memcpy(RAM_Write_Addr, RAM_Write_Buffer, 1000);
        EthIf_Transmit(0, BufIdx, 0xBAAD, FALSE, 1000, EthClientLite_Broadcast);
      }
    }
    
    if ( *Write_Flag == 1)  // assert write flag is equal to 1
    {
       for (Idx1=0; Idx1 < 100; Idx1++)
        {
          RAM_Write_Buffer[Idx1] = count;
        }
        
      memcpy(RAM_Write_Addr, RAM_Write_Buffer, 100);
      
      if (count == 100)
      {
        count=0;
      }
      else
      {
       count++;
      }
      *Write_Flag = 0; // set write flag to 0
    } 
    
    if ( *Read_flag == 0) // assert read flag is equal to 0
    {
       memcpy(RAM_Read_Buffer, RAM_Read_Addr,100);
       *Read_flag =1; // set read flag to 1
    }
//    if (ModeCnt == 2)
//    {
//      LenByte = 1000;
//      if (BUFREQ_OK == EthIf_ProvideTxBuffer(0, 0xBAAD, 0, &BufIdx, &Buf, &LenByte))
//          {
//            Buf[0] = PayloadIdx++;
//
//            EthIf_Transmit(0, BufIdx, 0xBAAD, FALSE, 1000, EthClientLite_Broadcast);
//          }
//    }
//    if (ModeCnt == 3)
//    {
//      LenByte = 1000;
//      if (BUFREQ_OK == EthIf_ProvideTxBuffer(0, 0xBAAD, 0, &BufIdx, &Buf, &LenByte))
//          {
//            Buf[0] = PayloadIdx++;
//
//            EthIf_Transmit(0, BufIdx, 0xBAAD, FALSE, 1000, EthClientLite_TesterAddr);
//          }
//    }
    ///ModeCnt++;

    if (ModeCnt > 3)
    {
      ModeCnt = 0;
    }
  }

#if ((defined ETHIF_ENABLE_ZERO_COPY_EXTENSIONS) && (STD_ON == ETHIF_ENABLE_ZERO_COPY_EXTENSIONS))
  if (NULL_PTR != EthTrcvClientLite_Bufs2BeReleased)
  {
    EthClientLite_BufQueueType* TmpBufStruct = EthTrcvClientLite_Bufs2BeReleased;

    EthIf_ReleaseRxBuffer(TmpBufStruct->CtrlIdx, TmpBufStruct->BufPtr);

    VStdSuspendAllInterrupts();
    EthTrcvClientLite_Bufs2BeReleased = EthTrcvClientLite_Bufs2BeReleased->NextBuf;

    TmpBufStruct->NextBuf = EthTrcvClientLite_FreeBufStructs;
    EthTrcvClientLite_FreeBufStructs = TmpBufStruct;
    VStdResumeAllInterrupts();

  }
#endif
}

/* In case interrupts are disabled - dummy ISR routine to avoid reconfiguring OS */
# if ( ETH_ENABLE_RX_INTERRUPT == STD_OFF )
# if ((defined ETH_INTERRUPT_CATEGORY) && (ETH_ISR_CATEGORY_2 == ETH_INTERRUPT_CATEGORY))
ISR ( EthIsr_Rx )
{


}
ISR ( EthIsr_Tx )
{


}
# else
void EthIsr_Rx (void)
{


}
void EthIsr_Tx (void)
{


}
# endif
#endif
  /* ETH_ENABLE_RX_INTERRUPT */
