#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "xc.h"
#include "uart3.h"
#include "../MyMain.h"
#include "../define_type.h"
#include "../HardwareProfile.h"

extern struct WStructure WeightStruc[2];
extern  uint8_t     PageWrite(uint32_t PageNumber);
extern  uint8_t     WriteDefaultCalibSetting ( void );
extern void         WriteErrorMessage(void);
extern void         SensitiveScaleReadData(void);
extern void         GetCalibrationSetting(void);
extern  uint8_t     Adjust_Zero ( uint8_t  Start,uint8_t ZeroTrack,uint8_t bypass,uint8_t Lid);
extern  void        CalibrateService( uint8_t Lid);
extern  APP_DATA    appData;
extern  uint8_t     Loadcell_Id,DataPack[200],sensitiveScaleDataPack[50],sPack[50],CalibSettingState,CalibProcess,ResLogCounter,
                    ErrorCalibrationTimout,TempCnt,Mask_Number,DispPos,Show_Front_Zero,waitForSensitivReceivedWeightCounter;

extern uint8_t  ResetCause  __attribute__ ((persistent));

extern union CalParm CalibrationParametes;
extern union SetParameter SystemParm;
extern const uint16_t  DivitionTable[13],DC_VALUE,BAT_VALUE;
extern const uint16_t  CrcTable[];

extern uint16_t beez_time,SerOutputWrp,SerOutputRrp,SerInputWrp,SerInputRrp;

extern int32_t View_Weight,Offset;
extern  int32_t  SavedOffset[2]  __attribute__ ((persistent));
extern uint8_t   Ser_Write_Buf[SER_MAX_WRITE],Ser_Read_Buf[SER_MAX_READ];

extern uint32_t Colpy;

extern union   Var_Bits BITS_2;
extern bool Cash_Open,Dc_Power_Good,Bat_Power_Good,NoCalibrate,Bat_Blink,Show_Back_Zero,Show_Decimal_Point,sendSensitive,sendNormal,
            Show_Sign,Tare_Flag,WeightCalculated,StartResolutionLog,packServiced,sensitiveDataCompleted,
            isSensitiveScaleZero,requestZero,Is_Around_Tolerance;
uint8_t  TempBuffer[10];


const uint8_t * LoadcellNumber="\r\nSensor ID : ";
                              
const uint8_t * WeightForCalibrate="\r\nCalib Weight : ";
const uint8_t * MaxWeight_1="\r\nMax W1 : ";
const uint8_t * MaxWeight_2="\r\nMax W2 : ";
const uint8_t * Division_1="\r\nDiv 1 : ";
const uint8_t * Division_2="\r\nDiv 2 : ";
const uint8_t * Zero_Persentage="\r\nZero Persent : ";
const uint8_t * Decimal_point="\r\nDp : ";
const uint8_t * Zero_Adc="\r\nZero ADC : ";
const uint8_t * CalibInfo="\r\nCal Factor : mul By 1000 : ";
const uint8_t * CalibStatus="\r\nCalib : ";
const uint8_t * CalibDone="Done\r\n";
const uint8_t * CalibFail="no Calib\r\n";




extern void Show_Dash( void );
extern void Change_To_String(uint8_t Buf[15],uint32_t Number);
extern void SYSTEM_Initialize_aux(void);
extern void InitMainPrograms(void);
extern void UpdateSetting(void);
extern uint32_t Change_To_long(uint8_t Buf[10]);
extern uint8_t Change_To_Str(uint8_t Buf[10],uint32_t Number);
extern uint32_t RealNumber( uint32_t Number,uint8_t Lid);
extern uint8_t Weight_Tasks( uint8_t Lid );
extern uint32_t   ChangeNumberDot(uint32_t  DotNumber);
extern void Load_Display_New( void );
extern void CountDownService( void  );
extern void UpdateDisplayData(void);

void  WeightTransferManage(uint8_t PFunc,uint8_t PAddress);
void AutoSendWeight(void);
void UpdateCalibrationParameters(void);
void Print_NewLine(void);
void Write_Serial_Buffer(uint8_t Data);
void  SendTakinAddWeight(uint8_t PFunc,uint8_t PAddress);
void  SendTakinWeight(uint8_t PFunc,uint8_t PAddress);
void  SendTakinWeight_Old(uint8_t PFunc,uint8_t PAddress);
uint16_t rnginc(uint16_t temp1,uint16_t temp2);


/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

static uint8_t * volatile rxTail;
static uint8_t *rxHead;
static uint8_t *txTail;
static uint8_t * volatile txHead;
static bool volatile rxOverflowed;
extern bool AutoSend,w_neg,StartResolutionLog;

extern uint32_t in_data;
extern int32_t ADC_DATA,ADC_DATA1,ADC_DATA2;

extern uint32_t FlashData[4];
extern uint8_t SwitchWeight,One_Gram,ResLogCounter;
extern void Send_Weight(int32_t ShowValue,bool DateTransmit,bool PrnUnit,bool ShowBack,bool ShDp,uint8_t ShowNormalWeight,bool GramUnit);


/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART3_CONFIG_TX_BYTEQ_LENGTH (200+1)
#define UART3_CONFIG_RX_BYTEQ_LENGTH (200+1)

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t txQueue[UART3_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t rxQueue[UART3_CONFIG_RX_BYTEQ_LENGTH];

void (*UART3_TxDefaultInterruptHandler)(void);
void (*UART3_RxDefaultInterruptHandler)(void);

/**
  Section: Driver Interface
*/

void UART3_Initialize(void)
{
    IEC1bits.U3TXIE = 0;
    IEC1bits.U3RXIE = 0;

    // STSEL 1; PDSEL 8N; RTSMD disabled; OVFDIS disabled; ACTIVE disabled; RXINV disabled; WAKE disabled; BRGH enabled; IREN disabled; ON enabled; SLPEN disabled; SIDL disabled; ABAUD disabled; LPBACK disabled; UEN TX_RX; CLKSEL SYSCLK; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U3MODE = (0x28008 & ~(1<<15));  // disabling UART ON bit
    // UTXISEL TX_ONE_CHAR; UTXINV disabled; ADDR 0; MASK 0; URXEN disabled; OERR disabled; URXISEL RX_ONE_CHAR; UTXBRK disabled; UTXEN disabled; ADDEN disabled; 
    U3STA = 0x00;
    // U3TXREG 0; 
    U3TXREG = 0x00;
    // BaudRate = 115200; Frequency = 24000000 Hz; BRG 51; 
    U3BRG = 0x33;
    
    txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;
   
    rxOverflowed = 0;

    UART3_SetTxInterruptHandler(UART3_Transmit_ISR);

    UART3_SetRxInterruptHandler(UART3_Receive_ISR);

    IEC1bits.U3RXIE = 1;
    
    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U3STASET = _U3STA_UTXEN_MASK;
    U3MODESET = _U3MODE_ON_MASK;   // enabling UART ON bit
    U3STASET = _U3STA_URXEN_MASK;
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/

void UART3_SetTxInterruptHandler(void* handler)
{
    UART3_TxDefaultInterruptHandler = handler;
}

void __attribute__ ((vector(_UART3_TX_VECTOR), interrupt(IPL1SOFT))) _UART3_TX ( void )
{
    (*UART3_TxDefaultInterruptHandler)();
}

void UART3_Transmit_ISR ( void )
{ 
    if(IEC1bits.U3TXIE)
    {
        if(SerOutputWrp!=SerOutputRrp)
        {
            U3TXREG=Ser_Write_Buf[SerOutputRrp];
            SerOutputRrp=rnginc(SerOutputRrp,SER_MAX_WRITE); 
            IFS1CLR= 1 << _IFS1_U3TXIF_POSITION;
        }
        else
        {
            IEC1bits.U3TXIE=0;
        }
    }
}

void UART3_SetRxInterruptHandler(void* handler)
{
    UART3_RxDefaultInterruptHandler = handler;
}

void __attribute__ ((vector(_UART3_RX_VECTOR), interrupt(IPL1SOFT))) _UART3_RX( void )
{
    (*UART3_RxDefaultInterruptHandler)();
}

void UART3_Receive_ISR(void)
{

 //   while((U3STAbits.URXDA == 1))
   // {
  		Ser_Read_Buf[SerInputWrp]=U3RXREG;
		SerInputWrp=rnginc(SerInputWrp,SER_MAX_READ);
   // }

    IFS1CLR= 1 << _IFS1_U3RXIF_POSITION;
}

void __attribute__ ((vector(_UART3_ERR_VECTOR), interrupt(IPL1SOFT))) _UART3_ERR( void )
{
    if ((U3STAbits.OERR == 1))
    {
        U3STACLR = _U3STA_OERR_MASK; 
    }
    
    IFS1CLR= 1 << _IFS1_U3EIF_POSITION;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART3_Read( void)
{
    uint8_t data = 0;

    while (rxHead == rxTail )
    {
    }
    
    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART3_CONFIG_RX_BYTEQ_LENGTH))
    {
        rxHead = rxQueue;
    }
    return data;
}

uint16_t rnginc(uint16_t temp1,uint16_t temp2)
{
	temp1 = (( temp1+1)<temp2)?(temp1+1):0;
	return(temp1);
}

void SetPart(uint8_t Part,uint8_t Data)
{
    if(Part==0)
        CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1=Data; 
    else
        CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P2=Data; 
    WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
    
}

void ReturnDivisionIndex(uint16_t LoadedDivision, uint8_t Part)
{
    switch(LoadedDivision)
    {
        case 1: 
            SetPart(Part,0); 
        break;
        case 2: 
            SetPart(Part,1); 
        break;
        case 5: 
            SetPart(Part,2); 
        break;
        case 10: 
            SetPart(Part,3); 
        break;
        case 20: 
            SetPart(Part,4); 
        break;
        case 25: 
            SetPart(Part,5); 
        break;
        case 50: 
            SetPart(Part,6); 
        break;
        case 100: 
            SetPart(Part,7); 
        break;
        case 200: 
            SetPart(Part,8); 
        break;
        case 500: 
            SetPart(Part,9); 
        break;
        case 1000: 
            SetPart(Part,10); 
        break;
        case 2000: 
            SetPart(Part,11); 
        break;
        case 5000: 
            SetPart(Part,12); 
        break;
    }
}

uint32_t crccheck(uint32_t DataLenght)
{
    uint32_t crc = 0xFFFF;
    uint16_t LoopCnti;
    for (LoopCnti=0;LoopCnti<DataLenght;LoopCnti++)
    {
        crc = (uint16_t)((crc >> 8) ^ CrcTable[(crc ^ DataPack[LoopCnti]) & 0xFF]);
    }
    return crc;
}


uint32_t ComputeCrc(uint32_t DataLenght)
{
    uint32_t crc = 0xFFFF;
    uint16_t LoopCnti;
    for (LoopCnti=1;LoopCnti<DataLenght;LoopCnti++)
    {
        crc = (uint16_t)((crc >> 8) ^ CrcTable[(crc ^ sensitiveScaleDataPack[LoopCnti]) & 0xFF]);
    }
    return crc;
}

    
            


void SendTakinFuncrions(uint8_t Address,uint8_t Function,uint8_t PackData)
{
    uint8_t LoopCnt=0;
    uint32_t crci;
    DataPack[0]=Function;
    DataPack[1]=1;
    DataPack[2]=PackData;
    crci=crccheck(3);   
    DataPack[3]=(uint8_t)(crci>>8);
    DataPack[4]=(uint8_t)(crci&0xff);
    UART3_Write(Address);
    for(LoopCnt=0;LoopCnt<5;LoopCnt++)
    UART3_Write(DataPack[LoopCnt]);
}

uint32_t MakeTexts(const uint8_t * StrData,uint32_t OffsetMem,uint32_t NumberAdded,bool Bypass)
{
    strcpy(DataPack+OffsetMem+2,StrData);
    OffsetMem+=strlen(StrData);
    if(Bypass==0)
    {
        OffsetMem+=Change_To_Str(DataPack+OffsetMem+2,NumberAdded);
        DataPack[OffsetMem+2]=',';OffsetMem++;
    }
    return OffsetMem;
}

uint32_t MakePowerInfoPacket(uint32_t OffsetMem)
{
    double TVal;
    uint8_t Len;
    TVal=DC_VALUE;
    TVal*=Dc_Coeff;
    Len=Change_To_Str(DataPack+OffsetMem+2,(uint32_t)TVal);
    OffsetMem+=Len;
    DataPack[OffsetMem+2]=',';OffsetMem++;
    TVal=BAT_VALUE;
    TVal*=Bat_Coeff;
    Len=Change_To_Str(DataPack+OffsetMem+2,(uint32_t)TVal);
    OffsetMem+=Len;
    DataPack[OffsetMem+2]=',';OffsetMem++;
    TVal=DC_VALUE;
    TVal*=Dc_Coeff;
    TVal=(uint32_t)TVal;
    if(Dc_Power_Good)
        DataPack[OffsetMem+2]='1';
        else
        DataPack[OffsetMem+2]='0';
    OffsetMem++;
    DataPack[OffsetMem+2]=',';OffsetMem++;
    TVal=BAT_VALUE;
    TVal*=Bat_Coeff;
    TVal=(uint32_t)TVal;
    if(Bat_Power_Good)
        DataPack[OffsetMem+2]='1';
        else
        DataPack[OffsetMem+2]='0';
    OffsetMem++;
    return OffsetMem;

}


void Uncalibration_Message(void)
{
    beez_time=BEEP_500ms;
    Print_NewLine();
    Write_Serial_Buffer('!');
    Write_Serial_Buffer('!');
    Write_Serial_Buffer('!');
    Write_Serial_Buffer(' ');
    Write_Serial_Buffer('S');
    Write_Serial_Buffer('c');
    Write_Serial_Buffer('a');
    Write_Serial_Buffer('l');
    Write_Serial_Buffer('e');
    Write_Serial_Buffer(' ');
    Write_Serial_Buffer('U');
    Write_Serial_Buffer('n');
    Write_Serial_Buffer('C');
    Write_Serial_Buffer('a');
    Write_Serial_Buffer('l');
    Write_Serial_Buffer('i');
    Write_Serial_Buffer('b');
    Write_Serial_Buffer('r');
    Write_Serial_Buffer('a');
    Write_Serial_Buffer('t');
    Write_Serial_Buffer('e');
    Write_Serial_Buffer('d');
    Write_Serial_Buffer('.');
    
}



bool TakinTasks(uint8_t data)
{
    
    static uint8_t TakinCalibState=0,LoopCnt=0,crclow=0,crchigh=0,ProtocolAddress=0,ProtocolDataLenght=0,ProtocolFunction=0;
    uint8_t Len,SettingPartState=0,TempVar;
    uint32_t TempVariable=0,crci;
    double TVal=0;
    if(TakinCalibState==0 && data==0x01)
    {
        TakinCalibState=1;
        ProtocolAddress=data;
    }
    else
    if(TakinCalibState==1)
    {
        TakinCalibState=2;
        ProtocolFunction=data;
        DataPack[0]=data; 

    }
    else
    if(TakinCalibState==2)
    {
        TakinCalibState=3;
        ProtocolDataLenght=data;
        DataPack[1]=data;
        LoopCnt=2;    
    }
    else
    if(TakinCalibState==3)
    {
        DataPack[LoopCnt]=data;
        LoopCnt++;
        if(LoopCnt==ProtocolDataLenght+2)
            TakinCalibState=4;
    }
    else
    if(TakinCalibState==4)
    {
        TakinCalibState=5;
        crchigh=data;

    }
    else
    if(TakinCalibState==5)
    {
        TakinCalibState=0;
        crclow=data;
        TempVariable=crchigh;
        TempVariable<<=8;
        TempVariable|=crclow;

        crci=crccheck(ProtocolDataLenght+2);
        if(ProtocolFunction==0x29)
           crci= TempVariable;
        if(TempVariable==crci)
        {
            switch(ProtocolFunction)
            {

                case 0x3A:
                    beez_time=BEEP_50ms;
                    StartResolutionLog=true;
                    ResLogCounter=0;
                    WeightStruc[Loadcell_Id].maxAdc=WeightStruc[0].ADC_DATA;
                    WeightStruc[Loadcell_Id].minAdc=WeightStruc[0].ADC_DATA;
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                    Print_NewLine();
                    Write_Serial_Buffer('P');
                    Write_Serial_Buffer('l');
                    Write_Serial_Buffer('e');
                    Write_Serial_Buffer('a');
                    Write_Serial_Buffer('s');
                    Write_Serial_Buffer('e');
                    Write_Serial_Buffer(' ');
                    Write_Serial_Buffer('w');
                    Write_Serial_Buffer('a');
                    Write_Serial_Buffer('i');
                    Write_Serial_Buffer('t');
                    Write_Serial_Buffer('.');
                    Write_Serial_Buffer('.');
                    Write_Serial_Buffer('.');
                break;
                case 0x38:
                    CASH=1;
                    Cash_Open=1;
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x12:
                    if(SystemParm.Set_Parameter.DualSyncLoadcell)
                        SendTakinAddWeight(ProtocolFunction,ProtocolAddress);
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                        
                break;
                case 0x27:
                    if(SystemParm.Set_Parameter.DualSyncLoadcell==0)
                        WeightStruc[Loadcell_Id].Tare_Flag=1;
                    else
                    {
                        WeightStruc[0].Tare_Flag=1;
                        WeightStruc[1].Tare_Flag=1;
                    }
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x28:
                    if(SystemParm.Set_Parameter.DualSyncLoadcell==0)
                    {
                        WeightStruc[Loadcell_Id].Tare_Flag=0;
                        WeightStruc[Loadcell_Id].Tare_Weight=0;
                    }
                    else
                    {
                        WeightStruc[0].Tare_Flag=0;
                        WeightStruc[0].Tare_Weight=0;
                        WeightStruc[1].Tare_Flag=0;
                        WeightStruc[1].Tare_Weight=0;
                    }
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x26:
                    requestZero=true;
                    if(SystemParm.Set_Parameter.DualSyncLoadcell==0)
                    {
                        if(Adjust_Zero(1,1,0,Loadcell_Id)==0)
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        else
                            SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                    }
                    else
                    {
                        if(Adjust_Zero(1,0,0,0)==0)
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        else
                            SendTakinFuncrions(ProtocolAddress,0x18,'1');
                        if(Adjust_Zero(1,0,0,1)==0)
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        else
                            SendTakinFuncrions(ProtocolAddress,0x18,'2');
                    }
                break;
                case 0x15:
                    ResetCause=0x00;
                    SYSTEM_Initialize_aux();
                    InitMainPrograms();
                    appData.state=APP_STATE_COUNT_DOWN;
                    Show_Dash();
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
//                    __pic32_software_reset();
               break;
                case 0x16:
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x14:
                    if(SystemParm.Set_Parameter.AutoSend==false)
                        WeightTransferManage(ProtocolFunction,ProtocolAddress);
                break;
                case 0x13:
                    if(SystemParm.Set_Parameter.AutoSend==false)
                        SendTakinWeight(ProtocolFunction,ProtocolAddress);
                   // if(WeightCalculated)
                break;
                case 0x36:
                    DataPack[0]=ProtocolFunction;
                    TempVariable=0;
                    TempVariable=MakePowerInfoPacket(TempVariable);
                    DataPack[1]=TempVariable;
                    crci=crccheck(TempVariable+2);   
                    DataPack[TempVariable+2]=(uint8_t)(crci>>8);
                    DataPack[TempVariable+3]=(uint8_t)(crci&0xff);
                    UART3_Write(ProtocolAddress);
                    for(LoopCnt=0;LoopCnt<TempVariable+4;LoopCnt++)
                        UART3_Write(DataPack[LoopCnt]);
                break;
                case 0x37:
                    DataPack[0]=ProtocolFunction;
                    TempVariable=0;
                    TempVariable=MakeTexts(LoadcellNumber,TempVariable,Loadcell_Id,0);
                    TempVariable=MakeTexts(WeightForCalibrate,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Calibration_Weight_Number,0);
                    TempVariable=MakeTexts(MaxWeight_1,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Max_weight_P1,0);
                    TempVariable=MakeTexts(MaxWeight_2,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Max_weight_P2,0);
                    TempVariable=MakeTexts(Division_1,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1,0);
                    TempVariable=MakeTexts(Division_2,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P2,0);
                    TempVariable=MakeTexts(Zero_Persentage,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Percentage,0);
                    TempVariable=MakeTexts(Decimal_point,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].Point_Index,0);
                    TempVariable=MakeTexts(Zero_Adc,TempVariable,labs(CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Atod),0);
                    TempVariable=MakeTexts(CalibInfo,TempVariable,labs(CalibrationParametes.CalibParameter[Loadcell_Id].W_factor.factor*1000),0);
                    TempVariable=MakeTexts(CalibStatus,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated,1);
                    if(CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated==0x01)
                        TempVariable=MakeTexts(CalibDone,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated,1);
                    else
                        TempVariable=MakeTexts(CalibFail,TempVariable,CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated,1);
                    DataPack[1]=TempVariable;
                    crci=crccheck(TempVariable+2);   
                    DataPack[TempVariable+2]=(uint8_t)(crci>>8);
                    DataPack[TempVariable+3]=(uint8_t)(crci&0xff);
                    UART3_Write(ProtocolAddress);
                    for(LoopCnt=0;LoopCnt<TempVariable+4;LoopCnt++)
                        UART3_Write(DataPack[LoopCnt]);
                    
                break;
                case 0x39:
                    if(DataPack[2]<=1)
                    {
                        Loadcell_Id=DataPack[2];
                        SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        if(SystemParm.Set_Parameter.DualSyncLoadcell==0)
                            Channel_Selection=Loadcell_Id;
                        SystemParm.Set_Parameter.DefualtChannel=Loadcell_Id;
                        UpdateSetting();
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;

                case 0x40:
                    if(DataPack[2]<=1)
                    {
                        SystemParm.Set_Parameter.DualSyncLoadcell=DataPack[2];
                        SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        UpdateSetting();
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                    if(SystemParm.Set_Parameter.DualSyncLoadcell==0)
                    {
                        Channel_Selection=(bool)SystemParm.Set_Parameter.DefualtChannel;
                        Loadcell_Id=SystemParm.Set_Parameter.DefualtChannel;
                    }
                break;
                case 0x41:
                    TempVar=DataPack[2];
                    if(SystemParm.Set_Parameter.DualSyncLoadcell && TempVar<=2)
                    {
                        if (((CalibrationParametes.CalibParameter[0].Point_Index == CalibrationParametes.CalibParameter[1].Point_Index) && 
                                         (CalibrationParametes.CalibParameter[0].Divition_Index_P1 == CalibrationParametes.CalibParameter[1].Divition_Index_P1) && 
                                         (CalibrationParametes.CalibParameter[0].Divition_Index_P2 == CalibrationParametes.CalibParameter[1].Divition_Index_P2) && 
                                         (CalibrationParametes.CalibParameter[0].Max_weight_P1 == CalibrationParametes.CalibParameter[1].Max_weight_P1) && 
                                         (CalibrationParametes.CalibParameter[0].Max_weight_P2 == CalibrationParametes.CalibParameter[1].Max_weight_P2)) && TempVar==1)
                        {
                            SystemParm.Set_Parameter.ViewFormat=TempVar;
                            UpdateSetting();
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        }
                        else
                            SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                        if (TempVar==0 || TempVar==2)
                        {
                            SystemParm.Set_Parameter.ViewFormat=TempVar;
                            UpdateSetting();
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        }
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;
                case 0x42:
                    TempVar=DataPack[2];
                    if (TempVar<2)
                    {
                        One_Gram=TempVar;
                        SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;
                case 0x43:
                    TempVar=DataPack[2];
                    if (TempVar<2)
                    {
                        if(CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated==1)
                        {
                            SystemParm.Set_Parameter.AutoSend=TempVar;
                            SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                            UpdateSetting();
                        }
                        else
                            Uncalibration_Message();
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;
                case 0x44:
                    TempVar=DataPack[2];
                    if (TempVar<2)
                    {
                        SystemParm.Set_Parameter.AutoSendNewOld=TempVar;
                        SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        UpdateSetting();            
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;
                case 0x33:
                    Adjust_Zero(1,0,1,Loadcell_Id);
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x34:
                    if(DataPack[2]<=2)
                    {
                        SystemParm.Set_Parameter.SelectedManufacture=DataPack[2];
                        SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                        if(SystemParm.Set_Parameter.SelectedManufacture==PAND)
                        {
                            if(CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated==1)
                                SystemParm.Set_Parameter.AutoSend=1;
                            else
                                Uncalibration_Message();
                        }
                        else
                            SystemParm.Set_Parameter.AutoSend=0;
                        if(SystemParm.Set_Parameter.SelectedManufacture==TOZIN_SADR)
                        {
                            if(CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated!=1)
                                Uncalibration_Message();
                        }
                        UpdateSetting();   
                        
                        
                                            ResetCause=0x00;
                    SYSTEM_Initialize_aux();
                    InitMainPrograms();
                    appData.state=APP_STATE_COUNT_DOWN;
                    Show_Dash();
                    //SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                    
                    
                    }
                    else
                        SendTakinFuncrions(ProtocolAddress,0x18,0x30);
                break;
                case 0x35:
                    Adjust_Zero(1,0,1,Loadcell_Id);
                    CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Atod=WeightStruc[Loadcell_Id].offset;
                    UpdateCalibrationParameters();            
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                break;
                case 0x20:
                    if(WeightStruc[Loadcell_Id].ADC_DATA<0)
                    {
                        Write_Serial_Buffer('!');
                        Write_Serial_Buffer('!');
                        Write_Serial_Buffer('!');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('C');
                        Write_Serial_Buffer('h');
                        Write_Serial_Buffer('e');
                        Write_Serial_Buffer('c');
                        Write_Serial_Buffer('k');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('L');
                        Write_Serial_Buffer('o');
                        Write_Serial_Buffer('a');
                        Write_Serial_Buffer('d');
                        Write_Serial_Buffer('c');
                        Write_Serial_Buffer('e');
                        Write_Serial_Buffer('l');
                        Write_Serial_Buffer('l');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('i');
                        Write_Serial_Buffer('n');
                        Write_Serial_Buffer('s');
                        Write_Serial_Buffer('t');
                        Write_Serial_Buffer('l');
                        Write_Serial_Buffer('l');
                        Write_Serial_Buffer('a');
                        Write_Serial_Buffer('t');
                        Write_Serial_Buffer('i');
                        Write_Serial_Buffer('o');
                        Write_Serial_Buffer('n');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('o');
                        Write_Serial_Buffer('r');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('c');
                        Write_Serial_Buffer('a');
                        Write_Serial_Buffer('b');
                        Write_Serial_Buffer('l');
                        Write_Serial_Buffer('e');
                        Write_Serial_Buffer(' ');
                        Write_Serial_Buffer('.');
                    }
                    else
                    {
                        appData.state=APP_STATE_CALIBRATION;
                    }
                    ErrorCalibrationTimout=0;
                break;
                case 0x29:
                    LoopCnt=2;
                    SettingPartState=0;
                    do
                    {
                        data=DataPack[LoopCnt];
                        if(data!=',')
                        {
                            if(data>='0' && data<='9')
                            {
                                TempBuffer[TempCnt]=data;
                                TempCnt++;
                            }
                        }
                        if(data==',' || LoopCnt==ProtocolDataLenght+1)
                        {
                            switch(SettingPartState)
                            {
                                case 0:
                                    SettingPartState=1;
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Calibration_Weight_Number=Change_To_long(TempBuffer);
                                break;
                                case 1:
                                    SettingPartState=2;
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Max_weight_P1=Change_To_long(TempBuffer);
                                break;
                                case 2:
                                    SettingPartState=3;
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Max_weight_P2=Change_To_long(TempBuffer);
                                break;
                                case 3:
                                    TempVariable=Change_To_long(TempBuffer);
                                    ReturnDivisionIndex(TempVariable,0);
                                    SettingPartState=4;
                                break;
                                case 4:
                                    TempVariable=Change_To_long(TempBuffer);
                                    ReturnDivisionIndex(TempVariable,1);
                                    SettingPartState=5;
                                break;
                                case 5:
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Percentage=Change_To_long(TempBuffer);
                                    if(CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Percentage>100)
                                        CalibrationParametes.CalibParameter[Loadcell_Id].Zero_Percentage=0;
                                    SettingPartState=6;
                                break;
                                case 6:
                                    TempVariable=Change_To_long(TempBuffer);
                                    if(TempVariable<=3)
                                        CalibrationParametes.CalibParameter[Loadcell_Id].Point_Index=TempVariable;
                                    SettingPartState=7;
                                break;
                                case 7:
                                    TempVariable=Change_To_long(TempBuffer);
                                    if(TempVariable<=15)
                                        CalibrationParametes.CalibParameter[Loadcell_Id].Filter_Index=TempVariable;
                                    SettingPartState=8;
                                break;
                                case 8:
                                    TempVariable=Change_To_long(TempBuffer);
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Stable_Time_Index=TempVariable;
                                    SettingPartState=9;
                                break;
                                case 9:
                                    TempVariable=Change_To_long(TempBuffer);
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Stable_Extent_Index=TempVariable;
                                    SettingPartState=10;
                                break;
                                case 10:
                                    TempVariable=Change_To_long(TempBuffer);
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Lock_Functionality=TempVariable;
                                    SettingPartState=11;
                                break;
                                case 11:
                                    TempVariable=Change_To_long(TempBuffer);
                                    CalibrationParametes.CalibParameter[Loadcell_Id].Number_Of_Locked_Divisions=TempVariable;
                                    SettingPartState=0;
                                break;
                            }
                            for(TempCnt=0;TempCnt<10;TempCnt++)
                                TempBuffer[TempCnt]=0;
                            TempCnt=0;
                        }
                        LoopCnt++;

                    }while(LoopCnt<ProtocolDataLenght+2);
                    SendTakinFuncrions(ProtocolAddress,ProtocolFunction,0x30);
                    UpdateCalibrationParameters();
                break;

            }
        }
        else
        {
            TakinCalibState=0;
        }
    }
    else
    {
        SerInputWrp=SerInputRrp;
        TakinCalibState=0;
    }    
}


void  SendTakinAddWeight(uint8_t PFunc,uint8_t PAddress)
{
    uint32_t TempVariable;
    uint8_t Len,LoopCnt;
    uint16_t crci;
    DataPack[0]=PFunc;
    if(WeightStruc[0].w_neg || WeightStruc[1].w_neg )
    {
        DataPack[2]='-';
        DataPack[3]='1';
        Len=2;
    }
    else
        if(WeightStruc[0].View_Weight>(RealNumber(CalibrationParametes.CalibParameter[0].Max_weight_P2,0)+9*DivitionTable[CalibrationParametes.CalibParameter[0].Divition_Index_P2]) ||
           WeightStruc[1].View_Weight>(RealNumber(CalibrationParametes.CalibParameter[1].Max_weight_P2,0)+9*DivitionTable[CalibrationParametes.CalibParameter[1].Divition_Index_P2]))
        {
            DataPack[2]='-';
            DataPack[3]='2';
            Len=2;                                
        }
        else
        {
            Len=Change_To_Str(DataPack+2,labs(WeightStruc[0].View_Weight+WeightStruc[1].View_Weight));
        }
    TempVariable=Len;
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[0].ADC_DATA+WeightStruc[1].ADC_DATA));
    TempVariable+=Len;
    DataPack[TempVariable+2]=',';TempVariable++;
    if(WeightStruc[0].Zero_Flag && WeightStruc[1].Zero_Flag )
        DataPack[TempVariable+2]='1';
    else
        DataPack[TempVariable+2]='0';
    TempVariable++;
    DataPack[TempVariable+2]=',';TempVariable++;
    if(WeightStruc[0].Net_Flag && WeightStruc[1].Net_Flag )
        DataPack[TempVariable+2]='1';
    else
        DataPack[TempVariable+2]='0';
    TempVariable++;
    DataPack[TempVariable+2]=',';TempVariable++;
    if(WeightStruc[0].Stable_Flag && WeightStruc[1].Stable_Flag )
        DataPack[TempVariable+2]='1';
    else
        DataPack[TempVariable+2]='0';
    TempVariable++;
    DataPack[TempVariable+2]=',';TempVariable++;
    TempVariable=MakePowerInfoPacket(TempVariable);
    DataPack[1]=TempVariable;
    crci=crccheck(TempVariable+2);   
    DataPack[TempVariable+2]=(uint8_t)(crci>>8);
    DataPack[TempVariable+3]=(uint8_t)(crci&0xff);
    UART3_Write(PAddress);
    for(LoopCnt=0;LoopCnt<TempVariable+4;LoopCnt++)
        UART3_Write(DataPack[LoopCnt]);
}

void  WeightTransferManage(uint8_t PFunc,uint8_t PAddress)
{
    int LoopCnt=0;
    if(sensitiveDataCompleted==true)
    {
        if((isSensitiveScaleZero ==false && Is_Around_Tolerance==false) && WeightStruc[Loadcell_Id].View_Weight==0)
        {
            sendSensitive=true;
            sendNormal=false;
        }
        sensitiveDataCompleted=false;
    }
    if(WeightStruc[Loadcell_Id].View_Weight>0 && (isSensitiveScaleZero==true || Is_Around_Tolerance==true))
    {
        sendSensitive=false;
        sendNormal=true;
    }
    if(sendSensitive)
    {
        for(LoopCnt=0;LoopCnt<sPack[2]+5;LoopCnt++)
            UART3_Write(sPack[LoopCnt]);
    }
    if(sendNormal)
        SendTakinWeight_Old(PFunc,PAddress);
}

void  SendTakinWeight_Old(uint8_t PFunc,uint8_t PAddress)
{
    uint32_t TempVariable;
    uint8_t Len,LoopCnt;
    uint16_t crci,WeightStateFlage=0x0000;
    DataPack[0]=PFunc;
        if(WeightStruc[Loadcell_Id].w_neg)
        {
            DataPack[2]='-';
            DataPack[3]='1';
            Len=2;
        }
        else
            if(WeightStruc[Loadcell_Id].Over_Flag)
            {
                DataPack[2]='-';
                DataPack[3]='2';
                Len=2;                                
            }
            else
            {
                Len=Change_To_Str(DataPack+2,labs(WeightStruc[Loadcell_Id].View_Weight));

            }
    TempVariable=Len;
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[Loadcell_Id].ADC_DATA));
    TempVariable+=Len;
    DataPack[1]=TempVariable;
    crci=crccheck(TempVariable+2);   
    DataPack[TempVariable+2]=(uint8_t)(crci>>8);
    DataPack[TempVariable+3]=(uint8_t)(crci&0xff);
    UART3_Write(PAddress);
    for(LoopCnt=0;LoopCnt<TempVariable+4;LoopCnt++)
        UART3_Write(DataPack[LoopCnt]);
}



void  SendTakinWeight(uint8_t PFunc,uint8_t PAddress)
{
    uint32_t TempVariable;
    uint8_t Len,LoopCnt;
    uint16_t crci,WeightStateFlage=0x0000;
    DataPack[0]=PFunc;

    Len=Change_To_Str(DataPack+2,labs(WeightStruc[0].View_Weight));
    TempVariable=Len;
    
    DataPack[TempVariable+2]=',';TempVariable++;
    
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[0].Tare_Weight));
    TempVariable+=Len;
    
    DataPack[TempVariable+2]=',';TempVariable++;    
    
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[0].ADC_DATA));
    TempVariable+=Len;

    WeightStateFlage=(   WeightStruc[0].Under_Flag<<7 | (!(Dc_Power_Good))<<6 | WeightStruc[0].Over_Flag<<5 | WeightStruc[0].W2_Flag<<4  | WeightStruc[0].w_neg<<3 | WeightStruc[0].Stable_Flag<<2 | 
                         WeightStruc[0].Net_Flag<<1 | WeightStruc[0].Zero_Flag);
        
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,WeightStateFlage);
    TempVariable+=Len;
    
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[1].View_Weight));
    TempVariable+=Len;
    
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[1].Tare_Weight));
    TempVariable+=Len;
    
    DataPack[TempVariable+2]=',';TempVariable++;
    Len=Change_To_Str(DataPack+TempVariable+2,labs(WeightStruc[1].ADC_DATA));
    TempVariable+=Len;
    DataPack[TempVariable+2]=',';TempVariable++;

    WeightStateFlage=(   WeightStruc[1].Under_Flag<<7 | (!(Dc_Power_Good))<<6 | WeightStruc[1].Over_Flag<<5 | WeightStruc[1].W2_Flag<<4  | WeightStruc[1].w_neg<<3 | WeightStruc[1].Stable_Flag<<2 | 
                         WeightStruc[1].Net_Flag<<1 | WeightStruc[1].Zero_Flag);
    Len=Change_To_Str(DataPack+TempVariable+2,WeightStateFlage);
    TempVariable+=Len;
    DataPack[TempVariable+2]=',';TempVariable++;

    TempVariable=MakePowerInfoPacket(TempVariable);

    DataPack[1]=TempVariable;
    crci=crccheck(TempVariable+2);   
    DataPack[TempVariable+2]=(uint8_t)(crci>>8);
    DataPack[TempVariable+3]=(uint8_t)(crci&0xff);
    UART3_Write(PAddress);
    for(LoopCnt=0;LoopCnt<TempVariable+4;LoopCnt++)
        UART3_Write(DataPack[LoopCnt]);
}

void  SendWeightString(void)
{

    uint32_t TempVariable;
    uint8_t Len,LoopCnt;
    uint16_t crci;
    Colpy++;
    UART3_Write('W');
    UART3_Write('=');
    UART3_Write(' ');
    Len=Change_To_Str(DataPack,labs(WeightStruc[0].View_Weight));
    for(LoopCnt=0;LoopCnt<Len;LoopCnt++)
        UART3_Write(DataPack[LoopCnt]);
}

void TozinSadr_Services_Tasks(uint8_t data)
{
    if(data=='*')
        Send_Weight(WeightStruc[Loadcell_Id].View_Weight,0,0,0,0,0,1);
    if(data=='C')
    {
        UART3_Write('C');
    }
    if(data=='@')
    {
        WeightStruc[Loadcell_Id].Tare_Flag=1;
        UART3_Write('@');
    }
    if(data=='%')
    {
        UART3_Write('%');
        Adjust_Zero(1,0,0,Loadcell_Id);
    }
/*    if(data=='0')
    {
        ReturnDivisionIndex(10,0);    
        ReturnDivisionIndex(10,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('0');
    }
    if(data=='5')
    {
        ReturnDivisionIndex(5,0);    
        ReturnDivisionIndex(5,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('5');
    }
    if(data=='2')
    {
        ReturnDivisionIndex(2,0);    
        ReturnDivisionIndex(2,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('2');
    }*/
    if(data=='S' || data=='s')
    {
        SystemParm.Set_Parameter.SelectedManufacture=TAKIN_ELECTRONIC_ARKA;
        SystemParm.Set_Parameter.AutoSend=0;
        UpdateSetting();   
        UART3_Write('S');
        UART3_Write('w');
        UART3_Write('i');
        UART3_Write('t');
        UART3_Write('c');
        UART3_Write('h');
        UART3_Write('e');
        UART3_Write('d');
        UART3_Write(' ');
        UART3_Write('T');
        UART3_Write('o');
        UART3_Write(' ');
        UART3_Write('T');
        UART3_Write('a');
        UART3_Write('k');
        UART3_Write('i');
        UART3_Write('n');
        UART3_Write(13);
        UART3_Write(10);
    }
}

void Pand_Services_Tasks(uint8_t data)
{
    if(data=='T' || data=='t')
    {
        WeightStruc[Loadcell_Id].Tare_Flag=1;
        UART3_Write('T');
    }
    if(data=='Z' || data=='z')
    {
        UART3_Write('Z');
        requestZero=true;
        Adjust_Zero(1,1,0,Loadcell_Id);
    }
    if(data=='C' || data=='c')
    {
        UART3_Write('C');
        WeightStruc[Loadcell_Id].Tare_Flag=0;
        WeightStruc[Loadcell_Id].Tare_Weight=0;
    }
/*    if(data=='0')
    {
        ReturnDivisionIndex(10,0);    
        ReturnDivisionIndex(10,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('1');
        UART3_Write('0');
        UART3_Write('g');
    }
    if(data=='5')
    {
        ReturnDivisionIndex(5,0);    
        ReturnDivisionIndex(5,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('5');
        UART3_Write('g');
    }
    if(data=='2')
    {
        ReturnDivisionIndex(2,0);    
        ReturnDivisionIndex(2,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('2');
        UART3_Write('g');
    }
    if(data=='1')
    {
        ReturnDivisionIndex(1,0);    
        ReturnDivisionIndex(1,1);    
        WeightStruc[Loadcell_Id].Divition=DivitionTable[CalibrationParametes.CalibParameter[Loadcell_Id].Divition_Index_P1];
        UpdateCalibrationParameters();            
        UART3_Write('1');
        UART3_Write('g');
    }
  */  
    if(data=='S' || data=='s')
    {
        SystemParm.Set_Parameter.SelectedManufacture=TAKIN_ELECTRONIC_ARKA;
        SystemParm.Set_Parameter.AutoSend=0;
        UpdateSetting();   
        UART3_Write('S');
        UART3_Write('w');
        UART3_Write('i');
        UART3_Write('t');
        UART3_Write('c');
        UART3_Write('h');
        UART3_Write('e');
        UART3_Write('d');
        UART3_Write(' ');
        UART3_Write('T');
        UART3_Write('o');
        UART3_Write(' ');
        UART3_Write('T');
        UART3_Write('a');
        UART3_Write('k');
        UART3_Write('i');
        UART3_Write('n');
        UART3_Write(13);
        UART3_Write(10);
    }
}


void Rs232Tasks(void )
{
    uint8_t data = 0;
    if(SerInputWrp!=SerInputRrp)
    {
        data=Ser_Read_Buf[SerInputRrp];
        SerInputRrp=rnginc(SerInputRrp,SER_MAX_READ); 
        switch(SystemParm.Set_Parameter.SelectedManufacture)
        {
            case  TOZIN_SADR : 
                TozinSadr_Services_Tasks(data);
            break;
            case  PAND: 
                Pand_Services_Tasks(data);
            break;
            case  TAKIN_ELECTRONIC_ARKA: 
                TakinTasks(data);
            break;

        }
    }
}

void checkSensitiveDisconnection(void)
{
    if(sendSensitive==true)
    {
        if(waitForSensitivReceivedWeightCounter++>MAX_SENSITIVE_TIMEOUT )
        {
            waitForSensitivReceivedWeightCounter=0;
            sendSensitive=false;
            sendNormal=true;
        }
    }
    else
       waitForSensitivReceivedWeightCounter=0; 
}



void Calc_Show_Weight(void)
{
    if(SystemParm.Set_Parameter.ViewFormat==0)
    {
        if(WeightStruc[Loadcell_Id].data_completed)
            checkSensitiveDisconnection();
        if(CalibrationParametes.CalibParameter[Loadcell_Id].IsCalibrated==1)
        {
            if(Weight_Tasks(Loadcell_Id))
            {
                AutoSendWeight();
                if( WeightStruc[Loadcell_Id].Over_Flag==true ||  WeightStruc[Loadcell_Id].Under_Flag==true)
                    Show_Dash();
                else
                {
                    DispPos=0;
                    in_data=labs(WeightStruc[Loadcell_Id].View_Weight);	
                    in_data=ChangeNumberDot(in_data);
                    Show_Front_Zero=CalibrationParametes.CalibParameter[Loadcell_Id].Point_Index+1;
                    Mask_Number=6;
                    Show_Back_Zero=0;
                    Show_Decimal_Point=1;
                    Show_Sign=1;
                    Load_Display_New();
                }
            }
        }
    }
    else
    {
        if(SystemParm.Set_Parameter.ViewFormat==1)
        {
            if(WeightStruc[0].data_completed && WeightStruc[1].data_completed)
                checkSensitiveDisconnection();

            if(CalibrationParametes.CalibParameter[0].IsCalibrated==1 && CalibrationParametes.CalibParameter[1].IsCalibrated==1)
            {   
                if(Weight_Tasks(0) && Weight_Tasks(1))
                {
                    AutoSendWeight();
                    if( WeightStruc[0].Over_Flag==true ||  WeightStruc[0].Under_Flag==true || WeightStruc[1].Over_Flag==true ||  WeightStruc[1].Under_Flag==true)
                        Show_Dash();
                    else
                    {
                        DispPos=0;
                        in_data=labs(WeightStruc[0].View_Weight+WeightStruc[1].View_Weight);	
                        in_data=ChangeNumberDot(in_data);
                        Show_Front_Zero=CalibrationParametes.CalibParameter[0].Point_Index+1;
                        Mask_Number=6;
                        Show_Back_Zero=0;
                        Show_Decimal_Point=1;
                        Show_Sign=1;
                        Load_Display_New();
                    }
                }
            }
        }
        else
        {
            if(SystemParm.Set_Parameter.ViewFormat==2)
            {
                if(WeightStruc[SwitchWeight].data_completed)
                    checkSensitiveDisconnection();
                if(CalibrationParametes.CalibParameter[SwitchWeight].IsCalibrated==1)
                {   
                    if(Weight_Tasks(SwitchWeight))
                    {
                        AutoSendWeight();
                        if( WeightStruc[SwitchWeight].Over_Flag==true ||  WeightStruc[SwitchWeight].Under_Flag==true)
                            Show_Dash();
                        else
                        {
                            DispPos=0;
                            in_data=labs(WeightStruc[SwitchWeight].View_Weight);	
                            in_data=ChangeNumberDot(in_data);
                            Show_Front_Zero=CalibrationParametes.CalibParameter[SwitchWeight].Point_Index+1;
                            Mask_Number=6;
                            Show_Back_Zero=0;
                            Show_Decimal_Point=1;
                            Show_Sign=1;
                            Load_Display_New();
                        }
                    }
                }
            }
        }
    }
    UpdateDisplayData();
}

extern fir_t fir_ftr;

void AutoSendWeight(void)
{
    if(SystemParm.Set_Parameter.AutoSend)
    {
        if(SystemParm.Set_Parameter.SelectedManufacture==2)
        {
            Send_Weight(WeightStruc[Loadcell_Id].View_Weight,0,true,false,true,8,false);
            
 //           Send_Weight(fir_ftr.filtered,0,true,false,true,8,false);
        }
        else
        {
            if(SystemParm.Set_Parameter.AutoSendNewOld)
                SendTakinWeight(0x14,0x01);
            else
                WeightTransferManage(0x14,0x01);
        }
    } 
}



void Pc_Command_Services(void)
{
    static bool W1Completed=0,W2Completed=0;
    switch(appData.state)
    {
        case APP_STATE_COUNT_DOWN:
            if(ResetCause!=0xa3)
            {
                CountDownService();
                rxHead=rxTail;
            }
            else
            {
                WeightStruc[0].offset=SavedOffset[0];
                WeightStruc[1].offset=SavedOffset[1];
                appData.state=APP_STATE_NORMAL_MODE;
            }
        break;
        case APP_STATE_CALIBRATION:
            CalibrateService(Loadcell_Id);
            break;
        case APP_STATE_NORMAL_MODE:
            Calc_Show_Weight();
            Rs232Tasks();
            SensitiveScaleReadData();
            break;
        case APP_STATE_CALIBRATION_ERROR:
            WriteErrorMessage();
            appData.state=APP_STATE_NORMAL_MODE;
        break;
    }
}

void UART3_Write( uint8_t byte)
{
  //  while(UART3_IsTxReady() == 0);
    Ser_Write_Buf[SerOutputWrp]=byte;
	SerOutputWrp=rnginc(SerOutputWrp,SER_MAX_WRITE);     
    IEC1bits.U3TXIE = 1;
}

bool UART3_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}

bool UART3_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART3_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    return (size != 0);
}

bool UART3_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (bool)U3STAbits.TRMT;
    }
    
    return false;
}


/*******************************************************************************

  !!! Deprecated API !!!
  !!! These functions will not be supported in future releases !!!

*******************************************************************************/

static uint8_t UART3_RxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_rxTail = (uint8_t*)rxTail;
    
    if (snapshot_rxTail < rxHead) 
    {
        size = ( UART3_CONFIG_RX_BYTEQ_LENGTH - (rxHead-snapshot_rxTail));
    }
    else
    {
        size = ( (snapshot_rxTail - rxHead));
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

static uint8_t UART3_TxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART3_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

unsigned int __attribute__((deprecated)) UART3_ReadBuffer( uint8_t *buffer ,  unsigned int numbytes)
{
    unsigned int rx_count = UART3_RxDataAvailable();
    unsigned int i;
    
    if(numbytes < rx_count)
    {
        rx_count = numbytes;
    }
    
    for(i=0; i<rx_count; i++)
    {
        *buffer++ = UART3_Read();
    }
    
    return rx_count;    
}

unsigned int __attribute__((deprecated)) UART3_WriteBuffer( uint8_t *buffer , unsigned int numbytes )
{
    unsigned int tx_count = UART3_TxDataAvailable();
    unsigned int i;
    
    if(numbytes < tx_count)
    {
        tx_count = numbytes;
    }
    
    for(i=0; i<tx_count; i++)
    {
        UART3_Write(*buffer++);
    }
    
    return tx_count;  
}

UART3_TRANSFER_STATUS __attribute__((deprecated)) UART3_TransferStatusGet (void )
{
    UART3_TRANSFER_STATUS status = 0;
    uint8_t rx_count = UART3_RxDataAvailable();
    uint8_t tx_count = UART3_TxDataAvailable();
    
    switch(rx_count)
    {
        case 0:
            status |= UART3_TRANSFER_STATUS_RX_EMPTY;
            break;
        case UART3_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART3_TRANSFER_STATUS_RX_FULL;
            break;
        default:
            status |= UART3_TRANSFER_STATUS_RX_DATA_PRESENT;
            break;
    }
    
    switch(tx_count)
    {
        case 0:
            status |= UART3_TRANSFER_STATUS_TX_FULL;
            break;
        case UART3_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART3_TRANSFER_STATUS_TX_EMPTY;
            break;
        default:
            break;
    }

    return status;    
}

uint8_t __attribute__((deprecated)) UART3_Peek(uint16_t offset)
{
    uint8_t *peek = rxHead + offset;
    
    while(peek > (rxQueue + UART3_CONFIG_RX_BYTEQ_LENGTH))
    {
        peek -= UART3_CONFIG_RX_BYTEQ_LENGTH;
    }
    
    return *peek;
}

bool __attribute__((deprecated)) UART3_ReceiveBufferIsEmpty (void)
{
    return (UART3_RxDataAvailable() == 0);
}

bool __attribute__((deprecated)) UART3_TransmitBufferIsFull(void)
{
    return (UART3_TxDataAvailable() == 0);
}

uint32_t __attribute__((deprecated)) UART3_StatusGet (void)
{
    return U3STA;
}

unsigned int __attribute__((deprecated)) UART3_TransmitBufferSizeGet(void)
{
    if(UART3_TxDataAvailable() != 0)
    { 
        if(txHead > txTail)
        {
            return(txHead - txTail);
        }
        else
        {
            return(UART3_CONFIG_TX_BYTEQ_LENGTH - (txTail - txHead));
        }
    }
    return 0;
}

unsigned int __attribute__((deprecated)) UART3_ReceiveBufferSizeGet(void)
{
    if(UART3_RxDataAvailable() != 0)
    {
        if(rxHead > rxTail)
        {
            return(rxHead - rxTail);
        }
        else
        {
            return(UART3_CONFIG_RX_BYTEQ_LENGTH - (rxTail - rxHead));
        } 
    }
    return 0;
}

void __attribute__((deprecated)) UART3_Enable(void)
{
    U3STASET = _U3STA_UTXEN_MASK;
    U3STASET = _U3STA_URXEN_MASK;
    U3MODESET = _U3MODE_ON_MASK;
}

void __attribute__((deprecated)) UART3_Disable(void)
{
    U3STACLR = _U3STA_UTXEN_MASK;
    U3STACLR = _U3STA_URXEN_MASK;
    U3MODECLR = _U3MODE_ON_MASK;
}

