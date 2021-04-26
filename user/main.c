/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_clk.h"

#include "stdio.h"

#include "string.h"

#define UART1_PRINTF(fmt, ...) memset(devUart_dataSendBuf, 0, UART_DATA_RX_QLEN * sizeof(char));\
							   snprintf((char *)devUart_dataSendBuf, UART_DATA_RX_QLEN, fmt, ##__VA_ARGS__);\
							   usrApp_uart1StringSend((char *)devUart_dataSendBuf)

 #define UART_DEVDRV_PROTOCOL_HEAD_M		0xEF
 #define UART_DEVDRV_PROTOCOL_HEAD_S		0xEE
								   
#define UART_DATA_RX_TOUT					15	//串口接收超时时间
#define UART_DATA_RX_QLEN 					32
								   
#define DEV_DIMMER_MODULATION_TIME_LAG		10	//调光pwm调制相位起点滞后延迟节拍数 --延迟10个拍，防止下降沿中断为能跟上相位起点
								   
#define DEV_RELAY_MAGNETIC_HOLD_MAX_NUM		3   //磁保持继电器最大个数
#define DEV_RELAY_MAGNETIC_HOLD_TIME_KEEP	100 //磁保持继电器脉冲电平保持时间，单位：ms
								   
#define L8_DEVICE_TYPEDEFINE_BASE_ADDR		(0x49)
								   
#define MAGNETIC_HOLD_RELAY1_ACT_ON()		GPIO_WriteHigh(GPIOC, GPIO_PIN_7);GPIO_WriteLow(GPIOD, GPIO_PIN_2)
#define MAGNETIC_HOLD_RELAY1_ACT_OFF()		GPIO_WriteLow(GPIOC, GPIO_PIN_7);GPIO_WriteHigh(GPIOD, GPIO_PIN_2)
#define MAGNETIC_HOLD_RELAY1_ACT_RSV()		GPIO_WriteLow(GPIOC, GPIO_PIN_7);GPIO_WriteLow(GPIOD, GPIO_PIN_2)
								   
#define MAGNETIC_HOLD_RELAY2_ACT_ON()		GPIO_WriteHigh(GPIOD, GPIO_PIN_3);GPIO_WriteLow(GPIOC, GPIO_PIN_3)
#define MAGNETIC_HOLD_RELAY2_ACT_OFF()		GPIO_WriteLow(GPIOD, GPIO_PIN_3);GPIO_WriteHigh(GPIOC, GPIO_PIN_3)
#define MAGNETIC_HOLD_RELAY2_ACT_RSV()		GPIO_WriteLow(GPIOD, GPIO_PIN_3);GPIO_WriteLow(GPIOC, GPIO_PIN_3)
								   
#define MAGNETIC_HOLD_RELAY3_ACT_ON()		GPIO_WriteHigh(GPIOC, GPIO_PIN_4);GPIO_WriteLow(GPIOA, GPIO_PIN_3)
#define MAGNETIC_HOLD_RELAY3_ACT_OFF()		GPIO_WriteLow(GPIOC, GPIO_PIN_4);GPIO_WriteHigh(GPIOA, GPIO_PIN_3)
#define MAGNETIC_HOLD_RELAY3_ACT_RSV()		GPIO_WriteLow(GPIOC, GPIO_PIN_4);GPIO_WriteLow(GPIOA, GPIO_PIN_3)
						
typedef enum{

	devTypeDef_mulitSwOneBit = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 1),
	devTypeDef_mulitSwTwoBit,
	devTypeDef_mulitSwThreeBit,
	devTypeDef_dimmer,
	devTypeDef_fans,
	devTypeDef_scenario,
	devTypeDef_curtain,
	devTypeDef_thermostat = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 0x1E),
	devTypeDef_heater = (L8_DEVICE_TYPEDEFINE_BASE_ADDR + 0x1F),
}devTypeDef_enum;
								   
typedef struct __stt_uartProtocolFormat{

	uint8_t pHead;
	uint8_t command;
	uint8_t data;
	uint8_t pTailCheck;
	
}stt_protocolFmtUartDevDrv;
								   
/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
								   
static uint16_t dataReq_actionCounter = 1;

static struct __stt_dimmer_debugAttr{

	uint8_t frepCounter;
	uint8_t frepConfirm;
}devDimmer_debugParam;
			
static struct __stt_dimmer_attrFreq{ //

	u8 breightness;
	
	u8 periodBeat_cfm; //
	u8 periodBeat_counter; //
	
	u16 pwm_actEN:1; //
	u16 pwm_actCounter:15; //
	u8	pwm_actWaitCounter;
	
}devDimmer_freqParam;

static struct __stt_relayMagneticHold_attrRunning{ //

	uint16_t actionTrig_flg:1;
	uint16_t actionTrig_status:1;
	uint16_t timeKeepCounter:14;
	
}devOther_realyMagneticHoldParam[DEV_RELAY_MAGNETIC_HOLD_MAX_NUM];

static struct __stt_deviceRunningParam{

	uint8_t deviceType;
	
	struct __stt_devStatusDef{
	
		uint8_t deviceDimmer_brightness;
		uint8_t deviceOther_relayVal;
		
	}devStatus;
	
}deviceRunningParam = {

	.deviceType = devTypeDef_mulitSwThreeBit,
};

static struct __stt_dataRcv_attrJudge{

		  uint8_t rcvStart_flg:1;
	const uint8_t rcvTimeout_period:7;
		
		  uint8_t rcvTimeout_counter;
		  
}devUart_dataRcvJudgeAttr = {

	.rcvStart_flg 		= 0,
	.rcvTimeout_counter = 0,
	.rcvTimeout_period = UART_DATA_RX_TOUT,
};
static struct __stt_dataRcv{

	uint8_t dataRcvTout_trigFlg:1;
	uint8_t dataRcvTout_completeFlg:1;
	
	uint8_t dataRcv_tmp[UART_DATA_RX_QLEN];
	uint8_t dataRcvIst_tmp;
	uint8_t dataRcv_cfm[UART_DATA_RX_QLEN];
	uint8_t dataRcvLen_cfm;
	
}devUart_dataRcvBuf = {0};
static uint8_t devUart_dataSendBuf[UART_DATA_RX_QLEN] = {0};

void bspInit_timer1(void){

	TIM1_TimeBaseInit(0, TIM1_COUNTERMODE_UP, 800, 0); //0.0000000625 * 800 = 50us
	TIM1_GenerateEvent(TIM1_EVENTSOURCE_UPDATE);
	TIM1_ARRPreloadConfig(ENABLE); 
	TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE); //iptr enable
	TIM1_Cmd(ENABLE); 
}

static void usrApp_uart1StringSend(char *str);



void delay(u16 ms)
{
  unsigned int x , y;
  for(x = ms; x > 0; x--)           /*  通过一定周期循环进行延时*/
    for(y = 3000 ; y > 0 ; y--);
}

static 
unsigned char frame_Check(unsigned char frame_temp[], u8 check_num){
  
	unsigned char loop 		= 0;
	unsigned char val_Check = 0;
	
//	for(loop = 0; loop < check_num; loop ++){
//	
//		val_Check += frame_temp[loop];
//	}
//	
//	val_Check  = ~val_Check;
//	val_Check ^= 0xa7;
	
	for(loop = 0; loop < check_num; loop ++){
	
		val_Check ^= frame_temp[loop];
	}
	
	return val_Check;
}

static void bspInit_exti_sourceWaveMeasure(void){

	EXTI_DeInit();
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
	
	GPIO_DeInit(GPIOA);
	GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_PU_IT);          
	GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
}

static void bspInit_watchDog(void){
	
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_128);
	IWDG_SetReload(251); //500ms
	IWDG_Enable();
}

static void bspInit_gpio_hardwareRelay(void){
	
	GPIO_DeInit(GPIOB);
	GPIO_Init(GPIOB, GPIO_PIN_ALL, GPIO_MODE_OUT_PP_LOW_FAST); 
	GPIO_WriteLow(GPIOB, GPIO_PIN_ALL);
	
	GPIO_DeInit(GPIOC);
	GPIO_Init(GPIOC, GPIO_PIN_ALL, GPIO_MODE_OUT_PP_LOW_FAST); 
	GPIO_WriteLow(GPIOC, GPIO_PIN_ALL);
	
	GPIO_DeInit(GPIOD);
	GPIO_Init(GPIOD, GPIO_PIN_ALL, GPIO_MODE_OUT_PP_LOW_FAST); 
	GPIO_WriteLow(GPIOD, GPIO_PIN_ALL);
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT); //串口端口属性重新初始化
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_OD_LOW_SLOW); //串口端口属性重新初始化  --防止esp32串口RX开机不启
}

static void bspInit_uart_dataTrans(void){

	UART1_DeInit();
	UART1_Init((u32)115200, 
				UART1_WORDLENGTH_8D, 
				UART1_STOPBITS_1,
				UART1_PARITY_NO, 
				UART1_SYNCMODE_CLOCK_DISABLE, 
				UART1_MODE_TXRX_ENABLE);
	UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
//	UART1_ITConfig(UART1_IT_TXE, ENABLE);
	UART1_Cmd(ENABLE);
}

static void bsp_initialize(void){
	
	disableInterrupts();
	
    CLK_HSECmd(DISABLE); //关闭外部高速振荡器
    CLK_HSICmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	
	ITC_DeInit();
	ITC_SetSoftwarePriority(ITC_IRQ_TIM1_OVF, ITC_PRIORITYLEVEL_1);
	ITC_SetSoftwarePriority(ITC_IRQ_UART1_RX, ITC_PRIORITYLEVEL_1);
	ITC_SetSoftwarePriority(ITC_IRQ_PORTA, 	  ITC_PRIORITYLEVEL_3);
	
	bspInit_timer1();
	bspInit_exti_sourceWaveMeasure();
	bspInit_watchDog();
	bspInit_gpio_hardwareRelay();
	bspInit_uart_dataTrans();
	
	enableInterrupts();	
	
//	UART1_PRINTF("dhaosfhoa");
}

void interruptHandleFunc_timer1(void){ //@50us

	const  uint16_t period_1second = 20000;
	static uint16_t counter_1second = 0; 
	
	const  uint16_t period_10ms = 200;
	static uint16_t counter_10ms = 0; 
	
	const  uint8_t period_1ms 	= 20;
	static uint8_t counter_1ms 	= 0; 
	
	const  uint16_t period_dimmerFollow = 400;
	static uint16_t counter_dimmerFollow = 0; 
	
	TIM1_ClearITPendingBit(TIM1_IT_UPDATE); //flg clear!!!
	
	{ //调光业务
		
		uint8_t freq_periodBeatHalf = devDimmer_freqParam.periodBeat_cfm / 2;
		
		devDimmer_freqParam.periodBeat_counter ++;
		
		if(devDimmer_freqParam.pwm_actEN){
			
			if(devDimmer_freqParam.pwm_actWaitCounter)devDimmer_freqParam.pwm_actWaitCounter --;
			else{
			
				devDimmer_freqParam.pwm_actCounter ++;
				if(devDimmer_freqParam.pwm_actCounter <= devDimmer_freqParam.breightness && devDimmer_freqParam.pwm_actCounter < freq_periodBeatHalf){ //
					
					GPIO_WriteHigh(GPIOA, GPIO_PIN_2);
					
				}
				else
				{
					GPIO_WriteLow(GPIOA, GPIO_PIN_2);
					
					devDimmer_freqParam.pwm_actCounter = 0;
					devDimmer_freqParam.pwm_actEN = 0;
				}
			}
		}
	}
	
	if(counter_dimmerFollow < period_dimmerFollow)counter_dimmerFollow ++; //亮度惰性跟随
	else{
	
		counter_dimmerFollow = 0;
		
		if(devDimmer_freqParam.breightness != deviceRunningParam.devStatus.deviceDimmer_brightness){
		
			(devDimmer_freqParam.breightness < deviceRunningParam.devStatus.deviceDimmer_brightness)?
				(devDimmer_freqParam.breightness ++):
				(devDimmer_freqParam.breightness --);
		}
	}
	
	//**************************1s专用**************************//
	if(counter_1second < period_1second)counter_1second ++;
	else{
	
		counter_1second = 0;
		
		//调光调试数据更新
		devDimmer_debugParam.frepConfirm = devDimmer_debugParam.frepCounter;
		devDimmer_debugParam.frepCounter = 0;
	}
	
	//**************************10ms专用************************//
	if(counter_10ms < period_10ms)counter_10ms ++;
	else{
	
		counter_10ms = 0;
		
		IWDG_ReloadCounter(); //dog feed
	}
	
	//**************************1ms专用**************************//
	if(counter_1ms < period_1ms)counter_1ms ++;
	else{
	
		counter_1ms = 0;
		
		if(dataReq_actionCounter)dataReq_actionCounter --;
		
		if(devUart_dataRcvJudgeAttr.rcvStart_flg){
		
			if(devUart_dataRcvJudgeAttr.rcvTimeout_counter)devUart_dataRcvJudgeAttr.rcvTimeout_counter --;
			else{
				
				devUart_dataRcvBuf.dataRcvTout_trigFlg = 1;
				
				devUart_dataRcvJudgeAttr.rcvStart_flg = 0;
			}
		}
		
		{	//磁保持继电器运行业务
		
			uint8_t loop = 0;
			
			for(loop = 0; loop < DEV_RELAY_MAGNETIC_HOLD_MAX_NUM; loop ++){
			
				if(devOther_realyMagneticHoldParam[loop].actionTrig_flg){
					
					if(devOther_realyMagneticHoldParam[loop].timeKeepCounter){
					
						devOther_realyMagneticHoldParam[loop].timeKeepCounter --;
						
						switch(loop){
						
							case 0:{
							
								if(devOther_realyMagneticHoldParam[loop].actionTrig_status){
								
									MAGNETIC_HOLD_RELAY1_ACT_ON();
								}
								else
								{
									MAGNETIC_HOLD_RELAY1_ACT_OFF();
								}
							
							}break;
							
							case 1:{
							
								if(devOther_realyMagneticHoldParam[loop].actionTrig_status){
								
									MAGNETIC_HOLD_RELAY2_ACT_ON();
								}
								else
								{
									MAGNETIC_HOLD_RELAY2_ACT_OFF();
								}
							
							}break;
							
							case 2:{
							
								if(devOther_realyMagneticHoldParam[loop].actionTrig_status){
								
									MAGNETIC_HOLD_RELAY3_ACT_ON();
								}
								else
								{
									MAGNETIC_HOLD_RELAY3_ACT_OFF();
								}
							
							}break;
							
							default:break;
						}
					}
					else
					{
						devOther_realyMagneticHoldParam[loop].actionTrig_flg = 0;
					}
				}
				else
				{
					switch(loop){
					
						case 0:MAGNETIC_HOLD_RELAY1_ACT_RSV();break;
						case 1:MAGNETIC_HOLD_RELAY2_ACT_RSV();break;
						case 2:MAGNETIC_HOLD_RELAY3_ACT_RSV();break;
						
						default:break;
					}
				}
			}
		}
	}
}

void interruptHandleFunc_uartTx(void){

	if(UART1_GetFlagStatus(UART1_FLAG_TXE) == SET){
	
		UART1_ClearITPendingBit(UART1_IT_TXE);
	}
}

void interruptHandleFunc_uartRx(void){

	UART1_ClearITPendingBit(UART1_IT_RXNE);
	
	devUart_dataRcvBuf.dataRcv_tmp[devUart_dataRcvBuf.dataRcvIst_tmp ++] = UART1_ReceiveData8();
	if(devUart_dataRcvBuf.dataRcvIst_tmp >= (UART_DATA_RX_QLEN - 1))
		devUart_dataRcvBuf.dataRcvIst_tmp = (UART_DATA_RX_QLEN - 1);
	
	if(!devUart_dataRcvJudgeAttr.rcvStart_flg)
		devUart_dataRcvJudgeAttr.rcvStart_flg = 1;
	devUart_dataRcvJudgeAttr.rcvTimeout_counter = devUart_dataRcvJudgeAttr.rcvTimeout_period;
}

void interruptHandleFunc_exitElecSouceMeasure(void){
	
	devDimmer_freqParam.periodBeat_cfm = devDimmer_freqParam.periodBeat_counter;
	devDimmer_freqParam.periodBeat_counter = 0;
	
	devDimmer_freqParam.pwm_actEN = 1;
	devDimmer_freqParam.pwm_actWaitCounter = DEV_DIMMER_MODULATION_TIME_LAG;
	
	devDimmer_debugParam.frepCounter ++;
}

static void usrApp_uart1DataSend(uint8_t *dptr, uint16_t len){

	uint16_t loop = 0;
	uint16_t uartData_txLen = 0;
	
	(len > UART_DATA_RX_QLEN)?
		(uartData_txLen = UART_DATA_RX_QLEN):
	 	(uartData_txLen = len);

	memset(devUart_dataSendBuf, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
	memcpy(devUart_dataSendBuf, dptr, sizeof(uint8_t) * uartData_txLen);
	
	for(loop = 0; loop < uartData_txLen; loop ++){
	
		while((UART1_GetFlagStatus(UART1_FLAG_TXE)==RESET));
		UART1_SendData8(devUart_dataSendBuf[loop]);
		while((UART1_GetFlagStatus(UART1_FLAG_TC)==RESET));
	}
}

static void usrApp_uart1StringSend(char *str){

	uint16_t strLen = strlen((const char*)str);
	
	while(*str != '\0'){
	
		while((UART1_GetFlagStatus(UART1_FLAG_TXE)==RESET));
		UART1_SendData8(*str);
		while((UART1_GetFlagStatus(UART1_FLAG_TC)==RESET));
		
		str ++;
	}
}

static void process_uartDataRcv_timeOut_funcHandle(void){
	
	if(devUart_dataRcvBuf.dataRcvTout_trigFlg){
	
		devUart_dataRcvBuf.dataRcvTout_trigFlg = 0;
		
		memset(devUart_dataRcvBuf.dataRcv_cfm, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		memcpy(devUart_dataRcvBuf.dataRcv_cfm, devUart_dataRcvBuf.dataRcv_tmp, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		devUart_dataRcvBuf.dataRcvLen_cfm = devUart_dataRcvBuf.dataRcvIst_tmp;
		memset(devUart_dataRcvBuf.dataRcv_tmp, 0, sizeof(uint8_t) * UART_DATA_RX_QLEN);
		devUart_dataRcvBuf.dataRcvIst_tmp = 0;
		
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 1;
	}
}

static void devTest(void){
		
	process_uartDataRcv_timeOut_funcHandle();
	
//	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){
//		
//		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
//	
//		if(!strcmp("open", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
//		
//			usrApp_uart1StringSend("ok\n");
//		}
//		else
//		{
//			usrApp_uart1DataSend(devUart_dataRcvBuf.dataRcv_cfm, devUart_dataRcvBuf.dataRcvLen_cfm);
//		}
//	}
//
//	delay(1000);
//	UART1_PRINTF("soft mark\n");
//
//	if(!dataReq_actionCounter){
//		
//		dataReq_actionCounter = 1000;
//		
//		usrApp_uart1StringSend("hard mark\n");
//		
//		GPIO_WriteReverse(GPIOB, GPIO_PIN_5);
//	}

	if(!dataReq_actionCounter){
		
		dataReq_actionCounter = 2000;
		
		UART1_PRINTF("source freq:%dHz, loadVal:%d\n", devDimmer_debugParam.frepConfirm,
													   deviceRunningParam.devStatus.deviceDimmer_brightness);
	}
	
	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){
	
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
		
		if(!strcmp("open", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
		
			usrApp_uart1StringSend("ok\n");
			GPIO_WriteLow(GPIOB, GPIO_PIN_5);
		}
		else
		if(!strcmp("close", (const char *)devUart_dataRcvBuf.dataRcv_cfm)){
		
			usrApp_uart1StringSend("ok\n");
			GPIO_WriteHigh(GPIOB, GPIO_PIN_5);
		}
		else
		if(!memcmp("brightness:", (const uint8_t *)devUart_dataRcvBuf.dataRcv_cfm, 11)){
		
			uint8_t brightness = ((devUart_dataRcvBuf.dataRcv_cfm[11] - '0') * 10) +\
								 ((devUart_dataRcvBuf.dataRcv_cfm[12] - '0') * 1);
			
			if(brightness <= 100){
			
				usrApp_uart1StringSend("ok\n");
				deviceRunningParam.devStatus.deviceDimmer_brightness = brightness;
			}
			else
			{
				usrApp_uart1StringSend("format err\n");
			}
		}
	}
}

static void usrApp_deviceDriverByMcu_applicationExecute(uint8_t cmd, uint8_t dat){

	uint8_t loop = 0;
	
	switch(cmd){
	
		case 1:{
		
			deviceRunningParam.deviceType = dat;
			memset(&deviceRunningParam.devStatus, 0, sizeof(struct __stt_devStatusDef));
			
		}break;
		
		case 2:{
		
			switch(deviceRunningParam.deviceType){
			
				case devTypeDef_scenario:{}break;
				
				case devTypeDef_dimmer:{
				
					deviceRunningParam.devStatus.deviceDimmer_brightness = dat;
				
				}break;
				
				case devTypeDef_mulitSwOneBit:
				case devTypeDef_mulitSwTwoBit:
				case devTypeDef_mulitSwThreeBit:
				case devTypeDef_curtain:
				case devTypeDef_fans:
				case devTypeDef_thermostat:
                case devTypeDef_heater:{
                
					deviceRunningParam.devStatus.deviceOther_relayVal = dat;
				
					for(loop = 0; loop < DEV_RELAY_MAGNETIC_HOLD_MAX_NUM; loop ++){
					
						devOther_realyMagneticHoldParam[loop].actionTrig_flg  = 1;
						devOther_realyMagneticHoldParam[loop].timeKeepCounter = DEV_RELAY_MAGNETIC_HOLD_TIME_KEEP;
						
						(deviceRunningParam.devStatus.deviceOther_relayVal & (1 << loop))?
							(devOther_realyMagneticHoldParam[loop].actionTrig_status = 1):
						 	(devOther_realyMagneticHoldParam[loop].actionTrig_status = 0);
					}
                
                }break;

				default:break;
			}
			
		}break;
		
		default:break;
	}
}

static void process_bussinessDeviceDriver(void){

	if(devUart_dataRcvBuf.dataRcvTout_completeFlg){ //超时响应
	
		devUart_dataRcvBuf.dataRcvTout_completeFlg = 0;
	
		if((UART_DEVDRV_PROTOCOL_HEAD_M == devUart_dataRcvBuf.dataRcv_cfm[0]) && //帧头确认
		   (sizeof(stt_protocolFmtUartDevDrv) == devUart_dataRcvBuf.dataRcvLen_cfm)){ //长度确认
		
			stt_protocolFmtUartDevDrv dataTemp = {0};
			
			memcpy(&dataTemp, devUart_dataRcvBuf.dataRcv_cfm, sizeof(stt_protocolFmtUartDevDrv));
			if(frame_Check(devUart_dataRcvBuf.dataRcv_cfm, sizeof(stt_protocolFmtUartDevDrv) - 1) == dataTemp.pTailCheck){ //尾校验
			
				usrApp_deviceDriverByMcu_applicationExecute(dataTemp.command, dataTemp.data); //动作执行

				dataTemp.pHead = UART_DEVDRV_PROTOCOL_HEAD_S;
				dataTemp.pTailCheck = frame_Check((uint8_t *)&dataTemp, sizeof(stt_protocolFmtUartDevDrv) - 1);
				usrApp_uart1DataSend((uint8_t *)&dataTemp, sizeof(stt_protocolFmtUartDevDrv)); //响应回复
				
			}
			else
			{
				usrApp_uart1StringSend("check err\n");
			}
		}	
		else
		{
			usrApp_uart1StringSend("format err\n");
		}
	}
}

void main(void)
{
	
	bsp_initialize();

	for(;;){
	
		process_uartDataRcv_timeOut_funcHandle(); //串口超时接收业务
		
//		devTest();
		process_bussinessDeviceDriver();
	}
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
