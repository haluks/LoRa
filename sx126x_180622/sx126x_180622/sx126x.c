/*
 * sx126x.c
 *
 * Created: 9.01.2021 16:59:54
 *  Author: haluk
 */ 
#include "sx126x.h"

static uint8_t sx_rx_bas=0,sx_rx_son=0,sx_tx_bas=0,sx_tx_son=0;
static uint8_t sx_rx_ring[sx126x_Rx_Boyut];
static uint8_t sx_tx_ring[sx126x_Tx_Boyut];
volatile chipModeStatus_t sxModeStat;
volatile ackStatus_t ackstat=ACK_NOT_WAIT;
volatile txStatus_t txstat=TX_AGAIN_ON;
volatile rxStatus_t rxstat=MSG_DONE;
volatile uint8_t ackerr=0;
volatile uint8_t irqflag=0;
cmdStatus_t sxCmdStat;
uint32_t sendtime=0,receivetime=0;
uint8_t rxBufferAdd=0x00,txBufferAdd=0x00;

sx126x_t sx1262;
#if DEBUG_KONTROL
#include <stdio.h>
char str[15];
#endif

ISR (IRQ1){		
	if (IRQ_PIN&(1<<DIO1))	{			
		uint16_t irq1stat=GetIrqStat();
		ClearIrq(IRQ_ALL);	
		#if DEBUG_KONTROL
		sprintf(str,"irq:%d\n",irq1stat);
		uart_dizi(str);
		#endif			
		switch (irq1stat)		{
			case (IRQ_TX_DONE):
			sxModeStat=MODE_STDBY_RC;
			txstat=TX_AGAIN_ON;
			#if DEBUG_KONTROL
			uart_dizi("irq-txdone\n");
			#endif
			if (ACK_STATE==ACK_ON){
				if (ackstat==ACK_SENDING){
					#if DEBUG_KONTROL
					uart_dizi("irq-acksend\n");
					#endif
					ackstat=ACK_NOT_WAIT;
					}else{
					sendtime=millis();
					#if DEBUG_KONTROL
					uart_dizi("irq-ackwait\n");
					#endif
					ackstat=ACK_WAIT;
				}
			}			
			break;
			case (IRQ_RX_DONE):
			#if DEBUG_KONTROL
			uart_dizi("irq-rxdone\n");
			#endif
			RX_DIS;
			receivetime=millis();
			sxModeStat=MODE_STDBY_RC;
			rxstat=MSG_RCV;
			break;
			case (IRQ_RX_DONE|IRQ_CRC_ERR):
			#if DEBUG_KONTROL
			uart_dizi("irq-crcerr\n");
			#endif
			sxModeStat=MODE_STDBY_RC;
			break;
			case (IRQ_CAD_DET):
			#if DEBUG_KONTROL
			uart_dizi("irq-caddet\n");
			#endif
			sxModeStat=MODE_RX;
			break;
			case (IRQ_CAD_DET|IRQ_CAD_DONE)://kanal hareket tespit edildi
			sxModeStat=MODE_RX;
			#if DEBUG_KONTROL
			uart_dizi("irq-caddonedet\n");
			#endif
			break;
			case (IRQ_CAD_DONE)://kanal hareket tespit edildi
			sxModeStat=MODE_CAD;
			#if DEBUG_KONTROL
			uart_dizi("irq-caddone\n");
			#endif
			break;
			case (IRQ_TIMEOUT):
			if (sxModeStat==MODE_TX){
				if (txstat==TX_AGAIN_ON){
					txstat=TX_AGAIN_OFF;
					sxModeStat=MODE_TX;
					#if DEBUG_KONTROL
					uart_dizi("irq-txtime-ilk\n");
					#endif
					}else{
					txstat=TX_AGAIN_ON;
					//sxModeStat=MODE_STDBY_RC;
					sx126x_baslat();
					#if DEBUG_KONTROL
					uart_dizi("irq-txtime-son\n");
					#endif
				}				
				}else if((sxModeStat==MODE_RX)){
				sxModeStat=MODE_STDBY_RC;
				#if DEBUG_KONTROL
				uart_dizi("irq-rxtime\n");
				#endif
			}
			break;
			default:
			sxModeStat=MODE_STDBY_RC;
			#if DEBUG_KONTROL
			uart_dizi("irq-default\n");
			#endif
			break;
	}	
	irqflag++;
	}	
}
void Sx_Mode(){
	if (ACK_STATE==ACK_ON){
		if ((ackerr<=2)&&(ackstat==ACK_WAIT)&& ((uint32_t)millis()-sendtime>=ACK_WAIT_TIME)){
			sendtime=millis();
			#if DEBUG_KONTROL
			uart_dizi("acksuredoldu\n");
			#endif
			ackerr++;
			if (ackerr>2){
				ackstat=ACK_NOT_WAIT;
				}else{
				SetTx();
			}			
		}
		if (ackstat==ACK_SEND){
			if ((uint32_t)millis()-receivetime>=ACK_SEND_TIME){
				receivetime=millis();
				#if DEBUG_KONTROL
				uart_dizi("ackgonderildi\n");
				#endif
				ackstat=ACK_SENDING;
				Sx_Senddata(ACK);
			}
		}
	}	
	if (irqflag>0){
		irqflag=0;
	
	switch (sxModeStat){
		
		case MODE_CAD:
		#if DEBUG_KONTROL
		uart_dizi("mode-cad\n");
		#endif
		SetCad();		
		break;
		case MODE_STDBY_RC:
		#if DEBUG_KONTROL
		uart_dizi("mode-stdby\n");
		#endif			
			SetStdby(STDBY_RC);
			if (rxstat==MSG_RCV){//rxdone
				#if DEBUG_KONTROL
				uart_dizi("msggeldi\n");
				#endif
				Sx_ReadRcv();
				if (ACK_STATE==ACK_ON){
					if (ackstat==ACK_WAIT){
							uint8_t _ack=Sx_Read();
							if (_ack==ACK){
								#if DEBUG_KONTROL
								uart_dizi("ack-geldi\n");
								#endif							
								ackerr=0;
								ackstat=ACK_NOT_WAIT;							
							}
						}else {
						#if DEBUG_KONTROL
						uart_dizi("ack-msgalindi\n");
						#endif
						ackstat=ACK_SEND;						
					}
				}				
			}
			if (rxstat==MSG_DONE){	
				#if DEBUG_KONTROL
				uart_dizi("msgdone\n");	
				#endif	
				//SetCad();// bekleme modu
				SetRxContinuous();//bekleme modu
			}		
		break;
		case MODE_RX:
		#if DEBUG_KONTROL
		uart_dizi("mode-rx\n");
		#endif
			SetRx();
		break;
		case MODE_TX:
		#if DEBUG_KONTROL
		uart_dizi("mode-tx\n");
		#endif
			SetTx();
		break;
		case MODE_RX_CON:
		#if DEBUG_KONTROL
		uart_dizi("mode-rxcon\n");
		#endif
			SetRxContinuous();
		break;
		default:
		#if DEBUG_KONTROL
		uart_dizi("mode-default\n");
		#endif
			SetStdby(STDBY_RC);
		break;
	}
	}
	
}
void sx126x_baslat(){
	sx126x_config( MOD_PAR1_SF12, MOD_PAR2_BW_125, MOD_PAR3_CR_45, MOD_PAR4_LDR_ON,PACK_PAR3_HEAD_EXP,PACK_PAR5_CRC_ON,PACK_PAR6_IQ_STD,PA_DUT_22DBM,PA_HPM_22DBM,PA_DEV_1262);
	modul_begin(DIO1_IRQ,0);
	sx126x_begin(RF_FREQ, DIO2_RF_PIN);
	SetTxConfig(POWER);
	SetRxConfig(STP_TM_PRE_OFF,SYM_NUM_TM_0,RXGAIN_BOOST);
	SetCadPar(CAD_SYM_4,CAD_SF11_PEAK,CAD_DET_MIN,CAD_EXIT_STDBY,5000);
	SetStdby( STDBY_RC);
	SetRxContinuous();
}
void modul_begin(irqPin_t dio1,irqPin_t dio2){
	modul_IO();
	modul_RST();
	modul_IRQ(dio1,dio2);
}
void sx126x_config(ModPar1SF_t sf, ModPar2BW_t bw, ModPar3CR_t cr, ModPar4LDR_t ldr, PackPar3Headtype_t head, PackPar5Crc_t crc, PackPar6Iq_t loraiq, paDuty_t duty, paHpmax_t hpmax, paDevice_t device){
				
				sx1262.modparam.sprfact=sf;
				sx1262.modparam.bandwith=bw;
				sx1262.modparam.coderate=cr;
				sx1262.modparam.ldrop=ldr;
				sx1262.packparam.lorapresymb=LORA_PRE_SYM;
				sx1262.packparam.lorahead=head;
				sx1262.packparam.lorapaylen=LORA_PAY_LEN;
				sx1262.packparam.loracrc=crc;
				sx1262.packparam.loraiq=loraiq;
				sx1262.pacon.paduty=duty;
				sx1262.pacon.pahpmax=hpmax;
				sx1262.pacon.padevice=device;
				sx1262.pacon.palut=PA_PALUT;	
}
void sx126x_begin(double frequency,dio2Pin_t pinstate){
	spi_basla();
	if (TCXO==1){
		SetStdby(STDBY_RC);
		SetDio3Ctrl( TCXO_2_2V,100);//milisaniye
		Calibrate(CALIBRATE_ALL);
		ClearDevErr();
	}	
	//GetStatus();
	SetStdby(STDBY_RC);
	SetRegMode(REG_DC);
	SetFreq(frequency);	
	SetPackType();	
	SetModPar(&sx1262.modparam);
	SetPackPar(&sx1262.packparam);
	SetBufBaseAdr(txBufferAdd,rxBufferAdd);	
	SetDio2Ctrl(pinstate);
	SetIrqPar(IRQ_NONE,IRQ_NONE,IRQ_NONE,IRQ_NONE);	
	SetRxTxFallBack(RXTX_BACK_RC);	 	
}
void modul_IO(){// giriþ çýkýþ ayarlama
	SX_DDR|=(1<<RST);
	SX_DDR&=~(1<<BUSSY);
	SX_PORT|=(1<<RST);	
	RX_DDR|=(1<<RX_PIN);
}
void modul_IRQ(irqPin_t dio1,irqPin_t dio2){
	if (dio1!=0){
		IRQ_DDR&=~(1<<dio1);		
		PCICR|=(1<<PCIE2);
		PCMSK2|=(1<<dio1);
	}
	if (dio2!=0){
		IRQ_DDR&=~(1<<dio2);
		PCICR|=(1<<PCIE2);
		PCMSK2|=(1<<dio2);
	}	
	/*if (dio1==PORTD2){
		DDRD&=~(1<<PORTD2);//pd2 giriþ yapýldý
		//PORTD|=(1<<PORTD2);//pd2 dahili pull-up		
		EICRA|=(1<<ISC01)|(1<<ISC00);//pd2 yükselen kenar
		EIMSK|=(1<<INT0);//pd2
	}
	if (dio2==PORTD3){
		DDRD&=~(1<<PORTD3);//pd3 giriþ yapýldý
		//PORTD|=(1<<PORTD3);//pd3 dahili pull-up		
		EICRA|=(1<<ISC11)|(1<<ISC10);//pd3 yükselen kenar
		EIMSK|=(1<<INT1);//pd3
	}*/
	sei();	//tüm kesmeler açýldý*/
}
void modul_RST(){//modul reset
	SX_PORT&=~(1<<RST);
	_delay_us(200);
	SX_PORT|=(1<<RST);
	_delay_ms(1000);
}
void CMD_write(sxCmd_t _cmd, uint8_t *_buf, uint8_t _size){
	sx126x_bussy();
	SS_LOW;
	spi_data((uint8_t)_cmd);
	for (uint16_t i=0;i<_size;i++){
		spi_data(_buf[i]);
	}
	SS_HIGH;
}
void CMD_read(sxCmd_t _cmd, uint8_t *_buf, uint8_t _size){
	sx126x_bussy();
	SS_LOW;
	spi_data((uint8_t)_cmd);
	spi_data(NOP);
	for (uint16_t i=0;i<_size;i++){
		_buf[i]=spi_data(NOP);
	}
	SS_HIGH;
}
void REG_write(uint16_t _adr, uint8_t *_buf, uint8_t _size){
	sx126x_bussy();
	SS_LOW;
	spi_data(WR_REG);
	spi_data(_adr>>8)&0xFF;
	spi_data(_adr&0xFF);
	for (uint16_t i=0;i<_size;i++){
		spi_data(_buf[i]);
	}
	SS_HIGH;
}
void REG_read(uint16_t _adr, uint8_t *_buf, uint8_t _size){
	sx126x_bussy();
	SS_LOW;
	spi_data(RD_REG);
	spi_data(_adr>>8)&0xFF;
	spi_data(_adr&0xFF);
	spi_data(NOP);
	for (uint16_t i=0;i<_size;i++){
		_buf[i]=spi_data(NOP);
	}
	SS_HIGH;
}
void BUF_write(uint8_t _offset){
	sx126x_bussy();
	SS_LOW;
	spi_data(WR_BUFF);
	spi_data(_offset);
	do {
		sx_tx_son=(sx_tx_son+1)&sx126x_Tx_Mask;
		spi_data(sx_tx_ring[sx_tx_son]);
	} while (!(sx_tx_son==sx_tx_bas));
	SS_HIGH;
	ackerr=0;
}
void BUF_read(uint8_t _offset, uint16_t _size){
	sx126x_bussy();
	SS_LOW;
	spi_data(RD_BUFF);
	spi_data(_offset);
	spi_data(NOP);
	for (uint16_t i=0;i<_size;i++){
		sx_rx_bas=(sx_rx_bas+1)&sx126x_Rx_Mask;
		sx_rx_ring[sx_rx_bas]=spi_data(NOP);						
	}	
	SS_HIGH;
	//rxstat=MSG_READ;

}
void sx126x_bussy(){
	while (SX_PIN&(1<<BUSSY));
}
void SetRegMode(regulatMode_t regulator){
	CMD_write(SET_REGULATOR,&regulator,1);
}
void Calibrate(uint8_t calibrate){	
	CMD_write(SET_CALIB,&calibrate,1);
}
void GetStatus(){//durum kontrol
	uint8_t stat=0;
	CMD_read(GET_STAT,&stat,1);
	sx1262.status.Value=stat;
}
chipModeStatus_t sxChipMode(){
	GetStatus();
	sxModeStat=sx1262.status.Fields.chipStatus;
	return sxModeStat;
}
cmdStatus_t sxCmdStatus(){
	GetStatus();
	sxCmdStat=sx1262.status.Fields.cmdStatus;
	return sxCmdStat;
}
void SetStdby(stdbyMode_t stdby){
	CMD_write(SET_STBY,&stdby,1);
	if (stdby==STDBY_XOSC){		
		sxModeStat=MODE_STDBY_XOSC;
	}else{
		sxModeStat=MODE_STDBY_RC;
	}
}
void SetPaCon(pAconfig_t *paconfig, uint8_t ocp){
	uint8_t buf[4];
	buf[0]=paconfig->paduty;
	buf[1]=paconfig->pahpmax;
	buf[2]=paconfig->padevice;
	buf[3]=paconfig->palut;
	CMD_write(SET_PA_CON,buf,4);
	REG_write(OCP_REG,&ocp,1);	
}
void SetDio2Ctrl(dio2Pin_t pinstate){
	CMD_write(SET_DIO2_RF,&pinstate,1);	
}
void SetDio3Ctrl(TcxoVolt_t tcxovolt, uint8_t _delay){//datasheet 13.3.6 _delay = Delay(23:0)*15.625 ?s
	uint8_t buf[4];
	uint32_t Delay=(_delay<<6);
	
	buf[0]=tcxovolt;
	buf[1]=(Delay>>16)&0xFF;
	buf[2]=(Delay>>8)&0xFF;
	buf[3]=Delay&0xFF;
	CMD_write(SET_DIO3_TXCO,buf,4);
}
void SetIrqPar(irq_mask_t mask, irq_mask_t dio1, irq_mask_t dio2, irq_mask_t dio3){
	uint8_t buf[8];
	buf[0]=((mask>>8)&0xFF);
	buf[1]=((mask&0xFF));
	buf[2]=((dio1>>8)&0xFF);
	buf[3]=((dio1&0xFF));
	buf[4]=((dio2>>8)&0xFF);
	buf[5]=((dio2&0xFF));
	buf[6]=((dio3>>8)&0xFF);
	buf[7]=((dio3&0xFF));
	CMD_write(SET_IRQ_PAR,buf,8);
}
void ClearIrq(irq_mask_t clearirq ){
	uint8_t buf[2];
	buf[0]=(clearirq>>8)&0xFF;
	buf[1]=clearirq&0xFF;
	CMD_write(CLE_IRQ_STAT,buf,2);
}
void CalibImg(double freq){
	uint8_t calFreq[2];
	if( (uint16_t)freq > 900 ){
		calFreq[0] = 0xE1;
		calFreq[1] = 0xE9;
	}
	else if( (uint16_t)freq > 850 ){
		calFreq[0] = 0xD7;
		calFreq[1] = 0xD8;
	}
	else if( (uint16_t)freq > 770 ){
		calFreq[0] = 0xC1;
		calFreq[1] = 0xC5;
	}
	else if( (uint16_t)freq > 460 ){
		calFreq[0] = 0x75;
		calFreq[1] = 0x81;
	}
	else if( (uint16_t)freq > 425 ){
		calFreq[0] = 0x6B;
		calFreq[1] = 0x6F;
	}	
	CMD_write(SET_IMAGE_CAL,calFreq,2);
}
void SetFreq(double frequency){
	uint8_t buf[4];
	uint32_t freq=0;
	CalibImg(frequency);
	uint32_t step=1;
	step <<= 20;
	freq=(uint32_t)(frequency*step);//datasheet 13.4.1 frequency=(RFfreq*Fxtal)/2^25
	buf[0]=(freq>>24)&0xFF;
	buf[1]=(freq>>16)&0xFF;
	buf[2]=(freq>>8)&0xFF;
	buf[3]=(freq&0xFF);
	CMD_write(SET_RF_FREQ,buf,4);
}
void SetPackType(){
	//lora modem 0x01
	uint8_t packtype=0x01;
	CMD_write(SET_PACK_TYP,&packtype,1);
}
void SetModPar(modparam_t *modeparam){
	uint8_t buf[4];
	buf[0]=modeparam->sprfact;
	buf[1]=modeparam->bandwith;
	buf[2]=modeparam->coderate;
	/*if ( (modeparam->sprfact == MOD_PAR1_SF11 && modeparam->bandwith == MOD_PAR2_BW_125)||(modeparam->sprfact == MOD_PAR1_SF12&& (modeparam->bandwith == MOD_PAR2_BW_250||modeparam->bandwith == MOD_PAR2_BW_125)) ){
		modeparam->ldrop = MOD_PAR4_LDR_ON;
	}else {
		modeparam->ldrop = MOD_PAR4_LDR_OFF;
	}*/	
	buf[3]=modeparam->ldrop;
	CMD_write(SET_MOD_PAR,buf,4);
}
void SetPackPar(packparam_t *packetparam){
	uint8_t buf[6];
	buf[0]=(packetparam->lorapresymb>>8)&0xFF;
	buf[1]=packetparam->lorapresymb&0xFF;
	buf[2]=packetparam->lorahead;
	buf[3]=packetparam->lorapaylen;
	buf[4]=packetparam->loracrc;
	buf[5]=packetparam->loraiq;
	CMD_write(SET_PACK_PAR,buf,6);
	
}
void SetBufBaseAdr(uint8_t txadr,uint8_t rxadr){
	uint8_t buf[2];
	buf[0]=txadr;
	buf[1]=rxadr;
	CMD_write(SET_BUFF_ADD,buf,2);
}
uint16_t GetIrqStat(){
	uint8_t buf[2];
	CMD_read(GET_IRQ_STAT,buf,2);
	return ((uint16_t)buf[0]<<8)|buf[1];
}
uint16_t GetRxBuffStat(){
	uint8_t buf[2];
	CMD_read(GET_BUFF_STAT,buf,2);//buf[0] lenght, buf[1] buffpointer 	
	return ((uint16_t)buf[0]<<8)|(uint16_t)buf[1];
}
uint32_t GetPackStat(){
	uint8_t buf[3];
	CMD_read(GET_PACK_STAT,buf,3);
	return ((uint32_t)buf[0]<<16)|((uint32_t)buf[1]<<8)|(uint32_t)buf[2];
}
int8_t Rssi(){
	uint32_t packet=GetPackStat();
	return -((packet>>16)&0xff)>>1;
}
int8_t Snr(){
	uint32_t packet=GetPackStat();
	packet=(packet>>8)&0xff;
	if (packet<128){
		return packet>>2;
	}else{
		return (packet-256)>>2;
	}
}
int8_t SigRssi(){
	uint32_t packet=GetPackStat();
	return -(packet&0xff)>>1;
}
int8_t RssiInst(){
	uint8_t buf[1];
	CMD_read(GET_RSSI,buf,1);
	return -buf[0]>>1;
}
void SetTxClamp(){
	uint8_t clamp;
	REG_read(0x08D8,&clamp,1);
	clamp|=0x1E;
	REG_write(0x08D8,&clamp,1);	
}
void SetLoraSymTime(uint8_t symbol){	
	CMD_write(SET_SYM_NUM_TM,&symbol,1);
}
void SetTxParam(RampTimes_t ramp, uint8_t power){
	uint8_t buf[2];
	buf[0]=power;
	buf[1]=(uint8_t)ramp;
	CMD_write(SET_TX_PAR,buf,2);
}
void SetRx(){
	RX_EN;
	SetIrqPar(IRQ_RX_DONE|IRQ_CRC_ERR|IRQ_TIMEOUT,IRQ_RX_DONE|IRQ_CRC_ERR|IRQ_TIMEOUT,IRQ_NONE,IRQ_NONE);
	uint8_t buf[3];
	uint32_t rx_timeout=((uint32_t)RXTIMEOUT<<6);//datasheet 13.1.4 rxtimeout=timeout*15.625 us -> timeout
	buf[0]=(rx_timeout>>16)&0xFF;
	buf[1]=(rx_timeout>>8)&0xFF;
	buf[2]=rx_timeout&0xFF;
	CMD_write(SET_RX,buf,3);
	sxModeStat=MODE_RX;
}
void SetRxConfig(uint8_t stptimepre, uint8_t symnumtime,uint8_t rxgain ){	
	SetStdby(STDBY_RC);
	/*SetPackType();
	SetModPar(&sx1262.modparam);
	SetPackPar(&sx1262.packparam);*/
	SetStopTimePre(stptimepre);
	SetLoraSymTime(symnumtime);
	REG_write(RXGAIN_REG,&rxgain,1);	
}
void SetRxContinuous(){
	RX_EN;
	SetIrqPar(IRQ_RX_DONE|IRQ_CRC_ERR,IRQ_RX_DONE|IRQ_CRC_ERR,IRQ_NONE,IRQ_NONE);
	uint8_t buf[3];
	buf[0]=0xFF;
	buf[1]=0xFF;
	buf[2]=0xFF;
	CMD_write(SET_RX,buf,3);
	sxModeStat=MODE_RX_CON;
}
void SetTxConfig(uint8_t power){
	SetStdby(STDBY_RC);
	SetTxParam(  RAMP_800_US,power);
	SetPaCon(&sx1262.pacon,OCP_MAX);
	SetTxClamp();
}
void SetTx(){
	RX_DIS;
	SetIrqPar(IRQ_TX_DONE|IRQ_TIMEOUT,IRQ_TX_DONE|IRQ_TIMEOUT,IRQ_NONE,IRQ_NONE);
	uint8_t buf[3];
	uint32_t tx_timeout=((uint32_t)TXTIMEOUT<<6);//datasheet 13.1.4 rxtimeout=timeout*15.625 us -> timeout
	buf[0]=(tx_timeout>>16)&0xFF;
	buf[1]=(tx_timeout>>8)&0xFF;
	buf[2]=tx_timeout&0xFF;
	CMD_write(SET_TX,buf,3);
	sxModeStat=MODE_TX; 
	//while(!(IRQ_PIN&(1<<DIO1)));
}
void SetStopTimePre(uint8_t enable){
	CMD_write(SET_STP_TM_PRE,&enable,1);
}
void SetRxTxFallBack(uint8_t rxtxback){
	CMD_write(SET_RXTX_BACK,&rxtxback,1);
}
void SetCad(){
	SetIrqPar( IRQ_CAD_DONE| IRQ_CAD_DET,IRQ_CAD_DET|IRQ_CAD_DONE,IRQ_NONE,IRQ_NONE);
	CMD_write(SET_CAD,0,0);
	sxModeStat=MODE_CAD; 
}
void SetCadPar(uint8_t cadsymb, cadDetPeak_t cadpeak, uint8_t cadmin, uint8_t cadexit, uint32_t timeout){
	uint8_t buf[7];
	buf[0]=cadsymb;
	buf[1]=cadpeak;
	buf[2]=cadmin;
	buf[3]=cadexit;
	uint32_t cadtimeout=((uint32_t)timeout<<6);//datasheet 13.4.7 timeout=cadtimeout*15.625 us -> timeout saniye
	buf[4]=(cadtimeout>>16)&0xFF;
	buf[5]=(cadtimeout>>8)&0xFF;
	buf[6]=cadtimeout&0xFF;
	CMD_write(SET_CAD_PAR,buf,7);
	sxModeStat=MODE_CAD;
}
void ClearDevErr(){
	uint8_t buf[2] = { 0x00, 0x00 };
	CMD_write( CLE_ERROR, buf, 2 );
}
uint8_t Sx_Available(){
 	if (sx_rx_son==sx_rx_bas){
		// rxstat=MSG_DONE;
		 return 0;
 	}
	 return 1;
}
void Sx_ReadRcv(){
	uint8_t buf[2];
	CMD_read(GET_BUFF_STAT,buf,2);
	BUF_read(buf[1],buf[0]);
	rxstat=MSG_DONE;
	SetBufBaseAdr(txBufferAdd,rxBufferAdd);		
}
uint8_t Sx_Read(){;
	sx_rx_son=(sx_rx_son+1)&sx126x_Rx_Mask;
	return sx_rx_ring[sx_rx_son];
}
void Sx_ReadArray(char *stri){
	uint8_t poz=0;
	do{
		stri[poz]=Sx_Read();
		poz++;
	} while (Sx_Available());
	stri[poz]='\0';
}
/*void Sx_ReadArray(char *stri,uint8_t len){
	uint8_t poz=0;
	for (uint8_t i=len-1;i>0;i--){
		if (!(sx_rx_son==sx_rx_bas)){
			stri[poz++]=Sx_Read();
			}else{
		break;}
	}
	sx_rx_son=0;
	sx_rx_bas=0;
	stri[poz]='\0';
}//*/
void Sx_Send(const char *str){
	sx_tx_bas=0;
	sx_tx_son=0;
	uint8_t size=0;
	while (*str){
		sx_tx_bas=(sx_tx_bas+1)&sx126x_Tx_Mask;
		sx_tx_ring[sx_tx_bas]=*str++;
		size++;
	}
	sx1262.packparam.lorapaylen=size;
	SetPackPar(&sx1262.packparam);
	SetBufBaseAdr(txBufferAdd,rxBufferAdd);
	BUF_write(txBufferAdd);
	SetTx();
}
void Sx_Senddata(uint8_t data){
	sx_tx_bas=0;
	sx_tx_son=0;
	sx_tx_bas=(sx_tx_bas+1)&sx126x_Tx_Mask;
	sx_tx_ring[sx_tx_bas]=data;
	sx1262.packparam.lorapaylen=1;
	SetPackPar(&sx1262.packparam);
	BUF_write(txBufferAdd);		
	SetTx();
}
