/*
 * sx126x.h
 *
 * Created: 9.01.2021 16:59:39
 *  Author: haluk
 */ 
#ifndef SX126X_H_
#define SX126X_H_



//#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "uart.h"
#include "timer0.h"
#include <util/delay.h>
//#define TELEFON_KART		
//#define POMPA_KART			


#define sx126x_Rx_Boyut 128
#define sx126x_Tx_Boyut 128
#define sx126x_Rx_Mask (sx126x_Rx_Boyut-1)
#define sx126x_Tx_Mask (sx126x_Tx_Boyut-1)

//PIN IO-IRQ ////////////////////////
#define IRQ1		PCINT2_vect
#define IRQ2		PCINT2_vect
/*
#define IRQ1		INT0_vect
#define IRQ2		INT1_vect*/



#define DIO1				PORTD6//portd pinleri seçilecek
#define DIO2				PORTD6//portd pinleri seçilecek dio2 tx_en baðlý
#define RX_PIN				PORTD3// rx rf switch
#define RST					PORTD4
#define BUSSY				PORTD5
#define IRQ_PORT			PORTD
#define IRQ_DDR				DDRD
#define IRQ_PIN				PIND
#define RX_PORT				PORTD
#define RX_DDR				DDRD
#define SX_PORT				PORTD
#define SX_DDR				DDRD
#define SX_PIN				PIND




#define RX_EN				RX_PORT|=(1<<RX_PIN)
#define RX_DIS				RX_PORT&=~(1<<RX_PIN)
#define TCXO				1// tcxo varsa 1
typedef enum {
	DIO1_IRQ			=DIO1,
	DIO2_IRQ			=DIO2,
}irqPin_t;
typedef enum {
	DIO2_IRQ_PIN		=0,
	DIO2_RF_PIN			=1,
}dio2Pin_t;
typedef enum
{
	TCXO_1_6V                          = 0x00,
	TCXO_1_7V                          = 0x01,
	TCXO_1_8V                          = 0x02,
	TCXO_2_2V                          = 0x03,
	TCXO_2_4V                          = 0x04,
	TCXO_2_7V                          = 0x05,
	TCXO_3_0V                          = 0x06,
	TCXO_3_3V                          = 0x07,
}TcxoVolt_t;
//sx1262 ayarlar
#define ACK_ON			0x01
#define ACK_OFF			0x00
#define ACK_STATE		ACK_OFF
#define ACK				0x06
#define ACK_WAIT_TIME	20000//ack bekleme ms
#define ACK_SEND_TIME	4000//ack gonderme ms
#define TXTIMEOUT		5000//ms
#define RXTIMEOUT		5000//ms
#define POWER			22//22dbm
#define RF_FREQ			433.92//433.92//433920000Hz SBT yönetmelik
#define LORA_PAY_LEN	0xFF// veri paket boyutu max
#define LORA_PRE_SYM	0x0028// 12 preamble sembol sayýsý
#define LORA_TX			0x00
#define LORA_RX			0x01
#define STP_TM_PRE_ON	0x01
#define STP_TM_PRE_OFF	0x00
#define SYM_NUM_TM_0	0x00
#define SYM_NUM_TM_2	0x02//lora sembol sayýsý
#define SYM_NUM_TM_4	0x04
#define SYM_NUM_TM_8	0x08
#define CALIBRATE_ALL	0x7F// tüm kalibrasyonlar açýk
#define RXGAIN_REG		0x08AC//rx gain 
#define RXGAIN_BOOST	0x96//hassaiyet artar güç tüketim artar
#define RXGAIN_SAVE		0x94//default gain güç tüketimi azalýr
#define OCP_REG			0x08E7//pa sonrasý yapýlýr
#define OCP_MAX			0x38//2,5mA en düþük 60 (0x18) en yüksek 140mA (0x38)
#define RXTX_BACK_RC	0x20//stdby-rc mode
#define RXTX_BACK_XOSC	0x30//stdby-xosc mode
#define PACK_TYP_LORA	0x01// lora mod
#define NOP				0x00

typedef enum {
	//SET_SLEEP			=0x84,
	SET_STBY			=0x80,//bekleme ayar
	SET_TX				=0x83,//tx mode
	SET_RX				=0x82,//rx mode
	SET_STP_TM_PRE		=0x9F,// aktif olursa preamble algýlandýðýnda timer durdurur
	SET_CAD				=0xC5,// cad mode
	SET_REGULATOR		=0x96,// regulator
	SET_CALIB			=0x89,// kalibrasyon
	SET_IMAGE_CAL		=0x98,// frekans ayarý
	SET_PA_CON			=0x95,// güç amfi ayar
	SET_RXTX_BACK		=0x93,//veri hareketi sonrasý mode seçimi
	WR_REG				=0x0D,//  opcode
	RD_REG				=0x1D,//  opcode
	WR_BUFF				=0x0E,//  opcode
	RD_BUFF				=0x1E,//  opcode
	SET_IRQ_PAR			=0x08,// kesme ve pin iþleyiþi
	GET_IRQ_STAT		=0x12,//kesme durum döndürür 3 nop
	CLE_IRQ_STAT		=0x02,//kesme bayrak silme
	SET_DIO2_RF			=0x9D,// rf switch dio 2 ye baðlý halde
	SET_DIO3_TXCO		=0x97,// tcxo dio 3 e baðlý halde
	SET_RF_FREQ			=0x86,// RF frekans ayarý rf frq= (RF_FREQ*((uint32_t)(1<<25))/32.0)
	SET_PACK_TYP		=0x8A,// paket tipi stdby_rc moodda ilk yapýlacak
	GET_PACK_TYP		=0x11,//paket tipi döndürür 2 nop	
	SET_TX_PAR			=0x8E,// tx parametreleri
	SET_MOD_PAR			=0x8B,// modülasyon parametre
	SET_PACK_PAR		=0x8C,// paket parametre
	SET_CAD_PAR			=0x88,//cad ayarlarý	
	SET_BUFF_ADD		=0x8F,//buffer adres
	SET_SYM_NUM_TM		=0xA0,//timer kapatmak için gereken sembol numarasý
	GET_STAT			=0xC0,// durum 1 nop
	CLE_ERROR			=0x07,// hata silme
	GET_BUFF_STAT		=0x13,//status, veri boyutu ve rx pointer 3 nop	
	GET_PACK_STAT		=0x14,//paket durumu döndürür 4 nop
	GET_RSSI			=0x15,//sinyal gücü 2 nop
	
}sxCmd_t;

typedef enum {
	RAMP_10_US		=0x00,
	RAMP_20_US		=0x01,
	RAMP_40_US		=0x02,//40us
	RAMP_80_US		=0x03,//80us
	RAMP_200_US		=0x04,//200us
	RAMP_800_US		=0x05,
	RAMP_1700_US		=0x06,
	RAMP_3400_US		=0x07,
}RampTimes_t;

typedef enum {
	ACK_NOT_WAIT			=0x00,// ack bekleme
	ACK_WAIT				=0x01,// ack bekle	
	ACK_SEND				=0x02,// ack gönder
	ACK_SENDING				=0x03,// ack gönderiliyor
}ackStatus_t;
typedef enum {
	TX_AGAIN_ON				=0x00,// Tx tekrarla
	TX_AGAIN_OFF			=0x01,// Tx tekrarlama
}txStatus_t;

typedef enum {
	MSG_DONE				=0x00,// mesaj tamamlandý
	MSG_RCV					=0x01,// mesaj alýndý
	/*MSG_RDY					=0x02,// mesaj hazýr	
	MSG_READ				=0x03,// mesaj okundu	*/
}rxStatus_t;

typedef enum {
	STAT_CMD_DATA_HOST	=0x02,// veri alýndý hazýr
	STAT_CMD_TIMEOUT	=0x03,// komut zaman aþýmý
	STAT_CMD_ERROR		=0x04,// hatalý iþlem
	STAT_CMD_FAIL		=0x05,//iþlem yürütülemedi
	STAT_CMD_TX_DONE	=0x06,//veri gönderildi
}cmdStatus_t;
typedef enum {
	MODE_SLEEP			=0x00,// uyku mode
	MODE_CAD			=0x01,//CAD mode
	MODE_STDBY_RC		=0x02,//rc mode
	MODE_STDBY_XOSC		=0x03,//xosc mode
	MODE_FS				=0x04,//fs mode
	MODE_RX				=0x05,//rx mode
	MODE_TX				=0x06,//tx mode
	MODE_RX_CON			=0x07,//rx sürekli
}chipModeStatus_t;

typedef union {
	uint8_t Value;
	struct
	{   
		uint8_t reserved_0	:	1;
		uint8_t cmdStatus	:	3;
		uint8_t chipStatus	:	3;
		uint8_t reserved_7	:	1;
	}Fields;
}modulStatus_t;

typedef enum {
	IRQ_NONE			=0x0000,//kesme yok
	IRQ_TX_DONE			=0x0001,//tx bitti
	IRQ_RX_DONE			=0x0002,//rx bitti
	IRQ_PRE_DET			=0x0004,//preamble saptandý
	IRQ_SYN_WORD		=0x0008,//fsk modülasyon
	IRQ_HEAD_DET		=0x0010,//header saptandý
	IRQ_HEAD_ERR		=0x0020,//header hata
	IRQ_CRC_ERR			=0x0040,//crc hata
	IRQ_CAD_DONE		=0x0080,//cad bitti
	IRQ_CAD_DET			=0x0100,//cad saptandý
	IRQ_TIMEOUT			=0x0200,// zaman aþýmý
	IRQ_ALL				=0xFFFF,// tüm kesmeler
}irq_mask_t;

typedef enum {
	
	STDBY_RC			=0x00,//rc bekleme
	STDBY_XOSC			=0x01,//harici kristal bekleme
}stdbyMode_t;
typedef enum {
	REG_LDO				=0x00,
	REG_DC				=0x01//dc-dc regulator 	
}regulatMode_t;


// CAD config
#define CAD_SYM_2			0x01//2 sembol
#define CAD_SYM_4			0x02//4 sembol uygun
#define CAD_SYM_8			0x08//4 sembol uygun
#define CAD_DET_MIN			10//bu deðeri deðiþtirmeye gerek yok
#define CAD_EXIT_RX			0x01//cad sonrasý rx// cad_rx
#define CAD_EXIT_STDBY		0x00//cad sonrasý stdby// cad_only
// CAD timeout deðeri msaniye cinsinden girilecek.
// CAD timeout en fazla 262 saniye olabilir.

typedef enum {// bw 125kHz, cr 4/5 cad sembol 4 ve Sf degerine gore en iyi ayarlar
	CAD_SF7_PEAK		=21,
	CAD_SF8_PEAK		=22,
	CAD_SF9_PEAK		=23,
	CAD_SF10_PEAK		=24,
	CAD_SF11_PEAK		=25,
	CAD_SF12_PEAK		=28,
}cadDetPeak_t;

/// lora packet parametreleri
typedef enum{
	PACK_PAR3_HEAD_EXP	=0x00,// deðiþken boyut baþlýk açýk
	PACK_PAR3_HEAD_IMP	=0x00,// sabit boyut  baþlýk yok
}PackPar3Headtype_t;
typedef enum{
	PACK_PAR5_CRC_ON	=0x01,//crc açýk
	PACK_PAR5_CRC_OFF	=0x00,//crc kapalý
}PackPar5Crc_t;
typedef enum{
	PACK_PAR6_IQ_STD	=0x00,// standart ýq
	PACK_PAR6_IQ_INV	=0x01,// invert ýq
}PackPar6Iq_t;
typedef struct{
	uint16_t				lorapresymb;//param1-param2
	PackPar3Headtype_t		lorahead;
	uint8_t					lorapaylen;//param4
	PackPar5Crc_t			loracrc;
	PackPar6Iq_t			loraiq;		
}packparam_t;
/// lora modulation parametreleri
typedef enum {
	
	MOD_PAR1_SF5		=0x05,
	MOD_PAR1_SF6		=0x06,
	MOD_PAR1_SF7		=0x07,
	MOD_PAR1_SF8		=0x08,
	MOD_PAR1_SF9		=0x09,
	MOD_PAR1_SF10		=0x0A,
	MOD_PAR1_SF11		=0x0B,//sf11
	MOD_PAR1_SF12		=0x0C,
}ModPar1SF_t;
typedef enum {
	MOD_PAR2_BW_7		=0x00,
	MOD_PAR2_BW_10		=0x08,
	MOD_PAR2_BW_15		=0x01,
	MOD_PAR2_BW_20		=0x09,
	MOD_PAR2_BW_31		=0x02,
	MOD_PAR2_BW_41		=0x0A,
	MOD_PAR2_BW_62		=0x03,
	MOD_PAR2_BW_125		=0x04,//bw 125kHz
	MOD_PAR2_BW_250		=0x05,//bw 250kHz
	MOD_PAR2_BW_500		=0x06,
}ModPar2BW_t;
typedef enum {
	MOD_PAR3_CR_45		=0x01,//cr 4/5
	MOD_PAR3_CR_46		=0x02,
	MOD_PAR3_CR_47		=0x03,
	MOD_PAR3_CR_48		=0x04,
}ModPar3CR_t;

typedef enum {//lora parametreleri
	MOD_PAR4_LDR_ON		=0x01,
	MOD_PAR4_LDR_OFF	=0x00,//ldro kapalý
}ModPar4LDR_t;
typedef struct{
	ModPar1SF_t sprfact;
	ModPar2BW_t bandwith;
	ModPar3CR_t coderate;
	ModPar4LDR_t ldrop;	
}modparam_t;

//pa config
#define	PA_PALUT		0x01
typedef enum{
	PA_DUT_22DBM		=0x04,
	PA_DUT_20DBM		=0x03,
	PA_DUT_17DBM		=0x02,
	PA_DUT_14DBM		=0x02,
}paDuty_t;
typedef enum{
	PA_HPM_22DBM		=0x07,
	PA_HPM_20DBM		=0x05,
	PA_HPM_17DBM		=0x03,
	PA_HPM_14DBM		=0x02,
}paHpmax_t;
typedef enum{
	PA_DEV_1262			=0x00,
	PA_DEV_1261			=0x01,
}paDevice_t;

typedef struct  {
	paDuty_t		paduty;
	paHpmax_t		pahpmax;
	paDevice_t		padevice;
	uint8_t			palut;
}pAconfig_t;
typedef struct{
	packparam_t		packparam;
	modparam_t		modparam;
	pAconfig_t		pacon;
	modulStatus_t	status;
}sx126x_t;

void sx126x_baslat();
void sx126x_config(ModPar1SF_t sf, ModPar2BW_t bw, ModPar3CR_t cr, ModPar4LDR_t ldr,
					PackPar3Headtype_t head, PackPar5Crc_t crc, PackPar6Iq_t loraiq,
					paDuty_t duty, paHpmax_t hpmax, paDevice_t device);
void sx126x_begin(double frequency,dio2Pin_t pinstate);
void modul_begin(irqPin_t dio1,irqPin_t dio2);
void modul_IO();
void modul_IRQ(irqPin_t dio1,irqPin_t dio2);
void modul_RST();
void CMD_write(sxCmd_t _cmd, uint8_t *_buf, uint8_t _size);
void CMD_read(sxCmd_t _cmd, uint8_t *_buf, uint8_t _size);
void REG_write(uint16_t _adr, uint8_t *_buf, uint8_t _size);
void REG_read(uint16_t _adr, uint8_t *_buf, uint8_t _size);
void BUF_write(uint8_t _offset);
void BUF_read(uint8_t _offset, uint16_t _size);
void sx126x_bussy();
void SetRegMode(regulatMode_t regulator);
void Calibrate(uint8_t calibrate);
void GetStatus();
chipModeStatus_t sxChipMode();
cmdStatus_t sxCmdStatus();
void SetStdby(stdbyMode_t stdby);
void SetPaCon(pAconfig_t *paconfig, uint8_t ocp);
void SetDio2Ctrl(dio2Pin_t pinstate);
void SetDio3Ctrl(TcxoVolt_t tcxovolt, uint8_t _delay);
void SetIrqPar(irq_mask_t mask, irq_mask_t dio1, irq_mask_t dio2, irq_mask_t dio3);
void ClearIrq(irq_mask_t clearirq );
void CalibImg(double freq);
void SetFreq(double frequency);
void SetPackType();
void SetModPar(modparam_t *modeparam);
void SetPackPar(packparam_t *packetparam);
void SetBufBaseAdr(uint8_t txadr,uint8_t rxadr);
void SetTxClamp();
void SetLoraSymTime(uint8_t symbol);
void SetTxParam(RampTimes_t ramp, uint8_t power);
void SetCadPar(uint8_t cadsymb, cadDetPeak_t cadpeak, uint8_t cadmin, uint8_t cadexit, uint32_t timeout);
void SetCad();
void SetTxConfig(uint8_t power);
void SetRxConfig(uint8_t stptimepre, uint8_t symnumtime,uint8_t rxgain );
void SetRx();
void SetRxContinuous();
void SetTx();
void SetStopTimePre(uint8_t enable);
void SetRxTxFallBack(uint8_t rxtxback);
uint16_t GetIrqStat();
uint16_t GetRxBuffStat();
uint32_t GetPackStat();
int8_t Rssi();
int8_t Snr();
int8_t SigRssi();
int8_t RssiInst();
void ClearDevErr();
void Sx_ReadRcv();
void Sx_Send(const char *str);
void Sx_Senddata(uint8_t data);
uint8_t Sx_Read();
void Sx_ReadArray(char *stri);
void Sx_Mode();
uint8_t Sx_Available();

#endif