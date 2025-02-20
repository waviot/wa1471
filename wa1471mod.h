#ifdef WA1471
#ifndef _wa1471MOD_H
#define _wa1471MOD_H

//----------------------------------------------------------
// MODULATOR REGs
//----------------------------------------------------------
#define MOD_SPI_OFFSET					0x1000//0x38

//Write registers definitions
#define MOD_CONFIG				(MOD_SPI_OFFSET + 0x0)
#define MOD_PER0				(MOD_SPI_OFFSET + 0x1)
#define MOD_PER1				(MOD_SPI_OFFSET + 0x2)
#define MOD_PER2				(MOD_SPI_OFFSET + 0x3)
#define MOD_DATA_START			        (MOD_SPI_OFFSET + 0x4)

#define MOD_MESS_LEN0	                        (MOD_SPI_OFFSET + 0x5C) //1-640
#define MOD_MESS_LEN1	                        (MOD_SPI_OFFSET + 0x5D) 
#define MOD_MESS_LEN2	                        (MOD_SPI_OFFSET + 0x5E) 
#define MOD_MESS_LEN3	                        (MOD_SPI_OFFSET + 0x5F) 

#define MOD_CONF_TX_ABORT				0x40
#define MOD_CONF_IRQ_ON_TX_END_EN		        0x10
#define MOD_CONF_CLEAR_IRQ				0x02
#define MOD_CONF_TX_START				0x01

//Read registers definitions
#define MOD_CURR_TX_BITS_SENT	        (MOD_SPI_OFFSET + 0x0)
#define MOD_STATUS			(MOD_SPI_OFFSET + 0x1)
#define MOD_SENT_TOTAL			(MOD_SPI_OFFSET + 0x2)
#define MOD_ERR_TOTAL			(MOD_SPI_OFFSET + 0x3)
#define MOD_TX_LAST_BIT		(MOD_SPI_OFFSET + 0x5)

#define MOD_STATUS_TX_IN_PROGRESS		0x80
#define MOD_STATUS_IRQ_ON_TX_FLAG		0x10


typedef enum
{
	MOD_DBPSK_50_PROT_D	= 21,
	MOD_DBPSK_400_PROT_D	= 24,
	MOD_DBPSK_3200_PROT_D	= 26,
	MOD_DBPSK_25600_PROT_D	= 28,
	MOD_DBPSK_50_PROT_E	= 30,
	MOD_DBPSK_400_PROT_E	= 31,
	MOD_DBPSK_3200_PROT_E	= 32,
	MOD_DBPSK_25600_PROT_E	= 33,
}mod_bitrate_s;

void wa1471mod_init();
void wa1471mod_isr(void);
//void wa1471mod_set_hop_table(mod_hop_channels_t *hop_table);
uint16_t wa1471mod_phy_to_bitrate(mod_bitrate_s bitrate);
void wa1471mod_set_bitrate(mod_bitrate_s bitrate);
void wa1471mod_send(uint8_t* data, mod_bitrate_s bitrate);
void wa1471mod_set_freq(uint32_t freq);
_Bool wa1471mod_is_tx_in_progress();
#define wa1471_bpsk_pin_tx_finished wa1471_tx_finished

#endif
#endif //#ifdef WA1471