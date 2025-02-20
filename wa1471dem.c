#ifdef WA1471
#include "wa1471.h"
#include "math.h"
#define DEM_MAS_SIZE	8


dem_packet_st dem_mas[DEM_MAS_SIZE];

//dem_packet_info_st dem_info_mas[DEM_MAS_SIZE];

uint8_t	dem_mess_received;

struct scheduler_desc dem_processMessages_desc;

dem_packet_st tmp_dem_mas[DEM_MAS_SIZE];
//dem_packet_info_st tmp_dem_info_mas[DEM_MAS_SIZE];
uint8_t	tmp_dem_mess_received;

dem_bitrate_s current_rx_phy = DBPSK_UNDEFINED;

_Bool dem_rx_enabled = 0;

//uint8_t current_hop_table[8] = {DEM_MINUS90000,DEM_MINUS40000,DEM_PLUS40000,DEM_PLUS15000,DEM_MINUS15000,DEM_MINUS40000,DEM_MINUS65000,DEM_MINUS90000};

#define DEM_FREQ_OFFSET         50000

//const int32_t DEM_FREQ_OFFSETS[8] = {90000,65000,40000,15000,-15000,-40000,-65000,-90000};

struct scheduler_desc dem_update_noise_desc;
float dem_noise = -150;
static void wa1471dem_update_noise(struct scheduler_desc *desc);

uint32_t last_rx_freq = 0;

#ifdef DEM_CALC_SPECTRUM
uint32_t dem_spectrum_mas[64];
#endif

        uint8_t buf[2];
void wa1471dem_init(uint16_t dem_mask, uint32_t preambule)
{

	uint8_t NB_FI_RX_CRC_POLY[4] = {0xb7, 0x1d, 0xc1, 0x04};
        
        if(dem_mask == 0) dem_mask = DEM_50BPS_OFFSET + DEM_400BPS_OFFSET + DEM_3200BPS_OFFSET + DEM_25600BPS_OFFSET;
        
        for(uint16_t dem_offset = DEM_50BPS_OFFSET; dem_offset != 0; dem_offset <<= 1)
        {
          if(dem_mask&dem_offset == 0) continue;
          
          wa1471_spi_write8(dem_offset + DEM_CONTROL, DEM_CONTROL_RESET);  
          wa1471dem_set_threshold(dem_offset, 800); //1024
          wa1471_spi_read(dem_offset+ DEM_DET_TRESHOLD, buf, 2);
          wa1471dem_set_alpha(dem_offset, 128, 5);
          wa1471_spi_write8(0x2080, 21);
          wa1471dem_set_crc_poly(dem_offset, NB_FI_RX_CRC_POLY);
          if(preambule) wa1471dem_set_preambule(dem_offset, (uint8_t *)&preambule);
          wa1471_spi_write8(dem_offset+DEM_FFT_MSB, 0x80 + 23); //?
          wa1471_spi_write8(dem_offset+DEM_CONTROL, 0);
          wa1471dem_update_noise(0);
        }
        
}

void wa1471dem_reset(uint16_t dem_mask)
{
        if(dem_mask == 0) dem_mask = DEM_50BPS_OFFSET + DEM_400BPS_OFFSET + DEM_3200BPS_OFFSET + DEM_25600BPS_OFFSET;
	
        for(uint16_t dem_offset = DEM_50BPS_OFFSET; dem_offset != 0; dem_offset <<= 1)
        {
          if(dem_mask&dem_offset == 0) continue;
          wa1471_spi_write8(dem_offset + DEM_CONTROL, DEM_CONTROL_RESET);
          wa1471_spi_write8(dem_offset + DEM_CONTROL, 0);
        }

}

void wa1471dem_enable(uint16_t dem_mask)
{
   for(uint16_t dem_offset = DEM_50BPS_OFFSET; dem_offset != 0; dem_offset <<= 1)
   {
      if(dem_mask&dem_offset != 0) wa1471_spi_write8(dem_offset + DEM_CONTROL, 0);
      else wa1471_spi_write8(dem_offset + DEM_CONTROL, DEM_CONTROL_RESET);
   }
}


static uint8_t wa1471dem_get_bitrate_gain(dem_bitrate_s bitrate)
{
	switch(bitrate)
	{
	case DBPSK_50_PROT_D:
		return 0;
	case DBPSK_400_PROT_D:
		return 18;
	case DBPSK_3200_PROT_D:
		return 56;
	case DBPSK_25600_PROT_D:
		return 112;
	default:
		return 1;
	}
}

#define DEM_LOGOFFSET  208

static int16_t wa1471dem_get_rssi_logoffset()
{
	//return DEM_LOGOFFSET + rfe_rx_total_vga_gain - wa1471dem_get_bitrate_gain(current_rx_phy);
  return 200 - wa1471dem_get_bitrate_gain(current_rx_phy);
}

static void  wa1471dem_process_messages(struct scheduler_desc *desc)
{
	wa1471_hal->__wa1471_disable_pin_irq();

	tmp_dem_mess_received = dem_mess_received;
	memcpy(tmp_dem_mas, dem_mas, sizeof(tmp_dem_mas));
	dem_mess_received = 0;
	memset(dem_mas, 0 , sizeof(dem_mas));
        dem_packet_info_st info;

	wa1471_hal->__wa1471_enable_pin_irq();

	wa1471dem_update_noise(0);

	if(wa1471_hal->__wa1471_data_received)
	{
		for(uint8_t i = 0; i != tmp_dem_mess_received; i++)
		{
#ifdef WA1471_LOG
			sprintf(wa1471_log_string, "%05u: PLD: ", (uint16_t)(wa1471_scheduler->__scheduler_curr_time()&0xffff));
			for(uint8_t k = 0; k != 8; k++)
				sprintf(wa1471_log_string + strlen(wa1471_log_string), "%02X", tmp_dem_mas[i].packet.protd.payload[k]);
			sprintf(wa1471_log_string + strlen(wa1471_log_string), " IT crypto=%3d FREQ=%2d", tmp_dem_mas[i].packet.protd.iter, tmp_dem_mas[i].freq&0x3f);
#endif
                        uint64_t rssi64 = tmp_dem_mas[i].rssi_39_8;
			rssi64 <<= 8;
			rssi64 += tmp_dem_mas[i].rssi_7_0;
			float rssi = log10f(rssi64)*20 - 48 - wa1471dem_get_rssi_logoffset();
			info.rssi = (int16_t)rssi;
			float snr = rssi - dem_noise;
			if(snr < 0) snr = 0;
			info.snr = (uint8_t)snr;
            info.freq = last_rx_freq;
            info.bitrate = current_rx_phy;
            switch(current_rx_phy)
			{
                case DBPSK_50_PROT_D:
                     if(tmp_dem_mas[i].freq < 32)
                        info.freq += tmp_dem_mas[i].freq*50;
                    else info.freq -= tmp_dem_mas[i].freq*50;
                    break;
                case DBPSK_400_PROT_D:
                    if(tmp_dem_mas[i].freq < 4)
                        info.freq += tmp_dem_mas[i].freq*400;
                    else info.freq -= tmp_dem_mas[i].freq*400;
                    break;
                 case DBPSK_3200_PROT_D:
                    if(tmp_dem_mas[i].freq < 4)
                        info.freq += tmp_dem_mas[i].freq*3200;
                    else info.freq -= tmp_dem_mas[i].freq*3200;
                    break;
                 case DBPSK_25600_PROT_D:
                    if(tmp_dem_mas[i].freq < 2)
                        info.freq += tmp_dem_mas[i].freq*25600;
                    else info.freq -= tmp_dem_mas[i].freq*25600;
                    break;
            }


#ifdef WA1471_LOG
            float dsnr = log10f(((float)rssi64)/tmp_dem_mas[i].noise/4)*20;
			sprintf(wa1471_log_string + strlen(wa1471_log_string), " RSSI=%ld", tmp_dem_mas[i].rssi);
			sprintf(wa1471_log_string + strlen(wa1471_log_string), " LRSSI=%f", rssi);
			sprintf(wa1471_log_string + strlen(wa1471_log_string), " SNR=%f", rssi - dem_noise);
			sprintf(wa1471_log_string + strlen(wa1471_log_string), " DSNR=%f", dsnr);

			switch(current_rx_phy)
			{
			case DBPSK_50_PROT_D:
				sprintf(wa1471_log_string + strlen(wa1471_log_string), " 50BPS");
				break;
			case DBPSK_400_PROT_D:
				sprintf(wa1471_log_string + strlen(wa1471_log_string), " 400BPS");
				break;
			case DBPSK_3200_PROT_D:
				sprintf(wa1471_log_string + strlen(wa1471_log_string), " 3200BPS");
				break;
			case DBPSK_25600_PROT_D:
				sprintf(wa1471_log_string + strlen(wa1471_log_string), " 25600BPS");
				break;
			}
			wa1471_hal->__wa1471_log_send_str(wa1471_log_string);
#endif
			wa1471_hal->__wa1471_data_received((uint8_t*)&tmp_dem_mas[i].packet, (uint8_t*)&info);
		}
	}
}

void wa1471dem_isr(void)
{
	uint8_t status;

	wa1471_spi_read(DEM_50BPS_OFFSET + DEM_STATUS, &status, 1);

        
	if(!(status))
		return;
	dem_packet_st new_packet;

	do
	{
               uint16_t dem_offset;
               
               for(dem_offset = DEM_50BPS_OFFSET; dem_offset != 0; dem_offset <<= 1){
               
                if(dem_offset&(status<<8)) break;
               }
               
               if(!dem_offset) break;
               
		wa1471_spi_read(dem_offset + DEM_RECEIVED_MES_BUF, (uint8_t*)(&new_packet), sizeof(dem_packet_st));

		wa1471_spi_write8(dem_offset + DEM_RECEIVED_MES_BUF, 0);


		dem_mas[dem_mess_received] = new_packet;


		if(dem_mess_received == (DEM_MAS_SIZE - 1))
			return;

		wa1471_scheduler->__scheduler_add_task(&dem_processMessages_desc,  wa1471dem_process_messages, RELATIVE, (current_rx_phy == DBPSK_25600_PROT_D)?MILLISECONDS(1):MILLISECONDS(20));
                wa1471_scheduler->__scheduler_add_task(&dem_update_noise_desc,  wa1471dem_update_noise, RELATIVE, MILLISECONDS(DEM_NOISE_TICK));
		uint8_t i;
		for(i = 0; i < dem_mess_received; i++)
		{
			if(dem_mas[i].packet.protd.flags == dem_mas[dem_mess_received].packet.protd.flags)
				break;
		}

		if(i == dem_mess_received)
		{
			if(++dem_mess_received > 1);
		}
		else
		{
			if((dem_mas[i].rssi_39_8 < new_packet.rssi_39_8))
			{
				dem_mas[i] = dem_mas[dem_mess_received];
			}
			else if((dem_mas[i].rssi_39_8 == new_packet.rssi_39_8) && (dem_mas[i].rssi_7_0 < new_packet.rssi_7_0))
			{
				dem_mas[i] = dem_mas[dem_mess_received];
			}
		}
                if(!wa1471_hal->__wa1471_get_irq_pin_state()) break;
                wa1471_spi_read(DEM_50BPS_OFFSET + DEM_STATUS, &status, 1);
                if(!status) break;
	}while (1);
}

int16_t wa1471dem_get_bitrate_sensitivity(dem_bitrate_s bitrate)
{
	switch(bitrate)
	{
	case DBPSK_50_PROT_D:
	default:
		return -148;
	case DBPSK_400_PROT_D:
		return -139;
	case DBPSK_3200_PROT_D:
		return -130;
	case DBPSK_25600_PROT_D:
		return -118;
	}
}

static int8_t wa1471dem_get_sensitivity_diff(dem_bitrate_s bitrate_1, dem_bitrate_s bitrate_2)
{
	return wa1471dem_get_bitrate_sensitivity(bitrate_1) - wa1471dem_get_bitrate_sensitivity(bitrate_2);
}



void wa1471dem_set_bitrate(dem_bitrate_s bitrate)
{
               
	switch(bitrate)
	{
	case DBPSK_50_PROT_D:
		wa1471dem_reset(DEM_50BPS_OFFSET);
		wa1471rfe_set_rx_gain(RFE_DEFAULT_VGA_GAIN);
	break;
	case DBPSK_400_PROT_D:
		wa1471dem_reset(DEM_400BPS_OFFSET);
		wa1471rfe_set_rx_gain(RFE_DEFAULT_VGA_GAIN);
	break;
	case DBPSK_3200_PROT_D:
		wa1471dem_reset(DEM_3200BPS_OFFSET);
		wa1471rfe_set_rx_gain(RFE_DEFAULT_VGA_GAIN + 6);
	break;
	case DBPSK_25600_PROT_D:
		wa1471dem_reset(DEM_25600BPS_OFFSET);
		wa1471rfe_set_rx_gain(RFE_DEFAULT_VGA_GAIN + 24);
	break;
	}
              
	dem_noise -= wa1471dem_get_sensitivity_diff(current_rx_phy, bitrate);
	
        if(current_rx_phy != bitrate) wa1471dem_update_noise(0); //reinit noise engine
	current_rx_phy = bitrate;
	     
        #ifdef WA1471_LOG
	sprintf(wa1471_log_string, "%05u: dem_set_bitrate to %d", ((uint16_t)(wa1471_scheduler->__scheduler_curr_time()&0xffff)), bitrate);
	wa1471_hal->__wa1471_log_send_str(wa1471_log_string);
        #endif
}




void wa1471dem_set_alpha(uint16_t dem_offset, uint8_t noise_start_bit, uint8_t shift)
{
	wa1471_spi_write8(dem_offset+DEM_NOSE_START_BIT, noise_start_bit);
	wa1471_spi_write8(dem_offset+DEM_ALPHA_SHIFT, shift);
}

void wa1471dem_set_threshold(uint16_t dem_offset, uint16_t SOFT_DETECT_THR)
{
	uint8_t  SOFT_DETECT_THR_ARR[2] = {SOFT_DETECT_THR & 255, SOFT_DETECT_THR >> 8};
	wa1471_spi_write(dem_offset + DEM_DET_TRESHOLD, SOFT_DETECT_THR_ARR, 2);
}


void wa1471dem_set_crc_poly(uint16_t dem_offset, uint8_t* crc)
{
	wa1471_spi_write(dem_offset+DEM_CRC_POLY, crc, 4);
}


void wa1471dem_set_preambule(uint16_t dem_offset, uint8_t* preambule)
{
	wa1471_spi_write(dem_offset+DEM_PREAMBLE_ID, preambule, 4);
}

void setRxFreq(uint64_t freq);

void wa1471dem_set_freq(uint32_t freq)
{
        //wa1471rfe_set_freq(freq + DEM_FREQ_OFFSET);
    
        setRxFreq(freq + DEM_FREQ_OFFSET);
        
        last_rx_freq = freq;

        #ifdef WA1471_LOG
	sprintf(wa1471_log_string, "%05u: dem_set_freq to %ld", ((uint16_t)(wa1471_scheduler->__scheduler_curr_time()&0xffff)), freq);
	wa1471_hal->__wa1471_log_send_str(wa1471_log_string);
        #endif
}

//#include "stm32_init.h"
static uint32_t wa1471dem_get_rssi_int(_Bool aver_or_max)
{
	uint16_t data[64];
        //uint8_t data[128];
	uint8_t size;
	uint32_t rssi = 0;
	uint32_t max = 0;
        uint16_t dem_offset;
	switch(current_rx_phy)
	{
	case DBPSK_50_PROT_D:
		size = 64;
                dem_offset = DEM_50BPS_OFFSET;
		break;
	case DBPSK_400_PROT_D:
		size = 8;
                dem_offset = DEM_400BPS_OFFSET;
		break;
	case DBPSK_3200_PROT_D:
		size = 8;
                 dem_offset = DEM_3200BPS_OFFSET;
		break;
	case DBPSK_25600_PROT_D:
		size = 4;
                 dem_offset = DEM_25600BPS_OFFSET;
		break;
	}

        while(!wa1471_spi_read8(dem_offset+ DEM_CONTROL)&DEM_CONTROL_FFT_READY);     

        wa1471_spi_read(dem_offset+ DEM_FFT_READ_BUF, (uint8_t*)(&data[0]), 2*size);

        wa1471_spi_write8(dem_offset + DEM_FFT_READ_BUF + 100, 0);
               
        uint16_t tmp;
        
	for(int i = 0; i != size; i++)
	{
          tmp = data[i];
          if(i%2 == 0) {
            data[i] = data[i+1];
            data[i+1] = tmp;
          }

         
        uint32_t fft_mag = (uint32_t)(powf(2, ( 8 + (float)((((uint8_t*)(&data[0]))[2*i]) + 256 * (float)(((uint8_t*)(&data[0]))[2*i+1]))/256)));
        rssi += fft_mag;
		if(fft_mag > max)
			max = fft_mag;
#ifdef DEM_CALC_SPECTRUM
		dem_spectrum_mas[i] = fft_mag;//((dem_spectrum_mas[i]*7 + fft_mag + 1)>>3);
#endif
	}
          //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	return (aver_or_max?rssi/size:max);
}


float wa1471dem_get_rssi()
{
	return 20*log10f(wa1471dem_get_rssi_int(0)) - wa1471dem_get_rssi_logoffset();
}

float wa1471dem_get_noise()
{
	return dem_noise;
}

#ifdef DEM_CALC_SPECTRUM
void wa1471dem_get_spectrum(uint8_t size, float* data)
{
	for(int i = 0; i != size; i++)
		data[i] = 20*log10f(dem_spectrum_mas[i]) - wa1471dem_get_rssi_logoffset();
}
#endif

uint8_t wa1471dem_get_noise_calc_duration()
{
    const uint8_t NBFI_NOISE_DINAMIC_D[4] = {20, 8, 5, 5};
    return NBFI_NOISE_DINAMIC_D[current_rx_phy - DBPSK_50_PROT_D];
}



static void wa1471dem_update_noise(struct scheduler_desc *desc)
{

	static uint32_t dem_noise_sum;
	static uint32_t dem_noise_min;
	static uint8_t dem_noise_cntr;
	static uint8_t dem_noise_min_cntr ;

	if(desc == 0) //init noise engine
	{
		dem_noise_min = 0xffffffff;
		dem_noise_cntr = 0;
		dem_noise_min_cntr = wa1471dem_get_noise_calc_duration();
		return;
	}

	if(dem_noise_cntr == DEM_NOISE_AVER)
	{
		dem_noise_cntr = 0;
		uint32_t aver = dem_noise_sum/DEM_NOISE_AVER;
		dem_noise_sum = 0;
		if(aver < dem_noise_min)
			dem_noise_min = aver;
		if(--dem_noise_min_cntr == 0)
		{
			dem_noise_min_cntr = wa1471dem_get_noise_calc_duration();
			dem_noise = dem_noise_min;
			dem_noise = log10f(dem_noise)*20 - wa1471dem_get_rssi_logoffset() + 2; //2 dB is approximate difference between minimum and average noise level
			dem_noise_min = 0xffffffff;

		}

	}
	else
	{
		dem_noise_sum += wa1471dem_get_rssi_int(1);
		dem_noise_cntr++;
	}

	if(dem_rx_enabled) wa1471_scheduler->__scheduler_add_task(desc, 0, RELATIVE, MILLISECONDS(DEM_NOISE_TICK));
}

void wa1471dem_rx_enable(_Bool en)
{
	dem_rx_enabled = en;
	if(en)
	{
		wa1471_scheduler->__scheduler_add_task(&dem_update_noise_desc,  wa1471dem_update_noise, RELATIVE, MILLISECONDS(DEM_NOISE_TICK));
	}
	else
		wa1471_scheduler->__scheduler_remove_task(&dem_update_noise_desc);
}
#endif //#ifdef WA1471