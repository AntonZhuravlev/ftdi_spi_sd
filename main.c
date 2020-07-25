#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <ftdi.h>
#include <libftdi1/ftdi.h>
#include <libusb-1.0/libusb.h>

#define max(a,b) ((a) > (b) ? a : b)

// card types

enum {
	SD_CARD_TYPE_UNKNOWN = 0,
	SD_CARD_TYPE_SD1 = 1,	/** Standard capacity V1 SD card */
	SD_CARD_TYPE_SD2 = 2,   /** Standard capacity V2 SD card */
	SD_CARD_TYPE_SDHC = 3,  /** High Capacity SD card */
};

/*
R1: 0abcdefg
     ||||||`- 1th bit (g): card is in idle state
     |||||`-- 2th bit (f): erase sequence cleared
     ||||`--- 3th bit (e): illigal command detected
     |||`---- 4th bit (d): crc check error
     ||`----- 5th bit (c): error in the sequence of erase commands
     |`------ 6th bit (b): misaligned addres used in command
     `------- 7th bit (a): command argument outside allowed range
             (8th bit is always zero)
*/

#define R1_BIT_IDLE_STATE				(1)
#define R1_BIT_ERASE_SEQUENCE_CLEARED 	(2)
#define R1_BIT_ILLIGAL_COMMAND_DETECTED (4)
#define R1_BIT_CRC_ERROR 				(8)
#define R1_BIT_SEQUENCE_ERASE_CMD_ERROR (16)
#define R1_BIT_MISALIGNED_ADDRESS 		(32)
#define R1_BIT_ARGUMENT_OUT_OF_RANGE 	(64)

// SD card commands
enum {
/** GO_IDLE_STATE - init card in spi mode if CS low */
CMD0 = 0X00,
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
CMD8 = 0X08,
/** SEND_CSD - read the Card Specific Data (CSD register) */
CMD9 = 0X09,
/** SEND_CID - read the card identification information (CID register) */
CMD10 = 0X0A,
/** SEND_STATUS - read the card status register */
CMD13 = 0X0D,
/** READ_BLOCK - read a single data block from the card */
CMD17 = 0X11,
/** WRITE_BLOCK - write a single data block to the card */
CMD24 = 0X18,
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
CMD25 = 0X19,
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
CMD32 = 0X20,
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
CMD33 = 0X21,
/** ERASE - erase all previously selected blocks */
CMD38 = 0X26,
/** APP_CMD - escape for application specific command */
CMD55 = 0X37,
/** READ_OCR - read the OCR register of a card */
CMD58 = 0X3A,
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
ACMD23 = 0X17,
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
ACMD41 = 0X29,
};

typedef struct CID {
  // byte 0
  uint8_t mid;  // Manufacturer ID
  // byte 1-2
  char oid[2];  // OEM/Application ID
  // byte 3-7
  char pnm[5];  // Product name
  // byte 8
  unsigned prv_m : 4;  // Product revision n.m
  unsigned prv_n : 4;
  // byte 9-12
  uint32_t psn;  // Product serial number
  // byte 13
  unsigned mdt_year_high : 4;  // Manufacturing date
  unsigned reserved : 4;
  // byte 14
  unsigned mdt_month : 4;
  unsigned mdt_year_low :4;
  // byte 15
  unsigned always1 : 1;
  unsigned crc : 7;
}cid_t;
//------------------------------------------------------------------------------
// CSD for version 1.00 cards
typedef struct CSDV1 {
  // byte 0
  unsigned reserved1 : 6;
  unsigned csd_ver : 2;
  // byte 1
  uint8_t taac;
  // byte 2
  uint8_t nsac;
  // byte 3
  uint8_t tran_speed;
  // byte 4
  uint8_t ccc_high;
  // byte 5
  unsigned read_bl_len : 4;
  unsigned ccc_low : 4;
  // byte 6
  unsigned c_size_high : 2;
  unsigned reserved2 : 2;
  unsigned dsr_imp : 1;
  unsigned read_blk_misalign :1;
  unsigned write_blk_misalign : 1;
  unsigned read_bl_partial : 1;
  // byte 7
  uint8_t c_size_mid;
  // byte 8
  unsigned vdd_r_curr_max : 3;
  unsigned vdd_r_curr_min : 3;
  unsigned c_size_low :2;
  // byte 9
  unsigned c_size_mult_high : 2;
  unsigned vdd_w_cur_max : 3;
  unsigned vdd_w_curr_min : 3;
  // byte 10
  unsigned sector_size_high : 6;
  unsigned erase_blk_en : 1;
  unsigned c_size_mult_low : 1;
  // byte 11
  unsigned wp_grp_size : 7;
  unsigned sector_size_low : 1;
  // byte 12
  unsigned write_bl_len_high : 2;
  unsigned r2w_factor : 3;
  unsigned reserved3 : 2;
  unsigned wp_grp_enable : 1;
  // byte 13
  unsigned reserved4 : 5;
  unsigned write_partial : 1;
  unsigned write_bl_len_low : 2;
  // byte 14
  unsigned reserved5: 2;
  unsigned file_format : 2;
  unsigned tmp_write_protect : 1;
  unsigned perm_write_protect : 1;
  unsigned copy : 1;
  unsigned file_format_grp : 1;
  // byte 15
  unsigned always1 : 1;
  unsigned crc : 7;
}csd1_t;
//------------------------------------------------------------------------------
// CSD for version 2.00 cards
typedef struct CSDV2 {
  // byte 0
  unsigned reserved1 : 6;
  unsigned csd_ver : 2;
  // byte 1
  uint8_t taac;
  // byte 2
  uint8_t nsac;
  // byte 3
  uint8_t tran_speed;
  // byte 4
  uint8_t ccc_high;
  // byte 5
  unsigned read_bl_len : 4;
  unsigned ccc_low : 4;
  // byte 6
  unsigned reserved2 : 4;
  unsigned dsr_imp : 1;
  unsigned read_blk_misalign :1;
  unsigned write_blk_misalign : 1;
  unsigned read_bl_partial : 1;
  // byte 7
  unsigned reserved3 : 2;
  unsigned c_size_high : 6;
  // byte 8
  uint8_t c_size_mid;
  // byte 9
  uint8_t c_size_low;
  // byte 10
  unsigned sector_size_high : 6;
  unsigned erase_blk_en : 1;
  unsigned reserved4 : 1;
  // byte 11
  unsigned wp_grp_size : 7;
  unsigned sector_size_low : 1;
  // byte 12
  unsigned write_bl_len_high : 2;
  unsigned r2w_factor : 3;
  unsigned reserved5 : 2;
  unsigned wp_grp_enable : 1;
  // byte 13
  unsigned reserved6 : 5;
  unsigned write_partial : 1;
  unsigned write_bl_len_low : 2;
  // byte 14
  unsigned reserved7: 2;
  unsigned file_format : 2;
  unsigned tmp_write_protect : 1;
  unsigned perm_write_protect : 1;
  unsigned copy : 1;
  unsigned file_format_grp : 1;
  // byte 15
  unsigned always1 : 1;
  unsigned crc : 7;
}csd2_t;
//------------------------------------------------------------------------------
// union of old and new style CSD register
union csd_t {
  csd1_t v1;
  csd2_t v2;
};

struct ftdi_context ftdic;
unsigned char OutputBuffer[1024]; // Buffer to hold MPSSE commands and data to be sent to FT4232H
unsigned char InputBuffer[1024];  // Buffer to hold Data unsigned chars to be read from FT4232H
unsigned int dwNumBytesToSend = 0; // Index of output buffer
unsigned int dwNumBytesSent = 0, dwNumBytesRead = 0, dwNumInputBuffer = 0;

int divisor = 64;
uint8_t cs_bits = 0x08;
uint8_t pindir = 0x0b;

/* Utility */

void dump_data(size_t len, unsigned char *data)
{
	int i = 0;
	printf("[");
	for (i = 0; i < len; i++)
	{
		printf(" 0x%02x", data[i]);
	}
	printf("]");
	return;
}

/* FTDI Utility */

static int send_buf(struct ftdi_context *ftdic, const unsigned char *buf,
		    int size)
{
	int r;
	r = ftdi_write_data(ftdic, (unsigned char *) buf, size);
	if (r < 0) {
		printf("ftdi_write_data: %d, %s\n", r,
				ftdi_get_error_string(ftdic));
		return 1;
	}
	return 0;
}

static int get_buf(struct ftdi_context *ftdic, const unsigned char *buf,
		   int size)
{
	int r;
	while (size > 0) {
		r = ftdi_read_data(ftdic, (unsigned char *) buf, size);
		if (r < 0) {
			printf("ftdi_read_data: %d, %s\n", r,
					ftdi_get_error_string(ftdic));
			return 1;
		}
		buf += r;
		size -= r;
	}
	return 0;
}

int init_ftdi()
{
    int ftStatus = 0;
    unsigned int dwCount;
	char SerialNumBuf[64];
	int bCommandEchoed;
    int i;


    ftStatus = ftdi_init(&ftdic);
	if(ftStatus < 0) {
		printf("ftdi init failed\n");
		return 0;
	}

	ftdi_set_interface(&ftdic, INTERFACE_A);

    ftStatus = ftdi_usb_open(&ftdic, 0x0403, 0x6010);

    if(ftStatus < 0) {
		printf("Error opening usb device: %s\n", ftdi_get_error_string(&ftdic));
		return 1;
	}

    ftStatus |= ftdi_usb_reset(&ftdic); 			// Reset USB device
	ftStatus |= ftdi_usb_purge_rx_buffer(&ftdic);	// purge rx buffer
	ftStatus |= ftdi_usb_purge_tx_buffer(&ftdic);	// purge tx buffer

    if (ftdi_set_latency_timer(&ftdic, 2) < 0) {
		printf("Unable to set latency timer (%s).\n", ftdi_get_error_string(&ftdic));
	}

    if (ftdi_write_data_set_chunksize(&ftdic, 270)) {
		printf("Unable to set chunk size (%s).\n", ftdi_get_error_string(&ftdic));
	}

    /* Set MPSSE mode */
	ftdi_set_bitmode(&ftdic, 0xFF, BITMODE_RESET);
	if (ftdi_set_bitmode(&ftdic, 0x00, BITMODE_MPSSE) < 0) {
		printf("Unable to set bitmode to SPI (%s).\n", ftdi_get_error_string(&ftdic));
	}

    char buf[1024];

    printf("Set clock divisor\n");
	buf[0] = TCK_DIVISOR;
	buf[1] = (divisor / 2 - 1) & 0xff;
	buf[2] = ((divisor / 2 - 1) >> 8) & 0xff;
	if (send_buf(&ftdic, buf, 3)) {
        printf("ERROR: send_buf faild\n");
        /////
	}

    /* Disconnect TDI/DO to TDO/DI for loopback. */
	printf("No loopback of TDI/DO TDO/DI\n");
	buf[0] = LOOPBACK_END;
	if (send_buf(&ftdic, buf, 1)) {
        printf("ERROR: send_buf faild\n");
	}

	printf("Set data bits\n");
	buf[0] = SET_BITS_LOW;
	buf[1] = cs_bits;
	buf[2] = pindir;
	if (send_buf(&ftdic, buf, 3)) {
        printf("ERROR: send_buf faild\n");
	}

    return 0;
}

static int set_cs(int state)
{
	unsigned char buf[3];	
	buf[0] = SET_BITS_LOW;
	if (state)
	{
		printf("SET CS#\n");
		buf[1] = ~ 0x08 & cs_bits; /* assert CS (3rd) bit only */
	}
	else
	{
		printf("RESET CS#\n");
		buf[1] = cs_bits; /* assert CS (3rd) bit only */
	}	
	buf[2] = pindir;	
	if (send_buf(&ftdic, buf, 3))
	{
		printf("send_buf failed\n");
		return -1;
	}
	return 0;
}

/* Returns 0 upon success, a negative number upon errors. */
static int ft2232_spi_send_command(unsigned int writecnt, unsigned int readcnt,
				   const unsigned char *writearr,
				   unsigned char *readarr)
{
	static unsigned char *buf = NULL;
	/* failed is special. We use bitwise ops, but it is essentially bool. */
	int i = 0, ret = 0, failed = 0;
	size_t bufsize;
	static size_t oldbufsize = 0;
	if (writecnt > 65536 || readcnt > 65536)
		return -1;
	/* buf is not used for the response from the chip. */
	bufsize = max(writecnt + 9, 260 + 9);
	/* Never shrink. realloc() calls are expensive. */
	if (!buf || bufsize > oldbufsize) {
		buf = realloc(buf, bufsize);
		if (!buf) {
			printf("Out of memory!\n");
			/* TODO: What to do with buf? */
			return -1;
		}
		oldbufsize = bufsize;
	}

	if (writecnt) {
		buf[i++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
		buf[i++] = (writecnt - 1) & 0xff;
		buf[i++] = ((writecnt - 1) >> 8) & 0xff;
		memcpy(buf + i, writearr, writecnt);
		i += writecnt;
	}

	ret = send_buf(&ftdic, buf, i);
	i = 0;
	failed |= ret;
	
	/*
	 * Optionally terminate this batch of commands with a
	 * read command, then do the fetch of the results.
	 */
	if (readcnt) {
		buf[i++] = MPSSE_DO_READ;
		buf[i++] = (readcnt - 1) & 0xff;
		buf[i++] = ((readcnt - 1) >> 8) & 0xff;
		ret = send_buf(&ftdic, buf, i);
		failed |= ret;
		/* We can't abort here, we still have to deassert CS#. */
		if (ret)
			printf("send_buf failed before read: %i\n", ret);
		i = 0;
		if (ret == 0) {
			/*
			 * FIXME: This is unreliable. There's no guarantee that
			 * we read the response directly after sending the read
			 * command. We may be scheduled out etc.
			 */
			ret = get_buf(&ftdic, readarr, readcnt);
			failed |= ret;
			/* We can't abort here either. */
			if (ret)
				printf("get_buf failed: %i\n", ret);
		}
	}
	
	return failed ? -1 : 0;
}

/* SD commands */

// send command and return error code.  Return zero for OK
int cardCommand(uint8_t cmd, uint32_t arg, uint8_t *R1, uint8_t *response, size_t response_size) 
{
	uint8_t buff_out[1024];
	uint8_t buff_in[32];
	size_t len = 0;
	uint8_t crc;
	int i;
	uint8_t *ptr = NULL;
	int ret = -1;


	/* send command */
	buff_out[len] = cmd | 0x40;
	len++;

	/* send argument */
	buff_out[len] = arg >> 24;
	len++;
	buff_out[len] = arg >> 16;
	len++;
	buff_out[len] = arg >> 8;
	len++;
	buff_out[len] = arg;
	len++;

	/* send CRC */
	switch (cmd)
	{
		case CMD0: crc = 0X95; break; /* correct crc for CMD0 with arg 0 */
		case CMD8: crc = 0X87; break; /* correct crc for CMD8 with arg 0x1AA */
		default:   crc = 0XFF; break;
	}

	buff_out[len] = crc;
	len++;

	if (0 == ft2232_spi_send_command(len, 0, buff_out, NULL))
	{
		for (i = 0; i < 255; i++)
		{
			if (0 == ft2232_spi_send_command(0, 1, NULL, buff_in))
			{
				if (buff_in[0] != 0xFF)
				{
					*R1 = buff_in[0];
					ret = 0;

					if ((response_size > 0) && (response != NULL))
					{
						ret = ft2232_spi_send_command(0, response_size, NULL, response);
					}
					break;
				}
			}
		}
	}
	else
	{
		// error
	}

	return ret;
}

int openCard()
{
	int ret = -1;
	unsigned char buff_out[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	ret = ft2232_spi_send_command(sizeof(buff_out), 0, buff_out, NULL);
	if (ret == 0)
	{
		ret = set_cs(1);
	}
	return ret;
}

int closeCard()
{
	return set_cs(0);
}

int dump_CSD(union csd_t *csd)
{
	uint32_t card_size = 0;
	printf("csd_ver: %i\n", csd->v1.csd_ver);
	if (csd->v1.csd_ver == 0) 
	{
		uint8_t read_bl_len = csd->v1.read_bl_len;

		uint16_t c_size = (csd->v1.c_size_high << 10)
						| (csd->v1.c_size_mid << 2) | csd->v1.c_size_low;

		uint8_t c_size_mult = (csd->v1.c_size_mult_high << 1)
							| csd->v1.c_size_mult_low;

		card_size = (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
	} 
	else if (csd->v2.csd_ver == 1) 
	{
		uint32_t c_size = ((uint32_t)csd->v2.c_size_high << 16)
							| (csd->v2.c_size_mid << 8) | csd->v2.c_size_low;
		card_size = (c_size + 1) << 10;
	} 
	else 
	{
		printf("Error SD_CARD_ERROR_BAD_CSD!\n");
		return -1;
	}

	printf("card_size: %i\n",card_size);

	return 0;
}

int main(int argc, char *argv[]) 
{
	uint8_t buff_in[1024] = {0};
	uint8_t R1 = 0xff;
	int card_type = SD_CARD_TYPE_UNKNOWN;

	printf("\n=== init ===\n");

	init_ftdi();
	openCard();

	printf("\n=== write/read command ===\n");

	printf("\n+CMD0\n");
	printf("-CMD0 %i\n", cardCommand(CMD0, 0x00000000, &R1, NULL, 0));
	printf(" R1: 0x%02x\n", R1);
	if (!(R1 & R1_BIT_IDLE_STATE))
	{
		printf(" ERROR!!!\n");
	}

	printf("\n+CMD8\n");
	printf("CMD8 %i\n", cardCommand(CMD8, 0x000001AA, &R1, buff_in, 4));
	printf(" R1: 0x%02x\n", R1);
	if (R1 & R1_BIT_ILLIGAL_COMMAND_DETECTED)
	{
		printf(" SD_CARD_TYPE_SD1\n");
		card_type = SD_CARD_TYPE_SD1;
	}
	else
	{
		printf(" SD_CARD_TYPE_SD2\n");
		card_type = SD_CARD_TYPE_SD2;
	}
	printf(" DATA: ");
	dump_data(4, buff_in);
	printf("\n");

	// initialize card and send host supports SDHC if SD2
	uint32_t arg = (card_type == SD_CARD_TYPE_SD2) ? 0X40000000 : 0;

	/* wait for READY */ 
	do {
		printf("CMD55 %i\n", cardCommand(CMD55, arg, &R1, NULL, 0));
		printf(" R1: 0x%02x\n", R1);

		printf("ACMD41 %i\n", cardCommand(ACMD41, 0, &R1, NULL, 0));
		printf(" R1: 0x%02x\n", R1);
	} while (R1 != 0x00);

	// if SD2 read OCR register to check for SDHC card
	if (card_type == SD_CARD_TYPE_SD2)
	{
		printf("CMD58 %i\n", cardCommand(CMD58, 0, &R1, buff_in, 4));
		printf(" R1: 0x%02x\n", R1);

		printf(" DATA: ");
		dump_data(4, buff_in);
		printf("\n");

		if ((buff_in[0] & 0XC0) == 0XC0)
		{
			card_type = SD_CARD_TYPE_SDHC;
		}
	}

	printf(" Card Type: %s\n", (card_type == SD_CARD_TYPE_SDHC) ? "SD_CARD_TYPE_SDHC"
																: ((card_type == SD_CARD_TYPE_SD2) ? "SD_CARD_TYPE_SD2":"SD_CARD_TYPE_SD1"));

	printf("csd_t size: %i\n", sizeof(union csd_t));
	printf("csd1_t size: %i\n", sizeof(csd1_t));
	printf("csd2_t size: %i\n", sizeof(csd2_t));

	union csd_t csd_data;
	printf("\n+CMD9 (read CSD)\n");
	//printf("CMD9 %i\n", cardCommand(CMD9, 0x00000000, &R1, (uint8_t *)(&csd_data), sizeof(csd_data)));
	printf("CMD9 %i\n", cardCommand(CMD9, 0x00000000, &R1, buff_in, sizeof(csd_data)+5));
	printf(" R1: 0x%02x\n", R1);
	printf(" DATA: ");
	dump_data(sizeof(csd_data)+5, buff_in);
	printf("\n");

	if (buff_in[0] & 0x80)
	{
		if (buff_in[1] & 0x80)
		{
			dump_CSD((union csd_t *)(buff_in+2));
		}
		else
		{
			dump_CSD((union csd_t *)(buff_in+1));
		}
	}
	else
	{
		dump_CSD((union csd_t *)(buff_in));
	}
	
	/*
	printf("\n+CMD16 (read/write block size)\n");
	printf("CMD16 %i\n", cardCommand(16, 512, &R1, NULL, 0));
	printf(" R1: 0x%02x\n", R1);
	*/

	printf("\n+CMD17 (0) (read block)\n");
	printf("CMD17 %i\n", cardCommand(CMD17, 0, &R1, buff_in, 512+2+5));
	printf(" R1: 0x%02x\n", R1);
	printf(" DATA: ");
	dump_data(512+2+5, buff_in);
	printf("\n");

	printf("CMD17 (1) %i\n", cardCommand(CMD17, 1, &R1, buff_in, 512+2+5));
	printf(" R1: 0x%02x\n", R1);
	printf(" DATA: ");
	dump_data(512+2+5, buff_in);
	printf("\n");

	printf("CMD17 (512) %i\n", cardCommand(CMD17, 512, &R1, buff_in, 512+2+5));
	printf(" R1: 0x%02x\n", R1);
	printf(" DATA: ");
	dump_data(512+2+5, buff_in);
	printf("\n");

	closeCard();
	ftdi_usb_close(&ftdic);
    ftdi_deinit(&ftdic);
	return 0;
}