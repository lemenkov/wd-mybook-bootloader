/*******************************************************************
 *
 * File:            sata.c
 *
 * Description:     SATA controller access.
 *
 * Date:            28 October 2005
 *
 * Author:          J J Larkworthy
 *
 * Copyright:       Oxford Semiconductor Ltd, 2005
 *
 *
 * Verifies that the external FLASH image is intact before copying the 
 * executable for the LEON coprocessor into SRAM. 
 * Then the arm processor is allowed to execute the code in FLASH.
 * 
 *******************************************************************/
/*  SATA boot loader.
 * should initialise sata controller and external phy before loading first segment.
 * The first segment contains the Master Boot Record - just the length and start
 * sector in our case, and the partition table. The partition table is used
 * by the final OS to establish which partition contains what.
 *
 * From the first sector establish where the boot image lies on the disk and 
 * how bit then copy into the SRAM from the begining. 
 * 
 */

#include "types.h"
#include "oxnas.h"
#include "ata.h"
#include "sata.h"
#include "dma.h"


#define MAKE_FIELD(value, num_bits, bit_num) (((value) & ((1 << (num_bits)) - 1)) << (bit_num))

#define CFG_ATA_DATA_OFFSET 0
#define CFG_ATA_REG_OFFSET  0
#define CFG_ATA_ALT_OFFSET  0


#define printf(A) 


/* The internal SATA drive on which we should attempt to find partitions */
static volatile u32 *sata_regs_base;
static volatile u32 *sata_link_base;
static u32 wr_sata_orb1;
static u32 wr_sata_orb2;
static u32 wr_sata_orb3;
static u32 wr_sata_orb4;

static const int MAX_SRC_READ_LOOPS = 10000;	/* 0.1 second in units of 10uS */
static const int MAX_SRC_WRITE_LOOPS = 10000;	/* 0.1 second in units of 10uS */
static const int MAX_DMA_XFER_LOOPS = 10000;	/* 0.1 second in units of 10uS */
static const int MAX_DMA_ABORT_LOOPS = 10000;	/* 0.1 second in units of 10uS */
static const int IDE_SPIN_UP_TIME_OUT = 10000;	/* 0.1 second in units of 10uS */
static const int IDE_TIME_OUT = 10000;
static const int MAX_NO_ERROR_LOOPS = 10000;
static const int MAX_NOT_BUSY_LOOPS = 10000;




/* static data controlling the operation of the DMA controller in this 
 * application.
 */
static oxnas_dma_device_settings_t oxnas_sata_dma_settings;

static const oxnas_dma_device_settings_t oxnas_sata_dma_rom_settings = {
    .address_ = SATA_DATA_BASE,
    .fifo_size_ = 16,
    .dreq_ = OXNAS_DMA_DREQ_SATA,
    .read_eot_ = 0,
    .read_final_eot_ = 1,
    .write_eot_ = 0,
    .write_final_eot_ = 1,
    .bus_ = OXNAS_DMA_SIDE_A,
    .width_ = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_ = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_ = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 0
};

const oxnas_dma_device_settings_t oxnas_ram_dma_rom_settings = {
    .address_ = 0,
    .fifo_size_ = 0,
    .dreq_ = OXNAS_DMA_DREQ_MEMORY,
    .read_eot_ = 1,
    .read_final_eot_ = 1,
    .write_eot_ = 1,
    .write_final_eot_ = 1,
    .bus_ = OXNAS_DMA_SIDE_B,
    .width_ = OXNAS_DMA_TRANSFER_WIDTH_32BITS,
    .transfer_mode_ = OXNAS_DMA_TRANSFER_MODE_BURST,
    .address_mode_ = OXNAS_DMA_MODE_FIXED,
    .address_really_fixed_ = 1
};

unsigned char oxnas_sata_inb(int device, int port);

#define PHY_RESET_FAIL               1
#define PHY_2ND_RESET_FAIL           2
#define PROG_READ_FAILURE            4  
#define SECTOR_0_READ_FAIL           8
#define FAULT_LEDS                0x3FF

#if 0 
static void fault(u8 code)
{
#if 0	
   /* set a warning bit in GPIO'S */
    writel(code, GPIO_A_OE_VAL);
    /* now enable GPIO for output */
    writel(FAULT_LEDS, GPIO_A_SET_OE);
#endif    
}
#endif
/* memory copy - don't use standard library so need our own implimentation */
void *memcpy(void *dest, const void *src, u32 n)
{
    void* saved_dest = dest;

    for (; n > 0; n--) {
        *((char *) dest++) = *((char *) src++);
    }

    return saved_dest;
}



/* get a byte from the IDE controller. */
static unsigned char __inline__ ide_inb(int dev, int port)
{
    return oxnas_sata_inb(dev, port);
}

/*
 * Wait until Busy bit is off, or timeout (in ms)
 * Return last status
 */
static u8 ide_wait(int dev, u32 t)
{
    u32 delay = 10 * t;		/* poll every 100 us */
    u8 c;

    while ((c = ide_inb(dev, ATA_STATUS)) & ATA_STAT_BUSY) {
	udelay(100);
	if (delay-- == 0) {
	    break;
	}
    }
    return (c);
}

/* code not needed as communications with the disk drive has been established by
 * the boot rom
 */
 
/* read a SATA control register */
static u32 scr_read(int device, unsigned int sc_reg)
{
    /* Setup adr of required register. std regs start eight into async region */
    *(sata_link_base + OX800SATA_LINK_RD_ADDR) =
	sc_reg*4 + SATA_STD_ASYNC_REGS_OFF;

    /* Wait for data to be available */
    int loops = MAX_SRC_READ_LOOPS;
    do {
	if (*(sata_link_base + OX800SATA_LINK_CONTROL) & 1UL) {
	    break;
	}
	udelay(10);
    } while (--loops);

    /* Read the data from the async register */
    return *(sata_link_base + OX800SATA_LINK_DATA);
}

/* write data to the SATA control registers. */
static void scr_write(int device, unsigned int sc_reg, u32 val)
{
    /* Setup the data for the write */
    *(sata_link_base + OX800SATA_LINK_DATA) = val;

    /* Setup adr of required register. std regs start eight into async region */
    *(sata_link_base + OX800SATA_LINK_WR_ADDR) =
	sc_reg*4 + SATA_STD_ASYNC_REGS_OFF;

    /* Wait for data to be written */
    int loops = MAX_SRC_WRITE_LOOPS;
    do {
	if (*(sata_link_base + OX800SATA_LINK_CONTROL) & 1UL) {
	    break;
	}
	udelay(10);
    } while (--loops);
}

/* copy the data from internal shadowing RAM buffers to the SATA registers. */
void xfer_wr_shadow_to_orbs(int device)
{
    *(sata_regs_base + SATA_ORB1_OFF) = wr_sata_orb1;
    *(sata_regs_base + SATA_ORB2_OFF) = wr_sata_orb2;
    *(sata_regs_base + SATA_ORB3_OFF) = wr_sata_orb3;
    *(sata_regs_base + SATA_ORB4_OFF) = wr_sata_orb4;
}


/**
 * Possible that ATA status will not become no-error, so must have timeout
 * @returns An int which is zero on error
 */
static inline int wait_no_error(int device)
{
    int status = 0;
    int loops = MAX_NO_ERROR_LOOPS;
    do {
	if (!
	    (oxnas_sata_inb(device, ATA_PORT_COMMAND) &
	     (1 << ATA_STATUS_ERR_BIT))) {
	    status = 1;
	    break;
	}
	udelay(10);
    } while (--loops);

    return status;
}

/**
 * Expect SATA command to always finish, perhaps with error
 * @returns An int which is zero on error
 */
static inline int wait_sata_command_not_busy(int device)
{
    /* Wait for data to be available */
    int status = 0;
    int loops = MAX_NOT_BUSY_LOOPS;
    do {
	if (!(*(sata_regs_base + SATA_COMMAND_OFF) & SATA_CMD_BUSY_BIT)) {
	    status = 1;
	    break;
	}
	udelay(100);
    } while (--loops);

    return status;
}


/* output a byte to the OXNAS SATA controller */
void oxnas_sata_outb(int device, int port, unsigned char val)
{
    typedef enum send_method {
	SEND_NONE,
	SEND_SIMPLE,
	SEND_CMD,
	SEND_CTL,
    } send_method_t;

    send_method_t send_regs = SEND_NONE;
    switch (port) {
    case ATA_PORT_CTL:
	wr_sata_orb4 &= ~(0xFFUL << SATA_CTL_BIT);
	wr_sata_orb4 |= (val << SATA_CTL_BIT);
	send_regs = SEND_CTL;
	break;
    case ATA_PORT_FEATURE:
	wr_sata_orb2 &= ~(0xFFUL << SATA_FEATURE_BIT);
	wr_sata_orb2 |= (val << SATA_FEATURE_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_NSECT:
	wr_sata_orb2 &= ~(0xFFUL << SATA_NSECT_BIT);
	wr_sata_orb2 |= (val << SATA_NSECT_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_LBAL:
	wr_sata_orb1 &= ~(0xFFUL << SATA_LBAL_BIT);
	wr_sata_orb1 |= (val << SATA_LBAL_BIT);
	wr_sata_orb3 &= ~(0xFFUL << SATA_LBAL_BIT);
	wr_sata_orb3 |= (val << SATA_LBAL_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_LBAM:
	wr_sata_orb1 &= ~(0xFFUL << SATA_LBAM_BIT);
	wr_sata_orb1 |= (val << SATA_LBAM_BIT);
	wr_sata_orb3 &= ~(0xFFUL << SATA_LBAM_BIT);
	wr_sata_orb3 |= (val << SATA_LBAM_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_LBAH:
	wr_sata_orb1 &= ~(0xFFUL << SATA_LBAH_BIT);
	wr_sata_orb1 |= (val << SATA_LBAH_BIT);
	wr_sata_orb3 &= ~(0xFFUL << SATA_LBAH_BIT);
	wr_sata_orb3 |= (val << SATA_LBAH_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_DEVICE:
	wr_sata_orb1 &= ~(0xFFUL << SATA_DEVICE_BIT);
	wr_sata_orb1 |= (val << SATA_DEVICE_BIT);
	send_regs = SEND_SIMPLE;
	break;
    case ATA_PORT_COMMAND:
	wr_sata_orb2 &= ~(0xFFUL << SATA_COMMAND_BIT);
	wr_sata_orb2 |= (val << SATA_COMMAND_BIT);
	send_regs = SEND_CMD;
	break;
    }

    u32 command;
    switch (send_regs) {
    case SEND_CMD:
	wait_sata_command_not_busy(device);
	command = *(sata_regs_base + SATA_COMMAND_OFF);
	command &= ~SATA_OPCODE_MASK;
	command |= SATA_CMD_WRITE_TO_ORB_REGS;
	xfer_wr_shadow_to_orbs(device);
	*(sata_regs_base + SATA_COMMAND_OFF) = command;
	wait_no_error(device);
	break;
    case SEND_CTL:
	wait_sata_command_not_busy(device);
	command = *(sata_regs_base + SATA_COMMAND_OFF);
	command &= ~SATA_OPCODE_MASK;
	command |= SATA_CMD_WRITE_TO_ORB_REGS_NO_COMMAND;
	xfer_wr_shadow_to_orbs(device);
	*(sata_regs_base + SATA_COMMAND_OFF) = command;
	wait_no_error(device);
	break;
    default:
	break;
    }
}

/* get a byte from the OXNAS SATA controller */
unsigned char oxnas_sata_inb(int device, int port)
{
    unsigned char val = 0;

    switch (port) {
    case ATA_PORT_CTL:
	val =
	    (*(sata_regs_base + SATA_ORB4_OFF) & (0xFFUL << SATA_CTL_BIT))
	    >> SATA_CTL_BIT;
	break;
    case ATA_PORT_FEATURE:
	val =
	    (*(sata_regs_base + SATA_ORB2_OFF) &
	     (0xFFUL << SATA_FEATURE_BIT)) >> SATA_FEATURE_BIT;
	break;
    case ATA_PORT_NSECT:
	val =
	    (*(sata_regs_base + SATA_ORB2_OFF) &
	     (0xFFUL << SATA_NSECT_BIT)) >> SATA_NSECT_BIT;
	break;
    case ATA_PORT_LBAL:
	val =
	    (*(sata_regs_base + SATA_ORB1_OFF) & (0xFFUL << SATA_LBAL_BIT))
	    >> SATA_LBAL_BIT;
	break;
    case ATA_PORT_LBAM:
	val =
	    (*(sata_regs_base + SATA_ORB1_OFF) & (0xFFUL << SATA_LBAM_BIT))
	    >> SATA_LBAM_BIT;
	break;
    case ATA_PORT_LBAH:
	val =
	    (*(sata_regs_base + SATA_ORB1_OFF) & (0xFFUL << SATA_LBAH_BIT))
	    >> SATA_LBAH_BIT;
	break;
    case ATA_PORT_DEVICE:
	val =
	    (*(sata_regs_base + SATA_ORB1_OFF) &
	     (0xFFUL << SATA_DEVICE_BIT)) >> SATA_DEVICE_BIT;
	break;
    case ATA_PORT_COMMAND:
	val =
	    (*(sata_regs_base + SATA_ORB2_OFF) &
	     (0xFFUL << SATA_COMMAND_BIT)) >> SATA_COMMAND_BIT;
	break;
    default:
	break;
    }

    return val;
}

/* select to which device the application is directing commands */
static inline void device_select(int device)
{
    int interface_no = 0;

    *(sata_regs_base + SATA_DEVICE_SELECT_OFF) =
	(interface_no << 4) | device;
}

inline int dma_sata_busy(void)
{
    return (*DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS)) &
	DMA_CTRL_STATUS_IN_PROGRESS;
}

/* wait while the DMA transfer occurs. */
static void wait_dma_sata_xfer(int device)
{
    /* Wait for ATA core not busy */
    int loops = MAX_DMA_XFER_LOOPS;
    do {
	if (*(sata_regs_base + SATA_INT_STATUS_OFF) &
	    (1 << SATA_INT_STATUS_EOC_RAW_BIT)) {
	    break;
	}
	udelay(100);
    } while (--loops);

    if (loops) {
	/* Check for ATA core error */
	if (*(sata_regs_base + SATA_INT_STATUS_OFF) &
	    (1 << SATA_INT_STATUS_ERROR_BIT)) {

	    /* Abort DMA, as will never complete due to SATA error */
	    unsigned long ctrl_status =
		*DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS);
	    ctrl_status |= DMA_CTRL_STATUS_RESET;
	    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS) =
		ctrl_status;

	    // Wait for the channel to become idle - should be quick as should
	    // finish after the next AHB single or burst transfer
	    loops = MAX_DMA_ABORT_LOOPS;
	    do {
		if (!
		    (*DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS) &
		     DMA_CTRL_STATUS_IN_PROGRESS)) {
		    break;
		}
		udelay(10);
	    } while (--loops);

	    // Deassert reset for the channel
	    ctrl_status =
		*DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS);
	    ctrl_status &= ~DMA_CTRL_STATUS_RESET;
	    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS) =
		ctrl_status;
	} else {
	    /* Poll for DMA completion */
	    loops = MAX_DMA_XFER_LOOPS;
	    do {
		if (!dma_sata_busy()) {
		    break;
		}
		udelay(100);
	    } while (--loops);
	}
    }
}

/* start the DMA transfer */
void dma_sata_start(void)
{
    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS) =
	encode_start(*
		     (DMA_CALC_REG_ADR
		      (SATA_DMA_CHANNEL, DMA_CTRL_STATUS)));
}

/* configure the DMA controller to transfer data from the IDE controller data 
 * buffers into memory.
 */
static void dma_start_sata_read(u32 * buffer, int num_bytes)
{
    // Assemble complete memory settings
    oxnas_dma_device_settings_t mem_settings = oxnas_ram_dma_rom_settings;
    mem_settings.address_ = (unsigned long) buffer;
    mem_settings.address_mode_ = OXNAS_DMA_MODE_INC;
    memcpy(&oxnas_sata_dma_settings, &oxnas_sata_dma_rom_settings,
	   sizeof(oxnas_dma_device_settings_t));

    /* configure the DMA controller to transfer the incomming data */
    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_CTRL_STATUS) =
	encode_control_status(&oxnas_sata_dma_settings, &mem_settings);
    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_BASE_SRC_ADR) =
	oxnas_sata_dma_rom_settings.address_;
    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_BASE_DST_ADR) =
	mem_settings.address_;
    *DMA_CALC_REG_ADR(SATA_DMA_CHANNEL, DMA_BYTE_CNT) =
	encode_final_eot(&oxnas_sata_dma_settings, &mem_settings,
			 num_bytes);

    dma_sata_start();
}


/* get read data from the SATA controller into memory */
void oxnas_sata_input_data(int device, u32 * sect_buf, int words)
{
    /* Select the required internal SATA drive */
    device_select(device);

    /* Start the DMA channel receiving data from the SATA core into the passed buffer */
    dma_start_sata_read(sect_buf, words << 2);

    /* Wait for SATA core and DMA to finish */
    wait_dma_sata_xfer(device);
}

/* fall through function to specialise in the oxnas sata driver */
static void __inline__ ide_outb(int dev, int port, unsigned char val)
{
    oxnas_sata_outb(dev, port, val);
}

/* fall through function to specialise in the oxnas sata driver */
static void input_data(int dev, u32 * sect_buf, int words)
{
    oxnas_sata_input_data(dev, sect_buf, words);
}

/* read data from an IDE (SATA) disk drive */
static u32 ide_read(int device, lbaint_t blknr, u32 blkcnt, u32 * buffer)
{
    u32 n = 0;
    unsigned char c;
    unsigned char pwrsave = 0;	/* power save */

    //      ide_led (DEVICE_LED(device), 1);        /* LED on       */

    /* Select device
     */
    ide_outb(device, ATA_DEV_HD, ATA_LBA | ATA_DEVICE(device));
    c = ide_wait(device, IDE_TIME_OUT);

    if (c & ATA_STAT_BUSY) {
	goto IDE_READ_E;
    }

    /* first check if the drive is in Powersaving mode, if yes,
     * increase the timeout value */
    ide_outb(device, ATA_COMMAND, ATA_CMD_CHK_PWR);
    udelay(50);

    c = ide_wait(device, IDE_TIME_OUT);	/* can't take over 500 ms */

    if (c & ATA_STAT_BUSY) {
	goto IDE_READ_E;
    }
    if ((c & ATA_STAT_ERR) == ATA_STAT_ERR) {
    } else {
	c = ide_inb(device, ATA_SECT_CNT);
	if (c == 0)
	    pwrsave = 1;
    }


    while (blkcnt-- > 0) {

	c = ide_wait(device, IDE_TIME_OUT);

	if (c & ATA_STAT_BUSY) {
	    break;
	}
	ide_outb(device, ATA_SECT_CNT, 1);
	ide_outb(device, ATA_LBA_LOW, (blknr >> 0) & 0xFF);
	ide_outb(device, ATA_LBA_MID, (blknr >> 8) & 0xFF);
	ide_outb(device, ATA_LBA_HIGH, (blknr >> 16) & 0xFF);

	ide_outb(device, ATA_DEV_HD, ATA_LBA |
		 ATA_DEVICE(device) | ((blknr >> 24) & 0xF));
	ide_outb(device, ATA_COMMAND, ATA_CMD_READ);

	udelay(50);

	if (pwrsave) {
	    c = ide_wait(device, IDE_SPIN_UP_TIME_OUT);	/* may take up to 4 sec */
	    pwrsave = 0;
	} else {
	    c = ide_wait(device, IDE_TIME_OUT);	/* can't take over 500 ms */
	}

	if ((c & (ATA_STAT_DRQ | ATA_STAT_BUSY | ATA_STAT_ERR)) !=
	    ATA_STAT_DRQ) {
	    break;
	}

	input_data(device, buffer, ATA_SECTORWORDS);
	(void) ide_inb(device, ATA_STATUS);	/* clear IRQ */

	++n;
	++blknr;
	buffer += ATA_SECTORWORDS;
    }
  IDE_READ_E:
    //      ide_led (DEVICE_LED(device), 0);        /* LED off      */
    return (n);
}

/* 10 sec wait 50 x 200ms for reset to complete */
#define PHY_LOOP_COUNT  50
/* reset the SATA PHY device and hence reset the attached disk drive */
static int phy_reset(int device)
{
    scr_write(device, SATA_SCR_CONTROL, 0x301);	/* Issue phy wake & core reset */
    scr_read(device, SATA_SCR_STATUS);	/* Dummy read; flush */
    scr_write(device, SATA_SCR_CONTROL, 0x300);	/* Issue phy wake & clear core reset */

    /* Wait for upto 5 seconds for PHY to become ready */
    int phy_status = 0;
    int loops = 0;
    do {
	udelay(200000);
	if ((scr_read(device, SATA_SCR_STATUS) & 0xf) == 0x3) {
	    phy_status = 1;
	    break;
	}
    } while (++loops < PHY_LOOP_COUNT);

    if (loops >= PHY_LOOP_COUNT) {
	printf("No SATA PHY found\n");
    }

    return phy_status;
}

static int wait_fis(void)
{
	
    /* Wait for upto 10 seconds for FIS to be received */
    int fis_status = 0;
    int loops = 0;
    do {
	udelay(200000);
	if 	(ide_inb(0, ATA_SECT_CNT) > 0) {
	    fis_status = 1;
	    break;
	}
    } while (++loops < PHY_LOOP_COUNT);

    if (loops >= PHY_LOOP_COUNT) {
	printf("No FIS received\n");
    }

    return fis_status;
}


u32 run_sata(u32 location, u32 length, u32* memory_location, int disk)
{
    /* initialise static data and set pointers to SATA input port. 
     * obtain the currently active port from status report
     * or use SATA 0 for version 0006 of the bootrom ASIC build .
     */
     if (disk == 0) {
    sata_regs_base = (volatile u32 *) SATA0_REGS_BASE;
    sata_link_base = (volatile u32 *) SATA0_LINK_REGS_BASE;
    }
    else
    {
    sata_regs_base = (volatile u32 *) SATA1_REGS_BASE;
    sata_link_base = (volatile u32 *) SATA1_LINK_REGS_BASE;
    }
    wr_sata_orb1 = 0;
    wr_sata_orb2 = 0;
    wr_sata_orb3 = 0;
    wr_sata_orb4 = 0;

    	if (0==phy_reset(0)) return 0;
	wait_fis();
	/* now get the real data */

	length = 1 + length / 512;	/* length is rounded down so add an initial sector to load */
	if (length == ide_read(0, location, length, ((u32 *) memory_location)))
	    return length;

	return 0;
}
