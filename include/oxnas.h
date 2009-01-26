/*******************************************************************
 *
 * File:            oxnas.h
 *
 * Description:     oxnas configuration header file 
 *
 * Date:            27 July 2005
 *
 * Author:          J J Larkworthy
 *
 * Copyright:       Oxford Semiconductor Ltd, 2005
 *
 *
 *******************************************************************/
#ifndef _OXNAS_H_
#define _OXNAS_H_

#include "serial_reg.h"
#include "types.h"

/* single clock on FPGA and no overclocking */
#define FPGA_CLK        (250000000)

#define NOMINAL_ARMCLK  ((NOMINAL_PLL400_FREQ) / 2)
#define NOMINAL_SYSCLK  ((NOMINAL_PLL400_FREQ) / 4)
#define NOMINAL_PCICLK  ((NOMINAL_PLL400_FREQ) / 12)

#define PLL_400MHZ   0x00003014
#define PLL_425MHZ   0x00002b10
#define PLL_437_5MHZ 0x00003e18
#define PLL_466MHZ   0x00003010
//#define OVERCLOCK_PLL (PLL_425MHZ)

/* Core base addresses */
#define ROM_BASE                0x40000000
#define USB_BASE                0x40200000
#define MAC_BASE                0x40400000
#define PCI_CSRS_BASE           0x40600000      
#define PCI_BASE                0x40800000

#define STATIC_CS0_BASE         0x41000000
#define STATIC_CS1_BASE         0x41400000
#define STATIC_CS2_BASE         0x41800000
#define STATIC_CONTROL_BASE     0x41C00000

#define SATA_DATA_BASE          0x42000000
#define DPE_BASE                0x43000000
#define APB_BRIDGE_A_BASE       0x44000000
#define APB_BRIDGE_B_BASE       0x45000000
#define SDRAM_CTRL_BASE         0x4A000000

#define UART_1_BASE (APB_BRIDGE_A_BASE + 0x200000)
#define UART_2_BASE (APB_BRIDGE_A_BASE + 0x300000)
#define UART_3_BASE (APB_BRIDGE_A_BASE + 0x900000)
#define UART_4_BASE (APB_BRIDGE_A_BASE + 0xA00000)

#define GPIO_1_BASE             (APB_BRIDGE_A_BASE)
#define GPIO_2_BASE             (APB_BRIDGE_A_BASE + 0x100000)
#define I2C_BASE                (APB_BRIDGE_A_BASE + 0x400000)
#define I2S_BASE                (APB_BRIDGE_A_BASE + 0x500000)
#define FAN_MON_BASE            (APB_BRIDGE_A_BASE + 0x600000)
#define PWM_BASE                (APB_BRIDGE_A_BASE + 0x700000)
#define IRRX_BASE               (APB_BRIDGE_A_BASE + 0x800000)

#define SYS_CONTROL_BASE        (APB_BRIDGE_B_BASE)
#define CLOCK_CONTROL_BASE      (APB_BRIDGE_B_BASE + 0x100000)
#define RTC_BASE                (APB_BRIDGE_B_BASE + 0x200000)
#define RPS_BASE                (APB_BRIDGE_B_BASE + 0x300000)
#define COPRO_RPS_BASE          (APB_BRIDGE_B_BASE + 0x400000)
#define AHB_MON_BASE            (APB_BRIDGE_B_BASE + 0x500000)
#define DMA_BASE                (APB_BRIDGE_B_BASE + 0x600000)
#define DPE_REGS_BASE           (APB_BRIDGE_B_BASE + 0x700000)
#define IBW_REGS_BASE           (APB_BRIDGE_B_BASE + 0x780000)
#define DDR_REGS_BASE           (APB_BRIDGE_B_BASE + 0x800000)
#define SATA0_REGS_BASE         (APB_BRIDGE_B_BASE + 0x900000)
#define SATA0_LINK_REGS_BASE    (APB_BRIDGE_B_BASE + 0x940000)
#define SATA1_REGS_BASE         (APB_BRIDGE_B_BASE + 0x980000)
#define SATA1_LINK_REGS_BASE    (APB_BRIDGE_B_BASE + 0x9c0000)
#define DMA_CHECKSUM_BASE       (APB_BRIDGE_B_BASE + 0xA00000)
#define COPRO_REGS_BASE         (APB_BRIDGE_B_BASE + 0xB00000)
#define DMA_SG_BASE             (APB_BRIDGE_B_BASE + 0xC00000)

/* System Control registers */
#define SYS_CTRL_USB11_CTRL             (SYS_CONTROL_BASE + 0x00)
#define SYS_CTRL_PCI_CTRL0              (SYS_CONTROL_BASE + 0x04)
#define SYS_CTRL_PCI_CTRL1              (SYS_CONTROL_BASE + 0x08)
#define SYS_CTRL_GPIO_PRIMSEL_CTRL0     (SYS_CONTROL_BASE + 0x0C)
#define SYS_CTRL_GPIO_PRIMSEL_CTRL1     (SYS_CONTROL_BASE + 0x10)
#define SYS_CTRL_GPIO_SECSEL_CTRL0      (SYS_CONTROL_BASE + 0x14)
#define SYS_CTRL_GPIO_SECSEL_CTRL1      (SYS_CONTROL_BASE + 0x18)
#define SYS_CTRL_GPIO_TERSEL_CTRL0      (SYS_CONTROL_BASE + 0x8C)
#define SYS_CTRL_GPIO_TERSEL_CTRL1      (SYS_CONTROL_BASE + 0x90)
#define SYS_CTRL_USB11_STAT             (SYS_CONTROL_BASE + 0x1c)
#define SYS_CTRL_PCI_STAT               (SYS_CONTROL_BASE + 0x20)
#define SYS_CTRL_CKEN_CTRL              (SYS_CONTROL_BASE + 0x24)
#define SYS_CTRL_RSTEN_CTRL             (SYS_CONTROL_BASE + 0x28)
#define SYS_CTRL_CKEN_SET_CTRL          (SYS_CONTROL_BASE + 0x2C)
#define SYS_CTRL_CKEN_CLR_CTRL          (SYS_CONTROL_BASE + 0x30)
#define SYS_CTRL_RSTEN_SET_CTRL         (SYS_CONTROL_BASE + 0x34)
#define SYS_CTRL_RSTEN_CLR_CTRL         (SYS_CONTROL_BASE + 0x38)
#define SYS_CTRL_USBHSMPH_CTRL          (SYS_CONTROL_BASE + 0x40)
#define SYS_CTRL_USBHSMPH_STAT          (SYS_CONTROL_BASE + 0x44)
#define SYS_CTRL_PLLSYS_CTRL            (SYS_CONTROL_BASE + 0x48)
#define SYS_CTRL_SEMA_STAT              (SYS_CONTROL_BASE + 0x4C)
#define SYS_CTRL_SEMA_SET_CTRL          (SYS_CONTROL_BASE + 0x50)
#define SYS_CTRL_SEMA_CLR_CTRL          (SYS_CONTROL_BASE + 0x54)
#define SYS_CTRL_SEMA_MASKA_CTRL        (SYS_CONTROL_BASE + 0x58)
#define SYS_CTRL_SEMA_MASKB_CTRL        (SYS_CONTROL_BASE + 0x5C)
#define SYS_CTRL_SEMA_MASKC_CTRL        (SYS_CONTROL_BASE + 0x60)
#define SYS_CTRL_CKCTRL_CTRL            (SYS_CONTROL_BASE + 0x64)
#define SYS_CTRL_COPRO_CTRL             (SYS_CONTROL_BASE + 0x68)
#define SYS_CTRL_PLLSYS_KEY_CTRL        (SYS_CONTROL_BASE + 0x6C)

#define SYS_CTRL_CKEN_DMA_BIT       1
#define SYS_CTRL_CKEN_DDR_BIT       3
#define SYS_CTRL_CKEN_SATA_BIT      4
#define SYS_CTRL_CKEN_MAC_BIT       7
#define SYS_CTRL_CKEN_PCI_BIT       8
#define SYS_CTRL_CKEN_STATIC_BIT    9

#define SYS_CTRL_RSTEN_ARM_BIT      0
#define SYS_CTRL_RSTEN_MAC_BIT      6
#define SYS_CTRL_RSTEN_PCI_BIT      7
#define SYS_CTRL_RSTEN_DMA_BIT      8
#define SYS_CTRL_RSTEN_DDR_BIT      10
#define SYS_CTRL_RSTEN_SATA_BIT     11
#define SYS_CTRL_RSTEN_SATA_PHY_BIT 12
#define SYS_CTRL_RSTEN_STATIC_BIT   15
#define SYS_CTRL_RSTEN_UART1_BIT    17
#define SYS_CTRL_RSTEN_UART2_BIT    18
#define SYS_CTRL_RSTEN_UART3_BIT    22
#define SYS_CTRL_RSTEN_UART4_BIT    23

/* GPIO */
#define GPIO_A_DATA                   ((GPIO_1_BASE) + 0x0)
#define GPIO_A_OE_VAL                 ((GPIO_1_BASE) + 0x4)
#define GPIO_A_SET_OE                 ((GPIO_1_BASE) + 0x1C)
#define GPIO_A_CLR_OE                 ((GPIO_1_BASE) + 0x20)
#define GPIO_A_INPUT_DEBOUNCE_ENABLE  ((GPIO_1_BASE) + 0x20)

#define GPIO_B_DATA                   ((GPIO_2_BASE) + 0x0)
#define GPIO_B_OE_VAL                 ((GPIO_2_BASE) + 0x4)
#define GPIO_B_SET_OE                 ((GPIO_2_BASE) + 0x1C)
#define GPIO_B_CLR_OE                 ((GPIO_2_BASE) + 0x20)
#define GPIO_B_INPUT_DEBOUNCE_ENABLE  ((GPIO_2_BASE) + 0x20)

#define DTCP_ENABLED

#define CONFIG_REGISTER  0x50000000
#define START_OF_ROM     0x40000000

//#define SYS_CTRL         0x4c000000
#define RESET_REG        28

#define START_OF_SRAM     0x4C000000
/* define the maximum usable static RAM space */
#define MAX_BUFFER       (0x8000 - 1024)

#define RPSA              0X453003C0

#define C_SYSCTRL_CKCTRL_CTRL_ADDR     (0X45000064)
#define SLOW_TIMER_TICK                (0X100)

/* clock control registers and data */
#define RPS_CLK_CTRL_DATA              (0x88)	/* enabled, free run, 256 prescale) */
#define RPSA_CLK_CTRL                  (0x45300208)
#define RPSA_CLK_COUNT                 (0x45300204)
#define RPSA_TIMER_WIDTH               24
#define RPSA_TIMER_MODULUS             (1<<RPSA_TIMER_WIDTH)

#define SECONDS                        19531
/* 1 tick is 51us so convert USECS2TICKS as */
// #define USECS2TICKS(A)                 (A/51)
#define USECS2TICKS(A)             ((5*A)/256 )


/**
 * SATA related definitions
 */
#define ATA_PORT_CTL        0
#define ATA_PORT_FEATURE    1
#define ATA_PORT_NSECT      2
#define ATA_PORT_LBAL       3
#define ATA_PORT_LBAM       4
#define ATA_PORT_LBAH       5
#define ATA_PORT_DEVICE     6
#define ATA_PORT_COMMAND    7

/* The offsets to the SATA registers */
#define SATA_ORB1_OFF           0
#define SATA_ORB2_OFF           1
#define SATA_ORB3_OFF           2
#define SATA_ORB4_OFF           3
#define SATA_ORB5_OFF           4

#define SATA_FIS_ACCESS         11
#define SATA_INT_STATUS_OFF     12	/* Read only */
#define SATA_INT_CLR_OFF        12	/* Write only */
#define SATA_INT_ENABLE_OFF     13	/* Read only */
#define SATA_INT_ENABLE_SET_OFF 13	/* Write only */
#define SATA_INT_ENABLE_CLR_OFF 14	/* Write only */
#define SATA_VERSION_OFF        15
#define SATA_BURST_BUF_CTRL_OFF 18
#define SATA_CONTROL_OFF        23
#define SATA_COMMAND_OFF        24
#define SATA_DEVICE_SELECT_OFF  25
#define SATA_DEVICE_CONTROL_OFF 26
#define SATA_DRIVE_CONTROL_OFF  27

/* The offsets to the link registers that are access in an asynchronous manner */
#define OX800SATA_LINK_DATA     0x00000000
#define OX800SATA_LINK_RD_ADDR  0x00000001
#define OX800SATA_LINK_WR_ADDR  0x00000002
#define OX800SATA_LINK_CONTROL  0x00000003

/* SATA interrupt status register fields */
#define SATA_INT_STATUS_EOC_RAW_BIT     8
#define SATA_INT_STATUS_EODC_RAW_BIT    9
#define SATA_INT_STATUS_ERROR_BIT       10
#define SATA_INT_STATUS_EOADT_RAW_BIT   13

/* ATA status (7) register field definitions */
#define ATA_STATUS_BSY_BIT     7
#define ATA_STATUS_DRDY_BIT    6
#define ATA_STATUS_DF_BIT      5
#define ATA_STATUS_DRQ_BIT     3
#define ATA_STATUS_ERR_BIT     0

/* ATA device (6) register field definitions */
#define ATA_DEVICE_FIXED_MASK 0xA0
#define ATA_DEVICE_DRV_BIT 4
#define ATA_DEVICE_DRV_NUM_BITS 1
#define ATA_DEVICE_LBA_BIT 6

/* ATA control (0) register field definitions */
#define ATA_CTL_SRST_BIT   2

/* ATA Command register initiated commands */
#define ATA_CMD_INIT    0x91
#define ATA_CMD_IDENT   0xEC

/* SATA core command register commands */
#define SATA_CMD_WRITE_TO_ORB_REGS_NO_COMMAND   1
#define SATA_CMD_WRITE_TO_ORB_REGS              2
#define SATA_CMD_READ_ALL_REGS                  3
#define SATA_CMD_READ_STATUS_REG                4

#define SATA_CMD_BUSY_BIT 7

#define SATA_OPCODE_MASK 0x3

#define SATA_LBAL_BIT    0
#define SATA_LBAM_BIT    8
#define SATA_LBAH_BIT    16
#define SATA_DEVICE_BIT  24
#define SATA_NSECT_BIT   0
#define SATA_FEATURE_BIT 16
#define SATA_COMMAND_BIT 24
#define SATA_CTL_BIT     24

#define SATA_STD_ASYNC_REGS_OFF 0x20
#define SATA_SCR_STATUS      0
#define SATA_SCR_ERROR       1
#define SATA_SCR_CONTROL     2
#define SATA_SCR_ACTIVE      3
#define SATA_SCR_NOTIFICAION 4

#define SATA_BURST_BUF_FORCE_EOT_BIT        0
#define SATA_BURST_BUF_DATA_INJ_ENABLE_BIT  1
#define SATA_BURST_BUF_DIR_BIT              2
#define SATA_BURST_BUF_DATA_INJ_END_BIT     3
#define SATA_BURST_BUF_FIFO_DIS_BIT         4
#define SATA_BURST_BUF_DIS_DREQ_BIT         5
#define SATA_BURST_BUF_DREQ_BIT             6

/* SATA boot parameter offsets into the MBR */
#define BOOT_CHECKVALUE_OFFSET    0x01b0
#define BOOT_LENGTH_OFFSET	0x01b8
#define BOOT_LOCATION_OFFSET    0x01b4

#define BOOT_STATUS0     0x4C007FF8
#define BOOT_VERSION     0x40004FFC

/* COPRO defines */
#define COPRO_BASE_REG            0x45b00000
#define COPRO_RESET_ADDRESS_REG   (0x45000068)
#define COPRO_RESET_BIT           (1<<1)

/* Image locations (by sector number) */
#define SECTOR_BOOT_STAGE2 64
#define SECTOR_RECOVERY_STAGE2 10608

/* macros to make reading and writing hardware easier */
#define readb(p)  (*(volatile u8 *)(p))
#define readl(p)  (*(volatile u32 *)(p))
#define writeb(v, p) (*(volatile u8 *)(p)= (v))
#define writel(v, p) (*(volatile u32*)(p)=(v))

extern void udelay(unsigned long time);
extern u32 time_now(void);


#endif				/* _OXNAS_H_ */
