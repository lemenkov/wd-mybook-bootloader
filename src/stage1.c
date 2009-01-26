/* test program to display a banner when loaded and executed. 
 * It reports where the 'send_banner'function is currently loaded. 
 * 
 * J J Larkworthy 20 April 2006
 * 
 */
#include "oxnas.h"
#include "sata.h"
#include "debug.h"
#include "crc32.h"

#define readb(p)  (*(volatile u8 *)(p))
#define readl(p)  (*(volatile u32 *)(p))
#define writeb(v, p) (*(volatile u8 *)(p)= (v))
#define writel(v, p) (*(volatile u32*)(p)=(v))

/* Constants which can be asigned to OVERCLOCK_PLL in environment to affect PLL rate */
#define PLL_400MHZ	0x00003014
#define PLL_425MHZ	0x00002b10
#define PLL_437_5MHZ	0x00003e18
#define PLL_466MHZ	0x00003010
#define PLL_500MHZ	0x00003410
#define PLL_533MHZ	0x00003810

/* Forward declaration of static functions */
static void init_ddr(void);
static void test_memory(u32 memory);
static void start_timer(void);
static void init_sata_hw(void);
//static void reset_sata_hw(void);
static void set_pll(void);

static const u32 stage2_disk_sector[] =
    { SECTOR_BOOT_STAGE2, SECTOR_RECOVERY_STAGE2 };
static const u32 numStage2Images =
    sizeof(stage2_disk_sector) / sizeof(u32);
static u32 ddr_control;
static u32 stage2_size;
static u32 stage2_ram_addr;
static u32 *header_length;
static u32 *header_crc;
extern const char *build_string;

int main(void)
{
    int disk;
    int stage2ImageNumber;
    int status;


    /* remove the pending reset again */
    writel(0xFFFFFFFF, GPIO_B_CLR_OE);
    writel((readl(SYS_CTRL_GPIO_TERSEL_CTRL1) & ~0x4),
           SYS_CTRL_GPIO_TERSEL_CTRL1);
    writel((readl(SYS_CTRL_GPIO_SECSEL_CTRL1) & ~0x4),
           SYS_CTRL_GPIO_SECSEL_CTRL1);
    writel((readl(SYS_CTRL_GPIO_PRIMSEL_CTRL1) & ~0x4),
           SYS_CTRL_GPIO_PRIMSEL_CTRL1);

    ddr_control = 0;
    stage2_size = 100000;
    stage2ImageNumber = 0;
    stage2_ram_addr = 0x48d00000;
    header_length = (u32 *) (stage2_ram_addr - 8);
    header_crc = (u32 *) (stage2_ram_addr - 4);

    // Base the UART divider on the system clock
    int baud_divisor_x16 = NOMINAL_SYSCLK / 115200;
    if (readl(CONFIG_REGISTER) != readl(START_OF_ROM)) {
        /* use FPGA system clock speed. */
        baud_divisor_x16 = FPGA_CLK / 115200;
    } else {
        /* use ASIC nominal system clock. */
        baud_divisor_x16 = NOMINAL_SYSCLK / 115200;
    }
    start_timer();
    set_pll();

    // Initialise the UART to be used by the CoPro for debug messages
    debug_uart = (NS16550_t) UART_2_BASE;
    init_NS16550(debug_uart, baud_divisor_x16);


    putc_NS16550(debug_uart, 'N');

    init_ddr();
    putc_NS16550(debug_uart, 'A');

    test_memory(stage2_ram_addr);       /* sends 'S' after writing */
    putc_NS16550(debug_uart, 'O');

    /* no need to re-initialise SATA controller just load details. */
    if ((readl(BOOT_STATUS0) == 0x01)
        || (readl(BOOT_VERSION) == 0x36303030ul)) {
        disk = 0;
    } else {
        disk = 1;
    }

    do {
        init_sata_hw();

        status =
            run_sata(stage2_disk_sector[stage2ImageNumber], stage2_size,
                     (u32 *) header_length, disk);

        if (status == 0) {
            putstr(debug_uart, "X");
        } else {
            putstr(debug_uart, "x");
        }

        if (stage2ImageNumber > 0) {
            putstr(debug_uart, "R");
        } else {
            putstr(debug_uart, "_");
        }

        putc_NS16550(debug_uart, (char) ('0' + (char) disk));

        /* try the backup stage2 on this disk first (first tim round, at least we know
           it is working to some extent, go to next disk if this wraps round */
        if (++stage2ImageNumber >= numStage2Images) {
            stage2ImageNumber = 0;
            if (++disk > 1) {
                disk = 0;
            }
        }

    } while (!
             (status != 0 && *header_length
              && *header_length <= stage2_size
              && *header_crc == crc32(0, (unsigned char *) stage2_ram_addr,
                                      *header_length)));

    putstr(debug_uart, "800\r\n");

    putstr(debug_uart, build_string);

    putc_NS16550(debug_uart, '\r');
    putc_NS16550(debug_uart, '\n');


#if (OVERCLOCK_PLL)
    putstr(debug_uart, "Overclocking with ");
    putstr(debug_uart, ultohex(OVERCLOCK_PLL));
    putc_NS16550(debug_uart, ' ');
#endif                          // OVERCLOCK_PLL


    ((void (*)(void)) stage2_ram_addr) ();

    return 0;
}

/*******************************************************************
 *
 * Function:             init_sata_hw
 *
 * Description:          initialise secondary facilities for sata use
 *
 ******************************************************************/
#define SATA_RESET_BITS    ((1<<SYS_CTRL_RSTEN_SATA_BIT) | (1<<SYS_CTRL_RSTEN_SATA_PHY_BIT))
static void init_sata_hw(void)
{
    writel(SATA_RESET_BITS, SYS_CTRL_RSTEN_SET_CTRL);
    udelay(10);
    writel(SATA_RESET_BITS, SYS_CTRL_RSTEN_CLR_CTRL);
}

/*******************************************************************
 *
 * Function:             reset_sata_hw
 *
 * Description:          puts  sata hardware back into reset before use
 *
 ******************************************************************/
// static void reset_sata_hw(void)
// {
    // writel((1<<SYS_CTRL_RSTEN_SATA_BIT)|(1<<SYS_CTRL_RSTEN_SATA_PHY_BIT), SYS_CTRL_RSTEN_SET_CTRL);
    // udelay(10);
    // writel((1<<SYS_CTRL_CKEN_SATA_BIT), SYS_CTRL_CKEN_CLR_CTRL);
// }

static void init_ddr(void)
{
    /* enable DDR clocks */
    writel((1 << SYS_CTRL_CKEN_DDR_BIT), SYS_CTRL_CKEN_SET_CTRL);

    /* Release DDR from reset */
    writel((1 << SYS_CTRL_RSTEN_DDR_BIT), SYS_CTRL_RSTEN_CLR_CTRL);

    switch (ddr_control) {
    case 0:                    /* using SDR */
        {
            if (readl(CONFIG_REGISTER) != readl(START_OF_ROM)) {
                /* Table of address, data for loading into the DRAM controller */
                /* Configure the DRAM controller for 2 off 16Mx16 SDR memories configured
                 * as 32-bit wide. */
                writel(0x00000000, 0x45800000); /* SDR width 32 */
                writel(0x80000000, 0x45800004); /* Enable DDR core, but not clients yet */
                writel(0x80000001, 0x45800014); /* Set DLL mode - automatic */
                udelay(400);    /* >200uS */
                writel(0x80200000, 0x4580000c); /* Assert CKE for all further commands */
                writel(0x80280400, 0x4580000c); /* Issue precharge to all banks */
                writel(0x80200032, 0x4580000c); /* Set burst length 4, sequential,CAS 3 */
                writel(0x80240000, 0x4580000c); /* Issue auto-refresh command, CKE not asserted */
                udelay(200);    /* 200 clock cycles ? */
                writel(0x80240000, 0x4580000c); /* Issue auto-refresh command, CKE not asserted */
                udelay(200);    /* 200 clock cycles ? */
                writel(0x80200000, 0x4580000c); /* Assert CKE for all further commands */
                writel(0x80200032, 0x4580000c); /* Set burst length 4, sequential,CAS 3 */
                writel(0x000b0186, 0x45800000); /* SDR, size and width and refresh rate */
                writel(0x00000124, 0x45800024); /* Set I/O drive strengths */
                writel(0x0000001f, 0x45800028); /* Enable all arbiter features */
                writel(0x00000000, 0x45800018); /* Disable all monitoring */
                writel(0xFFFFFFFF, 0x45800010); /* Disable all read buffering, due to h/w bug */
                writel(0x800000ff, 0x45800004); /*Enable all AHB clients and DDR core output */
            } else {
                /* Configure the DRAM controller for 1 off 32Mx16 DDR memory */
                writel(0x08, 0x4500002C);
                writel(0x400, 0x45000038);
                writel(0x80100000, 0x45800000);
                writel(0x80000000, 0x45800004); /* Enable DDR core, but not clients yet */
                writel(0x1e4, 0x45800024);
                writel(0x80000008, 0x45800014);
                udelay(400);    /* >200us */
                writel(0x801b061a, 0x45800000);
                udelay(200);    /* 200uS delay */
                writel(0x80280400, 0x4580000c);
                udelay(200);    /* 200uS delay */
                writel(0x80210000, 0x4580000c);
                udelay(200);    /* 200uS delay */
                writel(0x80200063, 0x4580000c);
                writel(0x0000001f, 0x45800028); /* Enable all arbiter features */
                writel(0x00000000, 0x45800018); /* Disable all monitoring */
                writel(0xFFFFFFFF, 0x45800010); /* Disable all read buffering, due to h/w bug */
                writel(0x00000000, 0x4580002C); /* Do NOT disable HPROT, ie want write coherency */
                writel(0x800000ff, 0x45800004); /* Enable all client interfaces */

            }
        }
        break;

    case 1:                    /* using DDR */

        {
            /* Configure the DRAM controller for 1 off 32Mx16 DDR memory */
            writel(0x08, 0x4500002C);
            writel(0x400, 0x45000038);
            writel(0x80100000, 0x45800000);
            writel(0x80000000, 0x45800004);     /* Enable DDR core, but not clients yet */
            writel(0x1e4, 0x45800024);  // Drive strength 7 on clocks, 4 on data and other pins.
            // This was setting the DLL to a fixed manual value of 8…writel(0x80000008, 0x45800014);
            // Should set to ‘automatic’ DLL mode (i.e. the DLL tracks delays), with an initial value of 1…
            writel(0xE0000001, 0x45800014);
            udelay(400);        /* >200us */
            // After this time, the DLL should have locked.
            // Ideally, we should sample the STAT_BUSY flag (bit 31 of 0x45800008).
            // It should be 0. If 1, the DLL has failed to lock.
// This is catastrophic, indicative of a hardware fault (on the PCB on in the chip).
// Assume we can’t do anything useful if this occurs so carry on anyway.
            // Now re-program the DLL to ‘automatic-offset’ mode, with offset +2…
            writel(0xA0000002, 0x45800014);
            writel(0x801a0000, 0x45800000);     // Set up DDR size 32MB, width 16-bit but not refreshing yet
            // Was enabling refreshing at this point but this could interfere with the initialization sequence below…
//          writel(0x801b061a, 0x45800000);
            udelay(200);        /* 200uS delay */
            // Should assert CKE first without anything else…
            writel(0x80FC0000, 0x4580000c);     // Write command to SDRAM: Assert CKE.
            udelay(200);        /* 200uS delay */
            writel(0x80280400, 0x4580000c);     // Write command to SDRAM: Precharge all banks.
            udelay(200);        /* 200uS delay */
            writel(0x80210000, 0x4580000c);     // Write command to SDRAM: Load ‘extended mode’ reg: enable SDRAM’s DLL.
            udelay(200);        /* 200uS delay */
            writel(0x80200163, 0x4580000c);     // Write command to SDRAM: Load ‘mode’ reg: CAS latency 2.5; burst length 8, sequential bursts. Reset SDRAM’s DLL.

            // Should then have another precharge…
            writel(0x80280400, 0x4580000c);     // Write command to SDRAM: Precharge all banks again.
            udelay(200);        /* 200uS delay */
            writel(0x801b030c, 0x45800000);     // Enable auto-refreshing now (every 7.81us)
            udelay(200);        /* 200uS delay */
            // Waiting for 200us here ensures that at least 2 auto-refresh commands get issued, as required, before any other accesses.

            // Re-write mode register but with ‘Reset DLL’ bit cleared…
            // Micron SDRAMs have a self-clearing ‘Reset DLL’ bit but JEDEC recommendation is to manually clear.
            writel(0x80200063, 0x4580000c);     // Write command to SDRAM: Load ‘mode’ reg: CAS latency2.5; burst length 8, sequential bursts.

            writel(0x0000001f, 0x45800028);     /* Enable all arbiter features */
            writel(0x00000000, 0x45800018);     /* Disable all monitoring */
            writel(0xFFFFFFFF, 0x45800010);     /* Disable all read buffering, due to h/w bug */
            writel(0x00000000, 0x4580002C);     /* Do NOT disable HPROT, ie want write coherency */
            writel(0x800000ff, 0x45800004);     /* Enable all client interfaces */
        }

        break;
    default:
        for (;;)                /* spin as not selected correctly */
            break;
    }
}

/* functions to handle modulus arithmetic on timer */
/*******************************************************************
 *
 * Function:             test_memory
 *
 * Description:          Check that memory is accessible and retains data.
 *
 ******************************************************************/

static void test_memory(u32 memory)
{
    volatile u32 *read;
    volatile u32 *write;
    u32 pattern;
    u8 check;
    u16 i;

    do {
        check = 0;
        read = write = (volatile u32 *) memory;
        pattern = 0xAA55AA55;
        for (i = 0; i < 4; i++) {
            *write++ = pattern;
            pattern = ~pattern;
        }
        putc_NS16550(debug_uart, 'S');
        pattern = 0xAA55AA55;
        for (i = 0; i < 4; i++) {
            check += (pattern == *read++) ? 1 : 0;
            pattern = ~pattern;
        }

    } while (check < 4);
}


/* functions to handle modulus arithmetic on timer */
/*******************************************************************
 *
 * Function:             time_dif
 *
 * Description:          calculate the difference between 2 times in ticks.
 *
 ******************************************************************/
long time_dif(u32 limit, u32 now)
{
    long ret_val;
    ret_val = limit - now;
    if (ret_val < (-(long) RPSA_TIMER_MODULUS) / 2)
        ret_val += RPSA_TIMER_MODULUS;
    return ret_val;
}

/*******************************************************************
 *
 * Function:             set_tim_limit
 *
 * Description:          create a time limit
 *
 ******************************************************************/
u32 set_time_limit(u32 time_limit)
{
    u32 ret_val = time_now() + time_limit;
    while (ret_val > RPSA_TIMER_MODULUS)
        ret_val -= RPSA_TIMER_MODULUS;
    return ret_val;
}

/*******************************************************************
 *
 * Function:             start_timer
 *
 * Description:          initialise the timer for use
 *
 ******************************************************************/
static void start_timer(void)
{
    /* enable clock in sys_ctrl at 5MHz */
    writel(SLOW_TIMER_TICK, C_SYSCTRL_CKCTRL_CTRL_ADDR);

    /* set timer 1 arps running free running prescale = 255 
     * tick = 51us 
     * maximum time out 3 seconds 
     */
    writel(RPS_CLK_CTRL_DATA, RPSA_CLK_CTRL);
}


/*******************************************************************
 *
 * Function:             time_now
 *
 * Description:          function to return the time in seconds 
 *                       from an internal source 
 * 
 * Note: RPS clock counts down so change to an up clock 
 ******************************************************************/
u32 time_now(void)
{
    return RPSA_TIMER_MODULUS - readl(RPSA_CLK_COUNT);
}

/*******************************************************************
 *
 * Function:             udelay
 *
 * Description:          function to pause operation in a busy wait 
 *                       for a number of microseconds.
 *
 ******************************************************************/
void udelay(unsigned long time)
{
    u32 now;
    u32 start = time_now();
    u32 limit = USECS2TICKS(time);
    if (limit < 2)
        limit = 2;
    long diff = 0;
    while (diff < limit) {
        now = time_now();
        diff = time_dif(now, start);
    }
}

/**
 *  Function:    set_pll
 *
 *  Description Modify the PLL setting to alter the system clock speed
 */
static void set_pll(void)
{
#if (OVERCLOCK_PLL)
    /* Delay for 1 second so the broken JTAG can get control */
    udelay(1000000);

    /* 0xBEADFACE -> PLL_KEY */
    /* Bypass PLL */
    u32 pll_sys = *(volatile u32 *) SYS_CTRL_PLLSYS_CTRL;
    pll_sys |= 0x20000;
    *(volatile u32 *) SYS_CTRL_PLLSYS_KEY_CTRL = 0xbeadface;
    *(volatile u32 *) SYS_CTRL_PLLSYS_CTRL = pll_sys;

    /* 0xBEADFACE -> PLL_KEY */
    /* Set m,p and s for PLL at 400MHz */
    pll_sys &= 0xffff0000;
    pll_sys |= OVERCLOCK_PLL;
    *(volatile u32 *) SYS_CTRL_PLLSYS_KEY_CTRL = 0xbeadface;
    *(volatile u32 *) SYS_CTRL_PLLSYS_CTRL = pll_sys;

    /* Wait at least 300uS */
    udelay(1000);

    /* 0xBEADFACE -> PLL_KEY */
    /* Disable PLL bypass */
    pll_sys &= 0xfffdffff;
    *(volatile u32 *) SYS_CTRL_PLLSYS_KEY_CTRL = 0xbeadface;
    *(volatile u32 *) SYS_CTRL_PLLSYS_CTRL = pll_sys;
#endif                          // OVERCLOCK_PLL
}
