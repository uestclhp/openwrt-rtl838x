// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/iopoll.h>
#include <linux/mdio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phylink.h>
#include <net/dsa.h>

#include "../ethernet/rtl838x_eth.h"
#include <asm/mach-rtl838x/mach-rtl838x.h>

#define RTL838X_GPIO_PABC_DIR	(GPIO_CTRL_REG_BASE + 0x8)
#define RTL838X_GPIO_A1		25
#define RTL838X_GPIO_PABC_DATA	(GPIO_CTRL_REG_BASE + 0xc)

#define RTL8380_VERSION_A 'A'
#define RTL8380_VERSION_B 'B'

struct rtl838x_port {
	bool enable;
	u32 pm;
	u16 pvid;
};


struct rtl838x_priv {
	struct dsa_switch *ds;
	struct device *dev;
	u32 id;
	char version;
	u32 base_port;
	struct rtl838x_port ports[32]; /* TODO: correct size! */
	struct mutex reg_mutex;
};

#define MIB_DESC(_size, _offset, _name) {.size = _size, .offset = _offset, .name = _name}
struct rtl838x_mib_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

static const struct rtl838x_mib_desc rtl838x_mib[] = {
	MIB_DESC(2, 0xf8, "ifInOctets"),
	MIB_DESC(2, 0xf0, "ifOutOctets"),
	MIB_DESC(1, 0xec, "dot1dTpPortInDiscards"),
	MIB_DESC(1, 0xe8, "ifInUcastPkts"),
	MIB_DESC(1, 0xe4, "ifInMulticastPkts"),
	MIB_DESC(1, 0xe0, "ifInBroadcastPkts"),
	MIB_DESC(1, 0xdc, "ifOutUcastPkts"),
	MIB_DESC(1, 0xd8, "ifOutMulticastPkts"),
	MIB_DESC(1, 0xd4, "ifOutBroadcastPkts"),
	MIB_DESC(1, 0xd0, "ifOutDiscards"),
	MIB_DESC(1, 0xcc, ".3SingleCollisionFrames"),
	MIB_DESC(1, 0xc8, ".3MultipleCollisionFrames"),
	MIB_DESC(1, 0xc4, ".3DeferredTransmissions"),
	MIB_DESC(1, 0xc0, ".3LateCollisions"),
	MIB_DESC(1, 0xbc, ".3ExcessiveCollisions"),
	MIB_DESC(1, 0xb8, ".3SymbolErrors"),
	MIB_DESC(1, 0xb4, ".3ControlInUnknownOpcodes"),
	MIB_DESC(1, 0xb0, ".3InPauseFrames"),
	MIB_DESC(1, 0xac, ".3OutPauseFrames"),
	MIB_DESC(1, 0xa8, "DropEvents"),
	MIB_DESC(1, 0xa4, "tx_BroadcastPkts"),
	MIB_DESC(1, 0xa0, "tx_MulticastPkts"),
	MIB_DESC(1, 0x9c, "CRCAlignErrors"),
	MIB_DESC(1, 0x98, "tx_UndersizePkts"),
	MIB_DESC(1, 0x94, "rx_UndersizePkts"),
	MIB_DESC(1, 0x90, "rx_UndersizedropPkts"),
	MIB_DESC(1, 0x8c, "tx_OversizePkts"),
	MIB_DESC(1, 0x88, "rx_OversizePkts"),
	MIB_DESC(1, 0x84, "Fragments"),
	MIB_DESC(1, 0x80, "Jabbers"),
	MIB_DESC(1, 0x7c, "Collisions"),
	MIB_DESC(1, 0x78, "tx_Pkts64Octets"),
	MIB_DESC(1, 0x74, "rx_Pkts64Octets"),
	MIB_DESC(1, 0x70, "tx_Pkts65to127Octets"),
	MIB_DESC(1, 0x6c, "rx_Pkts65to127Octets"),
	MIB_DESC(1, 0x68, "tx_Pkts128to255Octets"),
	MIB_DESC(1, 0x64, "rx_Pkts128to255Octets"),
	MIB_DESC(1, 0x60, "tx_Pkts256to511Octets"),
	MIB_DESC(1, 0x5c, "rx_Pkts256to511Octets"),
	MIB_DESC(1, 0x58, "tx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x54, "rx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x50, "tx_Pkts1024to1518Octets"),
	MIB_DESC(1, 0x4c, "rx_StatsPkts1024to1518Octets"),
	MIB_DESC(1, 0x48, "tx_Pkts1519tomaxOctets"),
	MIB_DESC(1, 0x44, "rx_Pkts1519tomaxOctets"),
	MIB_DESC(1, 0x40, "rxMacDiscards")
};

u32 rtl838x_rtl8231_read(u32 phy, u32 reg)
{
	/* Calculate read register address */
	u32 t = (phy << 2) & (0x1f << 2);
	t |= (reg << 7) & (0x1f << 7);
	
	/* Set execution bit: cleared when operation completed */
	t |= 1;
	sw_w32(t, RTL838X_EXT_GPIO_INDRT_ACCESS);
	do {	/* TODO: Return 0x80000000 if timeout */
		t = sw_r32(RTL838X_EXT_GPIO_INDRT_ACCESS);
	} while (t & 1);
	
	return (t & 0xffff0000) >> 16;
}

int rtl838x_rtl8231_write(u32 phy, u32 reg, u32 data)
{
	u32 t = (phy << 2) & (0x1f << 2);
	printk("rtl838x_rtl8231_write: %x, %x, %x\n", phy, reg, data);
	t |= (reg << 7) & (0x1f << 7);
	/* Set write bit */
	t |= 2;

	/* Set execution bit: cleared when operation completed */
	t |= 1;
	sw_w32(t, RTL838X_EXT_GPIO_INDRT_ACCESS);
	printk("AAA\n");
	do {	/* TODO: Return -1 if timeout */
		t = sw_r32(RTL838X_EXT_GPIO_INDRT_ACCESS);
	} while (t & 1);
	
	return 0;
}

int rtl8231_pin_set_direction(int phy, u32 gpio, u32 dir)
{
	/* dir 1: input
	 * dir 0: output
	 */
	
	 u32  v;
	 
	 if( gpio > 31 ) {
		 return -1;
		 printk("rtl8231_pin_set_direction: GPIO >= 32 not implemented!");
	 }
	 
	/* GPIO Selection */
	v = rtl838x_rtl8231_read(phy, RTL8231_GPIO_IO_SEL(gpio));
	if (v & 0x80000000) {
		printk("Error reading RTL8231\n");
		return -1;
	}
	rtl838x_rtl8231_write(phy, RTL8231_GPIO_PIN_SEL(gpio), v | (1 << (gpio % 16)));

	
	v = rtl838x_rtl8231_read(phy, RTL8231_GPIO_IO_SEL(gpio));
	if (v & 0x80000000) {
		printk("Error reading RTL8231\n");
		return -1;
	}
	rtl838x_rtl8231_write(phy, RTL8231_GPIO_IO_SEL(gpio),
				(v & ~(1 << (gpio % 16))) | (dir << (gpio % 16)));
	return 0;
}

int rtl8231_pin_set(u32 phy, u32 gpio, u32 data)
{
	u32 v = rtl838x_rtl8231_read(phy, RTL8231_GPIO_CTRL(gpio));
	if (v & 0x80000000) {
		printk("Error reading RTL8231\n");
		return -1;
	}
	rtl838x_rtl8231_write(phy, RTL8231_GPIO_CTRL(gpio), 
			      (v & ~(1 << (gpio % 16))) | (data << (gpio % 16)));
	return 0;
}


/* Needed for the Zyxel */
int rtl8231_init(void)
{
	uint32_t v;
	printk("rtl8231_init\n");
	/* Enable RTL8231 indirect access mode */
	sw_w32_mask(0, 1, RTL838X_EXTRA_GPIO_CTRL);
	sw_w32_mask(3, 1, RTL838X_DMY_REG5);

	printk("1\n");
	
	/* Reset RTL8231 via GPIO_A1 line */
	sw_w32_mask(0, 1 << RTL838X_GPIO_A1, RTL838X_GPIO_PABC_DIR);
	sw_w32_mask(0, 1 << RTL838X_GPIO_A1, RTL838X_GPIO_PABC_DATA);

	printk("2\n");
	/* wait 50ms for reset */
	udelay(50000);
	
	/*Select GPIO*/
	rtl838x_rtl8231_write(0, RTL8231_PIN_SEL_REG, 0xffff);
	rtl838x_rtl8231_write(0, RTL8231_PIN_SEL_REG + 1, 0xffff);

	printk("3\n");
	/* Set 8231 GPIOs [0-36] to GPI */
	rtl838x_rtl8231_write(0, 0x5, 0xffff);
	rtl838x_rtl8231_write(0, 0x6, 0xffff);
	printk("4\n");
	v = rtl838x_rtl8231_read(0, 0x4);
	if (v & 0x80000000) {
		printk("Error reading RTL8231\n");
		return -1;
	}
	rtl838x_rtl8231_write(0, 0x4, v | (0x1f << 5));

	printk("5\n");
	/* 8231 PINs 5/6/8 to output*/
	rtl8231_pin_set_direction(0, 5, 0);
	rtl8231_pin_set_direction(0, 6, 0);
	rtl8231_pin_set_direction(0, 8, 0);

	printk("6\n");
	/* PINs 5/6/8 output high */
	rtl8231_pin_set(0, 5, 1);
	rtl8231_pin_set(0, 6, 1);
	rtl8231_pin_set(0, 8, 1);

	/* Enable RTL8231 */
	v = rtl838x_rtl8231_read(0, RTL8231_LED_FUNC0);
	if (v & 0x80000000) {
		printk("Error reading RTL8231\n");
		return -1;
	}
	printk("7\n");
	rtl838x_rtl8231_write(0, RTL8231_LED_FUNC0, v | 0x2);
	printk("rtl8231_init done\n");
	return 0;
}

static void rtl8380_get_version(struct rtl838x_priv *priv)
{
    u32 rw_save, info_save;
    u32 info;

    rw_save = sw_r32(RTL838X_INT_RW_CTRL);
    sw_w32(rw_save | 0x3, RTL838X_INT_RW_CTRL);

    info_save = sw_r32(RTL838X_CHIP_INFO);
    sw_w32(info_save | 0xA0000000, RTL838X_CHIP_INFO);

    info = sw_r32(RTL838X_CHIP_INFO);
    sw_w32(info_save, RTL838X_CHIP_INFO);
    sw_w32(rw_save, RTL838X_INT_RW_CTRL);

    if ((info & 0xFFFF) == 0x6275) {
        if(((info >> 16) & 0x1F) == 0x1) {
            priv->version = RTL8380_VERSION_A; 
	} else if(((info >> 16) & 0x1F) == 0x2) {
            priv->version = RTL8380_VERSION_B;
        } else {
            priv->version = RTL8380_VERSION_B;
 	}
    } else {
            priv->version = '-';
    }
}

int rtl838x_phy_wait_op(int timeout)
{
	do {
		timeout--;
		udelay(10);
	} while ((sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & 0x1) && (timeout >= 0));
	if (timeout <= 0) {
		printk("TIMEOUT\n");
		return -1;
	}
	return 0;
}

/* 
 * Write to a register in a page of the PHY
 */
int rtl838x_write_phy(u32 port, u32 page, u32 reg, u32 val)
{
	u32 v;
	u32 park_page;
	u32 offset = 0;

	if (port > 27 || page > 4095 || reg > 31)
		return -1;
	
	val &= 0xffff;

	if (port == 24 || port == 26) {
		if (port == 26)
			offset = 0x100;
		sw_w32(val, MAPLE_SDS4_FIB_REG0r + offset + (reg << 2));
		return 0;
	}

	rtl838x_phy_wait_op(10000);

	sw_w32(1 << port, RTL838X_SMI_ACCESS_PHY_CTRL_0);
	udelay(10000);

	sw_w32_mask(0xffff0000, val << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);

	park_page = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & ((0x1f << 15) | 0x2);
	v = reg << 20 | page << 3 | 0x4;
	sw_w32(v | park_page, RTL838X_SMI_ACCESS_PHY_CTRL_1);
	sw_w32_mask(0, 1, RTL838X_SMI_ACCESS_PHY_CTRL_1);
    
	rtl838x_phy_wait_op(10000);
		
	return 0;
}

/* 
 * Reads a register in a page from the PHY
 */
int rtl838x_read_phy(u32 port, u32 page, u32 reg, u32 *val)
{
/*	u32 vport;*/
	u32 v;
/*	volatile void *addr;*/
	u32 offset = 0;
	u32 park_page;
	
//	printk("PHY-read: port %d reg: %x\n", port, reg);
	
	
	if (port > 27 || page > 4095 || reg > 31)
		return -1;
	
	if (port == 24 || port == 26) {
		if (port == 26)
			offset = 0x100;
		*val = sw_r32(MAPLE_SDS4_FIB_REG0r + offset + (reg << 2)) & 0xffff;
//		printk("PHY-read: port %d reg: %x res: %x\n", port, reg, *val);
		return 0;
	}
	
	rtl838x_phy_wait_op(10000);
	/* take remapped port into account
	addr = ((port / 6) << 2) + RTL838X_SMI_PORT0_5_ADDR_CTRL;
	vport =  sw_r32(addr);
	port = ( vport >> (5 * (port % 6))) & 0x1F;
	*/
	
	sw_w32_mask(0xffff0000, port << 16, RTL838X_SMI_ACCESS_PHY_CTRL_2);
	
	park_page = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_1) & ((0x1f << 15) | 0x2);
	v = reg << 20 | page << 3;
	sw_w32(v | park_page, RTL838X_SMI_ACCESS_PHY_CTRL_1);
	sw_w32_mask(0, 1, RTL838X_SMI_ACCESS_PHY_CTRL_1);
	
    
	rtl838x_phy_wait_op(10000);
	
	*val = sw_r32(RTL838X_SMI_ACCESS_PHY_CTRL_2) & 0xffff;
//	printk("PHY-read: port %d reg: %x res: %x\n", port, reg, *val);
	return 0;
}

int dsa_phy_read(struct dsa_switch *ds, int phy_addr, int phy_reg)
{
	u32 val;
	
//	printk("DSA: PHY-read: phy %d reg: %x\n", phy_addr, phy_reg);
	rtl838x_read_phy(phy_addr, 0, phy_reg, &val);
//	printk("Result: %x\n", val);
	return val;
}

int dsa_phy_write(struct dsa_switch *ds, int phy_addr, int phy_reg, u16 val)
{
//	printk("DSA: PHY-write: phy %d %x %x\n", phy_addr, phy_reg, val);

	return rtl838x_write_phy(phy_addr, 0, phy_reg, val);
}

u32 rtl838x_6275B_intPhy_perport[][2] = {
	{0x1f, 0x0b82}, {0x10, 0x0000},
	{0x1f, 0x0a44}, {0x11, 0x0418}, {0x1f, 0x0bc0}, {0x16, 0x0c00},
	{0x1f, 0x0000}, {0x1b, 0x809a}, {0x1c, 0x8933}, {0x1b, 0x80a3},
	{0x1c, 0x9233}, {0x1b, 0x80ac}, {0x1c, 0xa444}, {0x1b, 0x809f},
	{0x1c, 0x6b20}, {0x1b, 0x80a8}, {0x1c, 0x6b22}, {0x1b, 0x80b1},
	{0x1c, 0x6b23}, {0, 0}
};

u32 rtl8218b_6276B_hwEsd_perport[][2] = {
    {0x1f, 0xbc4}, {0x17, 0xa200}, {0, 0}
};

static void rtl8380_int_phy_toggle(int mac, bool on)
{
	u32 val;
	rtl838x_read_phy(mac, 0, 0, &val);
	if (on)
		rtl838x_write_phy(mac, 0, 0, val & ~(1 << 11));
	else 
		rtl838x_write_phy(mac, 0, 0, val | (1 << 11));
}

static void rtl8380_int_phy_reset(int mac)
{
	u32  val;
	rtl838x_read_phy(mac, 0, 0, &val);
	rtl838x_write_phy(mac, 0, 0, val | (0x1 << 15));
}

int rtl8380_sds_power(int mac, int val)
{
	u32 mode = (val == 1)? 0x4: 0x9;
	u32 offset = (mac == 24)? 5: 0;

	if ( (mac != 24) && (mac != 26) ) {
		printk("rtl8380_sds_power: not a fibre port: %d\n", mac);
		return -1;
	}

	sw_w32_mask(0x1f << offset, mode << offset, RTL838X_SDS_MODE_SEL);
       
	/* Serdes reset */
	offset = (mac == 24)? 0: 0x100;
	sw_w32_mask(1 << 11, 0, (volatile void *) (0xbb00f800 + offset));
	sw_w32_mask(0x3, 0, RTL838X_SDS4_REG28 + offset);
	sw_w32_mask(0x3, 0x3, RTL838X_SDS4_REG28 + offset);
	sw_w32_mask(0, 0x1 << 6, RTL838X_SDS4_DUMMY0 + offset);
	sw_w32_mask(0x1 << 6, 0, RTL838X_SDS4_DUMMY0 + offset);
	return 0;
}

static int rtl8380_configure_phy(int mac)
{
	int ipd_flag = 0;
	int p, i;
	u32 val;
	
	/* Enable PHY control via SoC */
	sw_w32_mask(0, 1 << 15, RTL838X_SMI_GLB_CTRL);
	rtl838x_read_phy(mac, 0, 3, &val);
	printk("Phy on MAC %d: %x\n", mac, val);
	rtl838x_write_phy(mac, 31, 27, 0x0002);
	rtl838x_read_phy(mac, 31, 28, &val);
	printk("PHY ID: %x\n", val);
	if (val == 0x6275 ) { /* Internal RTL8218B, version 2 */
		printk("Found internal RTL8218B\n");
		if( sw_r32(RTL838X_DMY_REG31) == 0x1 )
			ipd_flag = 1;
		printk("IPD: %d\n", ipd_flag);
		rtl838x_read_phy(mac, 0, 0, &val);
		printk("PHY status: %4x\n", val);
		if ( val & (1 << 11) ) {
			printk("Enabling PHY %d\n", mac);
			rtl8380_int_phy_toggle(mac, true);
		} else {
			printk("Resetting PHY %d\n", mac);
			rtl8380_int_phy_reset(mac);
			msleep(100);
		}
		/* Ready PHY for patch */
		for (p = 0; p < 8; p++) {
			rtl838x_write_phy(mac + p, 0xfff, 0x1f, 0x0b82);
			rtl838x_write_phy(mac + p, 0xfff, 0x10, 0x0010);
		}
		msleep(500);
		for (p = 0; p < 8; p++) {
			for (i = 0; i < 100 ; i++) {
				rtl838x_read_phy(mac + p, 0x0b80, 0x10, &val);
				if (val & 0x40) break;
			}
			if (i >= 100) {
				printk("ERROR: Port %d not ready\n", p);
				return -1;
			}
		}
		for (p = 0; p < 8; p++) {
			i = 0;
			while (rtl838x_6275B_intPhy_perport[i][0]) {
				rtl838x_write_phy(mac + p, 0xfff,
					rtl838x_6275B_intPhy_perport[i][0],
					rtl838x_6275B_intPhy_perport[i][1]);
				i++;
			}
			while (rtl8218b_6276B_hwEsd_perport[i][0]) {
				rtl838x_write_phy(mac + p, 0xfff,
					rtl8218b_6276B_hwEsd_perport[i][0],
					rtl8218b_6276B_hwEsd_perport[i][1]);
				i++;
			}
		}
		printk("PHY %d successfully patched\n", mac);
	}
	return 0;
}

static int rtl8380_enable_phy_polling(void)
{
	u32 v;
	printk("rtl8300_enable_phy_polling\n");
	msleep(1000);
	/* Enable 2 QSGMII ports and 8 GIG ports: */
	v = 0x3000000 | 0x0000FF00;
	sw_w32(v, RTL838X_SMI_POLL_CTRL);
	/* PHY update complete */
	sw_w32_mask(0, 0x8000, RTL838X_SMI_GLB_CTRL);
	return 0;
}

static u32 rtl8380_sds_take_reset[][2] = {
	{0xbb000034, 0x0000003f}, {0xbb00003c, 0x00000010}, {0xbb00e78c, 0x00007146},
	{0xbb00e98c, 0x00007146}, {0xbb00eb8c, 0x00007146}, {0xbb00ed8c, 0x00007146},
	{0xbb00ef8c, 0x00007146}, {0xbb00f18c, 0x00007146}, {0, 0}
};

static u32 rtl8380_sds_common[][2] = {
	{0xbb00f878, 0x0000071e}, {0xbb00f978, 0x0000071e}, {0xbb00e784, 0x00000F00},
	{0xbb00e984, 0x00000F00}, {0xbb00eb84, 0x00000F00}, {0xbb00ed84, 0x00000F00},
	{0xbb00ef84, 0x00000F00}, {0xbb00f184, 0x00000F00}, {0xbb00e788, 0x00007060},
	{0xbb00e988, 0x00007060}, {0xbb00eb88, 0x00007060}, {0xbb00ed88, 0x00007060},
	{0xbb00ef88, 0x00007060}, {0xbb00f188, 0x00007060}, {0xbb00ef90, 0x0000074d},
	{0xbb00f190, 0x0000074d}, {0, 0}
};

static u32 rtl8380_sds01_qsgmii_6275b[][2] = {
	{0xbb00f38c, 0x0000f46f}, {0xbb00f388, 0x000085fa}, {0xbb00f488, 0x000085fa},
	{0xbb00f398, 0x000020d8}, {0xbb00f498, 0x000020d8}, {0xbb00f3c4, 0x0000B7C9},
	{0xbb00f4ac, 0x00000482}, {0xbb00f4a8, 0x000080c7}, {0xbb00f3c8, 0x0000ab8e},
	{0xbb00f3ac, 0x00000482}, {0xbb00f3cc, 0x000024ab}, {0xbb00f4c4, 0x00004208},
	{0xbb00f4c8, 0x0000c208}, {0xbb00f464, 0x00000303}, {0xbb00f564, 0x00000303},
	{0xbb00f3b8, 0x0000FCC2}, {0xbb00f4b8, 0x0000FCC2}, {0xbb00f3a4, 0x00008e64},
	{0xbb00f3a4, 0x00008c64}, {0xbb00f4a4, 0x00008e64}, {0xbb00f4a4, 0x00008c64},
	{0, 0}
  };

static u32 rtl8380_sds23_qsgmii_6275b[][2] = {
	{0xbb00f58c, 0x0000f46d}, {0xbb00f588, 0x000085fa}, {0xbb00f688, 0x000085fa},
	{0xbb00f788, 0x000085fa}, {0xbb00f598, 0x000020d8}, {0xbb00f698, 0x000020d8},
	{0xbb00f5c4, 0x0000B7C9}, {0xbb00f5c8, 0x0000ab8e}, {0xbb00f5ac, 0x00000482},
	{0xbb00f6ac, 0x00000482}, {0xbb00f5cc, 0x000024ab}, {0xbb00f6c4, 0x00004208},
	{0xbb00f6c8, 0x0000c208}, {0xbb00f664, 0x00000303}, {0xbb00f764, 0x00000303},
	{0xbb00f5b8, 0x0000FCC2}, {0xbb00f6b8, 0x0000FCC2}, {0xbb00f5a4, 0x00008e64},
	{0xbb00f5a4, 0x00008c64}, {0xbb00f6a4, 0x00008e64}, {0xbb00f6a4, 0x00008c64},
	{0, 0}
 };

static u32 rtl8380_sds4_fiber_6275b[][2] = {
	{0xbb00f788, 0x000085fa}, {0xbb00f7ac, 0x00001482}, {0xbb00f798, 0x000020d8},
	{0xbb00f7a8, 0x000000c3}, {0xbb00f7c4, 0x0000B7C9}, {0xbb00f7c8, 0x0000ab8e},
	{0xbb00f864, 0x00000303}, {0xbb00f7b8, 0x0000FCC2}, {0xbb00f7a4, 0x00008e64},
	{0xbb00f7a4, 0x00008c64}, {0, 0}
};

static u32 rtl8380_sds5_fiber_6275b[][2] = {
	{0xbb00f888, 0x000085fa}, {0xbb00f88c, 0x00000000}, {0xbb00f890, 0x0000dccc},
	{0xbb00f894, 0x00000000}, {0xbb00f898, 0x00003600}, {0xbb00f89c, 0x00000003},
	{0xbb00f8a0, 0x000079aa}, {0xbb00f8a4, 0x00008c64}, {0xbb00f8a8, 0x000000c3},
	{0xbb00f8ac, 0x00001482}, {0xbb00f960, 0x000014aa}, {0xbb00f964, 0x00000303},
	{0xbb00f8b8, 0x0000f002}, {0xbb00f96c, 0x000004bf}, {0xbb00f8a4, 0x00008e64},
	{0xbb00f8a4, 0x00008c64}, {0, 0}
};

static u32 rtl8380_sds_reset[][2] = {
	{0xbb00e780, 0x00000c00}, {0xbb00e980, 0x00000c00}, {0xbb00eb80, 0x00000c00},
	{0xbb00ed80, 0x00000c00}, {0xbb00ef80, 0x00000c00}, {0xbb00f180, 0x00000c00},
	{0xbb00e780, 0x00000c03}, {0xbb00e980, 0x00000c03}, {0xbb00eb80, 0x00000c03},
	{0xbb00ed80, 0x00000c03}, {0xbb00ef80, 0x00000c03}, {0xbb00f180, 0x00000c03},
	{0, 0}
};

static u32 rtl8380_sds_release_reset[][2] = {
	{0xbb00e78c, 0x00007106}, {0xbb00e98c, 0x00007106}, {0xbb00eb8c, 0x00007106},
	{0xbb00ed8c, 0x00007106}, {0xbb00ef8c, 0x00007106}, {0xbb00f18c, 0x00007106},
	{0, 0}
};

static void rtl8380_configure_serdes(void)
{
	u32 v;
	u32 sds_conf_value;
	int i;

	/* Back up serdes power down value */
	sds_conf_value = sw_r32(RTL838X_SDS_CFG_REG);
	printk("SDS power down value: %x\n", sds_conf_value);

	/* take serdes into reset */
	i = 0;
	while (rtl8380_sds_take_reset[i][0]) {
		sw_w32(rtl8380_sds_take_reset[i][1], 
			(volatile void *)rtl8380_sds_take_reset[i][0]);
		i++;
		udelay(1000);
	}
	
	/* apply common serdes patch */
	i = 0;
	while (rtl8380_sds_common[i][0]) {
		sw_w32(rtl8380_sds_common[i][1],
			(volatile void *)rtl8380_sds_common[i][0]);
		i++;
		udelay(1000);
	}
	
	/* internal R/W enable */
	sw_w32(3, RTL838X_INT_RW_CTRL);
	
	/* SerDes ports 4 and 5 are FIBRE ports */
	sw_w32_mask(0x7 | 0x38, 1 | (1 << 3), RTL838X_INT_MODE_CTRL);

	/* SerDes module settings, SerDes 0-3 are QSGMII */
	v = 0x6 << 25 | 0x6 << 20 | 0x6 << 15 | 0x6 << 10;
	/* SerDes 4 and 5 are 1000BX FIBRE */
	v |= 0x4 << 5 | 0x4 ;
	sw_w32(v, RTL838X_SDS_MODE_SEL);

	printk("FF8 register: %x\n", sw_r32((volatile void *) 0xbb000FF8));
	sw_w32_mask(0xfffffff0, 0xaaaaaaaf & 0xf, (volatile void *) 0xbb000FF8);
	i = 0;
	while (rtl8380_sds01_qsgmii_6275b[i][0]) {
		sw_w32( rtl8380_sds01_qsgmii_6275b[i][1], (volatile void *) rtl8380_sds01_qsgmii_6275b[i][0]);
		i++;
	}
	
	i = 0;
	while (rtl8380_sds23_qsgmii_6275b[i][0]) {
		sw_w32(rtl8380_sds23_qsgmii_6275b[i][1],
			(volatile void *) rtl8380_sds23_qsgmii_6275b[i][0]);
		i++;
	}
	
	i = 0;
	while (rtl8380_sds4_fiber_6275b[i][0]) {
		sw_w32(rtl8380_sds4_fiber_6275b[i][1],
			(volatile void *) rtl8380_sds4_fiber_6275b[i][0]);
		i++;
	}
	
	i = 0;
	while (rtl8380_sds5_fiber_6275b[i][0]) {
		sw_w32(rtl8380_sds5_fiber_6275b[i][1],
			(volatile void *) rtl8380_sds5_fiber_6275b[i][0]);
		i++;
	}
	
	i = 0;
	while (rtl8380_sds_reset[i][0]) {
		sw_w32(rtl8380_sds_reset[i][1],
			(volatile void *) rtl8380_sds_reset[i][0]);
		i++;
	}
	
	i = 0;
	while (rtl8380_sds_release_reset[i][0]) {
		sw_w32(rtl8380_sds_release_reset[i][1],
			(volatile void *) rtl8380_sds_release_reset[i][0]);
		i++;
	}

	printk("SDS power down value now: %x\n", sw_r32(RTL838X_SDS_CFG_REG));
	sw_w32(sds_conf_value, RTL838X_SDS_CFG_REG);
	
	/* Fibre port power off, in order to disable LEDs */
/*	sw_w32_mask( 0, 1 << 11, (volatile void *)0xbb00f800); 
	sw_w32_mask( 0, 1 << 11, (volatile void *)0xbb00f900);*/
	
}

static int rtl8380_configure_mac(struct rtl838x_priv *priv)
{
	int i;
	/* fix timer for EEE */
	sw_w32(0x5001411, (volatile void *)0xbb00AA04);
	sw_w32(0x5001417, (volatile void *)0xbb00AA08);
	
	if (priv->id == 0x8382) {
		for (i = 0; i <= 28; i++)
			sw_w32(0, (volatile void *)0xBB00d57c + i * 0x80);
	}
	if (priv->id == 0x8380) {
		for (i = 8; i <= 28; i++)
			sw_w32(0, (volatile void *)0xBB00d57c + i * 0x80);
	}
	return 0;
}

void print_matrix(void)
{
	unsigned volatile int *ptr = RTL838X_SW_BASE + 0x4100;
	printk("> %8x %8x %8x %8x %8x %8x %8x %8x\n", 
	       ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]);
	printk("> %8x %8x %8x %8x %8x %8x %8x %8x\n", 
	       ptr[8], ptr[9], ptr[10], ptr[11], ptr[12], ptr[13], ptr[14], ptr[15]);
	printk("> %8x %8x %8x %8x %8x %8x %8x %8x\n", 
	       ptr[16], ptr[17], ptr[18], ptr[19], ptr[20], ptr[21], ptr[22], ptr[23]);
	printk("> %8x %8x %8x %8x %8x\n", ptr[24], ptr[25], ptr[26], ptr[27], ptr[28]);
}

static int rtl838x_get_port_bit(int port)
{
/*	if (port == 9 )
		return 28;
	if (port == 10)
		return 29;
	
	if (port < 0 || port >8)
		return -1;
	return 8 + port;*/
	if (port == 28 )
		return 28;
	if (port == 26)
		return 26;
	if (port == 24)
		return 24;
	
	if (port < 8 || port > 15) {
		printk("No such port %d\n", port);
		return -1;
	}
	return port;
}

static int rtl838x_setup(struct dsa_switch *ds)
{
	int i;
	u32 port_bitmap = 0;
	struct rtl838x_priv *priv = ds->priv;
	
	/* Power on fibre ports and reset them if necessary */
	printk("Powering on fibre ports & reset\n");
	rtl8380_sds_power(24, 1);
	rtl8380_sds_power(26, 1);
	
	// The following crashes. Add more print statements
//	rtl8231_init(); /* for Zyxel */
	
	/* Disable MAC polling the PHY so that we can start configuration */
	sw_w32(0x00000000, RTL838X_SMI_POLL_CTRL);

	rtl8380_configure_mac(priv);

	rtl8380_configure_serdes();

	rtl8380_configure_phy(8);

	for (i = 0; i < ds->num_ports; i++)
		priv->ports[i].enable = false;
	priv->ports[CPU_PORT].enable = true;
	
	/* Isolate ports from each other: traffic only CPU <-> port */
	for (i = 8; i < 16; i++) {
		sw_w32(1 << CPU_PORT, RTL838X_PORT_ISO_CTRL(i));
		sw_w32_mask(0, 1 << i, RTL838X_PORT_ISO_CTRL(i));
		port_bitmap |= 1 << i;
	}
	sw_w32(1 << CPU_PORT | 1 << 24, RTL838X_PORT_ISO_CTRL(24));
	sw_w32(1 << CPU_PORT | 1 << 26, RTL838X_PORT_ISO_CTRL(26));
	port_bitmap |= (1 << 24) | (1 << 26);
	sw_w32(port_bitmap, RTL838X_PORT_ISO_CTRL(CPU_PORT));
	
	print_matrix();
	
	/* Enable MAC Polling PHY again */
	rtl8380_enable_phy_polling();
	printk("Please wait until PHY is settled\n");
	msleep(1000);

	return 0;
}

static void rtl838x_get_strings(struct dsa_switch *ds, 
				int port, u32 stringset, u8 *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(rtl838x_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, rtl838x_mib[i].name,
			ETH_GSTRING_LEN);
}

static void rtl838x_get_ethtool_stats(struct dsa_switch *ds, int port,
				      uint64_t *data)
{
	const struct rtl838x_mib_desc *mib;
	int i;
	u64 high;

	for (i = 0; i < ARRAY_SIZE(rtl838x_mib); i++) {
		mib = &rtl838x_mib[i];

		data[i] = sw_r32(RTL838X_STAT_PORT_STD_MIB(port) 
							+ (mib->offset >> 2) );
		if (mib->size == 2) {
			high = sw_r32(RTL838X_STAT_PORT_STD_MIB(port) 
							+ (mib->offset >> 2) + 1);
			data[i] |= high << 32;
		}
	}
}

static int rtl838x_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(rtl838x_mib);
}

static enum dsa_tag_protocol 
rtl838x_get_tag_protocol(struct dsa_switch *ds, int port)
{
	/* The switch does not tag the frames, instead internally the header
	 * structure for each packet is tagged accordingly.
	 */
	return DSA_TAG_PROTO_TRAILER;
}

void rtl838x_adjust_link(struct dsa_switch *ds, int port,
			 struct phy_device *phydev)
{
	printk("rtl838x_adjust_link, port %d\n", port);
	return;
}

static int rtl838x_vlan_filtering(struct dsa_switch *ds, int port,
				  bool vlan_filtering)
{
	printk("rtl838x_port_vlan_filtering port %d\n", port);
	return 0;
}
	
static int rtl838x_vlan_prepare(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	return 0;
}

static void rtl838x_vlan_add(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	return;
}

static int rtl838x_vlan_del(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	return 0;
}

static void rtl838x_port_bridge_leave (struct dsa_switch *ds, int port,
				      struct net_device *bridge)
{
	struct rtl838x_priv *priv = ds->priv;
	u32 port_bitmap = 1 << rtl838x_get_port_bit(CPU_PORT);
	int i;

	printk("port_bridge_leave %x: %d", (u32)priv, port);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Remove this port from the port matrix of the other ports
		 * in the same bridge. If the port is disabled, port matrix
		 * is kept and not being setup until the port becomes enabled.
		 * And the other port's port matrix cannot be broken when the
		 * other port is still a VLAN-aware port. */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable) {
				sw_w32_mask(1 << port, 0, RTL838X_PORT_ISO_CTRL(i));
			}
			priv->ports[i].pm |= 1 << rtl838x_get_port_bit(port);

			port_bitmap &= ~(1 << rtl838x_get_port_bit(i));
		}
	}

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		sw_w32_mask(0, port_bitmap, RTL838X_PORT_ISO_CTRL(port));
	}
	priv->ports[port].pm &= ~port_bitmap;
	mutex_unlock(&priv->reg_mutex);
	
	print_matrix();
	return;
}

static int rtl838x_port_bridge_join(struct dsa_switch *ds, int port,
				      struct net_device *bridge)
{
	struct rtl838x_priv *priv = ds->priv;
	u32 port_bitmap = 1 << rtl838x_get_port_bit(CPU_PORT);
	int i;

	printk("port_bridge_join %x: %d %x", (u32)priv, port, port_bitmap);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge. If the port is disabled, port matrix is kept
		 * and not being setup until the port becomes enabled.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable) {
				sw_w32_mask(0, 1 << port, RTL838X_PORT_ISO_CTRL(i));
			}
			priv->ports[i].pm |= 1 << rtl838x_get_port_bit(port);

			port_bitmap |= 1 << rtl838x_get_port_bit(i);
//			printk("pm: %x, port_bitmap: %x\n",  priv->ports[i].pm, port_bitmap);
		}
	}

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		sw_w32_mask(0, 1 << port, RTL838X_PORT_ISO_CTRL(CPU_PORT));
		sw_w32_mask(0, port_bitmap, RTL838X_PORT_ISO_CTRL(port));
	}
	priv->ports[port].pm |= port_bitmap;
	mutex_unlock(&priv->reg_mutex);
	
	print_matrix();
	return 0;
}

static int rtl838x_port_enable(struct dsa_switch *ds, int port,
				struct phy_device *phydev)
{
	int bit;
	struct rtl838x_priv *priv = ds->priv;
	printk("rtl838x_port_enable: %x %d", (u32) priv, port);
	priv->ports[port].enable = true;
	
	if (dsa_is_cpu_port(ds, port))
		return 0;
	
	/* add port to switch mask of CPU_PORT */
	sw_w32_mask(0, 1 << port, RTL838X_PORT_ISO_CTRL(CPU_PORT));
	
	/* enable PHY polling */
	bit = rtl838x_get_port_bit(port);
	sw_w32_mask(0, 1 << bit, RTL838X_SMI_POLL_CTRL);
	
	return 0;
}

static void rtl838x_port_disable(struct dsa_switch *ds, int port,
				 struct phy_device *phydev)
{
	int bit;
	struct rtl838x_priv *priv = ds->priv;
	printk("rtl838x_port_disable %x: %d", (u32)priv, port);

	/* you can only disable user ports */
	if (!dsa_is_user_port(ds, port))
		return;
	
	/* remove port from switch mask of CPU_PORT */
	sw_w32_mask(1 << port, 0, RTL838X_PORT_ISO_CTRL(CPU_PORT));
	
	priv->ports[port].enable = false;
	
	/* disable PHY polling */
	bit = rtl838x_get_port_bit(port);
	sw_w32_mask(1 << bit, 0, RTL838X_SMI_POLL_CTRL);
	
	return;
}

static const struct dsa_switch_ops rtl838x_switch_ops = {
	.get_tag_protocol = rtl838x_get_tag_protocol,
	.setup = rtl838x_setup,
	.adjust_link = rtl838x_adjust_link,
	.port_vlan_filtering = rtl838x_vlan_filtering,
	.port_vlan_prepare = rtl838x_vlan_prepare,
	.port_vlan_add = rtl838x_vlan_add,
	.port_vlan_del = rtl838x_vlan_del,
	.port_bridge_join = rtl838x_port_bridge_join,
	.port_bridge_leave = rtl838x_port_bridge_leave,
	.port_enable = rtl838x_port_enable,
	.port_disable = rtl838x_port_disable,
	.phy_read = dsa_phy_read,
	.phy_write = dsa_phy_write,
	.get_strings = rtl838x_get_strings,
	.get_ethtool_stats = rtl838x_get_ethtool_stats,
	.get_sset_count = rtl838x_get_sset_count,
};

static int __init rtl838x_sw_probe(struct platform_device *pdev)
{
	int err = 0;
	struct rtl838x_priv *priv;
	struct device *dev = &pdev->dev;

	printk("Probing RTL838X switch device\n");
	// dump_stack();
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* TODO: Kernel 5.4
	 * priv->ds = devm_kzalloc(dev, sizeof(*priv->ds), GFP_KERNEL); */
	priv->ds = dsa_switch_alloc(dev, DSA_MAX_PORTS);

	if (!priv->ds)
		return -ENOMEM;
	priv->ds->dev = dev;
	priv->ds->priv = priv;
	priv->ds->ops = &rtl838x_switch_ops;
	priv->dev = dev;
	priv->base_port = 8;
	priv->ds->num_ports = 30;
	priv->id = sw_r32(RTL838X_MODEL_NAME_INFO) >> 16;
	
	switch (priv->id) {
	case 0x8380:
	printk("Found RTL8380M\n");
		break;
	case 0x8382:
		printk("Found RTL8382M\n");
		break;
	default:
		printk("Unknown chip id (%04x)\n", priv->id);
		return -ENODEV;
	}
	rtl8380_get_version(priv);
	printk("Chip version %c\n", priv->version);
		
	err = dsa_register_switch(priv->ds);
	if(err)
		printk("Error registering switch: %d\n", err);

	return err;
}


static int rtl838x_sw_remove(struct platform_device *pdev)
{
	printk("Removing platform driver for rtl838x-sw\n");
	return 0;
}

static const struct of_device_id rtl838x_switch_of_ids[] = {
	{ .compatible = "realtek,rtl838x-switch"},
	{ /* sentinel */ }
};


MODULE_DEVICE_TABLE(of, rtl838x_switch_of_ids);

static struct platform_driver rtl838x_switch_driver = {
	.probe = rtl838x_sw_probe,
	.remove = rtl838x_sw_remove,
	.driver = {
		.name = "rtl838x-switch",
		.pm = NULL,
		.of_match_table = rtl838x_switch_of_ids,
	},
};

module_platform_driver(rtl838x_switch_driver);

MODULE_AUTHOR("B. Koblitz");
MODULE_DESCRIPTION("RTL838X SoC Switch Driver");
MODULE_LICENSE("GPL");
