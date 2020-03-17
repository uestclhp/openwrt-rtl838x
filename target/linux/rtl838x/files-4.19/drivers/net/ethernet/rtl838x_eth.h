// 

#ifndef _RTL838X_ETH_H
#define _RTL838X_ETH_H

#define RTL838X_SW_BASE ((volatile void *)0xBB000000)

#define sw_r32(reg)	__raw_readl(reg)
#define sw_w32(val, reg)	__raw_writel(val, reg)
#define sw_w32_mask(clear, set, reg)	\
	sw_w32((sw_r32(reg) & ~(clear)) | (set), reg)
/*
 * Register definition
 */
 
#define CPU_PORT				28
#define RTL838X_MAC_PORT_CTRL(port)		(RTL838X_SW_BASE + 0xd560 + (((port) << 7)))
#define RTL838X_DMA_IF_INTR_STS			(RTL838X_SW_BASE + 0x9f54)
#define RTL838X_DMA_IF_INTR_MSK			(RTL838X_SW_BASE + 0x9f50)
#define RTL838X_DMA_IF_CTRL			(RTL838X_SW_BASE + 0x9f58)
#define RTL838X_RST_GLB_CTRL_0			(RTL838X_SW_BASE + 0x3c)
#define RTL838X_MAC_FORCE_MODE_CTRL(port)	(RTL838X_SW_BASE + 0xa104 + (((port) << 2)))
#define RTL838X_MAC				(RTL838X_SW_BASE + 0xa9ec)
#define RTL838X_MAC_ALE				(RTL838X_SW_BASE + 0x6b04)
#define RTL838X_MAC2				(RTL838X_SW_BASE + 0xa320)
#define RTL838X_DMA_RX_BASE(idx)		(RTL838X_SW_BASE + 0x9f00 + (((idx) << 2)))
#define RTL838X_DMA_TX_BASE(idx)		(RTL838X_SW_BASE + 0x9f40 + (((idx) << 2)))
#define RTL838X_DMA_IF_RX_RING_SIZE(idx)	(RTL838X_SW_BASE + 0xB7E4 + (((idx >> 3) << 2)))
#define RTL838X_DMA_IF_RX_RING_CNTR(idx)	(RTL838X_SW_BASE + 0xB7E8 + (((idx >> 3) << 2)))
#define RTL838X_DMA_IF_RX_CUR(idx)		(RTL838X_SW_BASE + 0x9F20 + (((idx) << 2)))
#define RTL838X_SMI_GLB_CTRL			(RTL838X_SW_BASE + 0xa100)
#define RTL838X_SMI_ACCESS_PHY_CTRL_0		(RTL838X_SW_BASE + 0xa1b8)
#define RTL838X_SMI_ACCESS_PHY_CTRL_1		(RTL838X_SW_BASE + 0xa1bc)
#define RTL838X_SMI_ACCESS_PHY_CTRL_2		(RTL838X_SW_BASE + 0xa1c0)
#define RTL838X_SMI_PORT0_5_ADDR_CTRL		(RTL838X_SW_BASE + 0xa1c8)
#define RTL838X_DMY_REG31			(RTL838X_SW_BASE + 0x3b28)
#define RTL838X_SDS_MODE_SEL			(RTL838X_SW_BASE + 0x28)
#define RTL838X_SDS_CFG_REG			(RTL838X_SW_BASE + 0x34)
#define RTL838X_INT_RW_CTRL			(RTL838X_SW_BASE + 0x58)
#define RTL838X_INT_MODE_CTRL			(RTL838X_SW_BASE + 0x5c)
#define RTL838X_MODEL_NAME_INFO			(RTL838X_SW_BASE + 0xd4)
#define RTL838X_CHIP_INFO			(RTL838X_SW_BASE + 0xd8)
#define RTL838X_DMY_REG5			(RTL838X_SW_BASE + 0x144)
#define RTL838X_SDS4_REG28			(RTL838X_SW_BASE + 0xef80)
#define RTL838X_SDS4_DUMMY0			(RTL838X_SW_BASE + 0xef8c)
#define RTL838X_SDS5_EXT_REG6			(RTL838X_SW_BASE + 0xf18c)
#define RTL838X_SMI_PORT0_5_ADDR_CTRL		(RTL838X_SW_BASE + 0xa1c8)
#define RTL838X_EXT_GPIO_INDRT_ACCESS		(RTL838X_SW_BASE + 0xa09c)
#define RTL838X_EXTRA_GPIO_CTRL			(RTL838X_SW_BASE + 0xa0e0)
#define RTL838X_SMI_POLL_CTRL			(RTL838X_SW_BASE + 0xa17c)
#define RTL838X_PORT_ISO_CTRL(port)		(RTL838X_SW_BASE + 0x4100 + ((port) << 2))
#define RTL838X_STAT_PORT_STD_MIB(port)		(RTL838X_SW_BASE + 0x1200 + (((port) << 8)))

/* Registers of the internal Serdes of the 8380 */
#define MAPLE_SDS4_REG0r			RTL838X_SDS4_REG28
#define MAPLE_SDS5_REG0r			(RTL838X_SDS4_REG28 + 0x100)
#define MAPLE_SDS4_REG3r			RTL838X_SDS4_DUMMY0
#define MAPLE_SDS5_REG3r			(RTL838X_SDS4_REG28 + 0x100)
#define MAPLE_SDS4_FIB_REG0r			(RTL838X_SDS4_REG28 + 0x880)
#define MAPLE_SDS5_FIB_REG0r			(RTL838X_SDS4_REG28 + 0x980)

/* RTL8231 registers for LED control */
#define RTL8231_LED_FUNC0			0x0000
#define RTL8231_GPIO_PIN_SEL(gpio)		((0x0002) + ((gpio) >> 4))
#define RTL8231_GPIO_PIN_SEL2			(0x0004)
#define RTL8231_GPIO_IO_SEL(gpio)		((0x0005) + ((gpio) >> 4))
#define RTL8231_GPIO_IO_SEL0			(0x0005)
#define RTL8231_GPIO_IO_SEL1			(0x0006)
#define RTL8231_GPIO_CTRL(gpio)			((0x001C) + ((gpio) >> 4))
#define RTL8231_PIN_SEL_REG			0x2

#endif /* _RTL838X_ETH_H */
