// SPDX-License-Identifier: GPL-2.0-only

#ifndef _RTL838X_PROM_H
#define _RTL838X_PROM_H

/* Definition family ID */
#define RTL8389_FAMILY_ID   (0x8389)
#define RTL8328_FAMILY_ID   (0x8328)
#define RTL8390_FAMILY_ID   (0x8390)
#define RTL8350_FAMILY_ID   (0x8350)
#define RTL8380_FAMILY_ID   (0x8380)
#define RTL8330_FAMILY_ID   (0x8330)

/* Definition SOC ID */
#define RTL8380_CHIP_INFO_ADDR		((volatile void *)(0xBB0000D8))
#define RTL8380_MODEL_NAME_INFO_ADDR	((volatile void *)(0xBB0000D4))


#define RTL8328_SOC_ID	    (0x8328)
#define RTL8380_SOC_ID	    (0x8380)
#define RTL8382_SOC_ID	    (0x8382)
#define RTL8389_SOC_ID	    (0x8389)
#define RTL8390_SOC_ID	    (0x8390)
#define RTL8391_SOC_ID	    (0x8391)
#define RTL8392_SOC_ID	    (0x8392)


/* Definition chip CPU Freq reg (0xBB00 switch part)*/
#define RTL839x5x_CPU_PLL0		(0x0028)
#define RTL839x5x_CPU_PLL1		(0x002C)
#define RTL838x3x_CPU_READ_CTL		(0x0058)
#define RTL838x3x_CPU_CTL0		(0x0FC4)
#define RTL838x3x_CPU_CTL1		(0x0FC8)
#define RTL838x3x_CPU_MISC		(0x0FCC)


/* Definition chip CPU Freq */
#define RTL839x5x_CPU_750M_PLL0     (0x000041AC)
#define RTL839x5x_CPU_750M_PLL1		(0x00000004)
#define RTL839x5x_CPU_700M_PLL0     (0x0000418C)
#define RTL839x5x_CPU_700M_PLL1		(0x00000004)
#define RTL839x5x_CPU_650M_PLL0     (0x0000416C)
#define RTL839x5x_CPU_650M_PLL1		(0x00000004)
#define RTL839x5x_CPU_500M_PLL0     (0x000041AC)
#define RTL839x5x_CPU_500M_PLL1		(0x00000005)

#define RTL838x3x_CPU_500M_CTL0		(0x00004748)
#define RTL838x3x_CPU_500M_CTL1     (0x0C14530E)
#define RTL838x3x_CPU_500M_MISC		(0x2DA0CFB8)
#define RTL838x3x_CPU_300M_CTL0		(0x000045C8)
#define RTL838x3x_CPU_300M_CTL1     (0x1414530E)
#define RTL838x3x_CPU_300M_MISC		(0x2D20CFB8)

#define RTL838X_SW_BASE ((volatile void *)(0xBB000000))

/* register access macros for EBU and CGU */
#define ltq_sw_w32(x, y)	rtl838x_w32((x), rtl838x_sw_membase + (y))
#define ltq_sw_r32(x)		rtl838x_r32(rtl838x_sw_membase + (x))

struct rtl838x_soc_info {
	unsigned char *name;
	unsigned int id;
	unsigned int family;
	unsigned char *compatible;
};

void rtl838x_soc_detect(struct rtl838x_soc_info *i);

#endif   /* _RTL838X_PROM_H */
