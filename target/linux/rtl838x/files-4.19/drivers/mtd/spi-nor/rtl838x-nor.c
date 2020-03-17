// SPDX-License-Identifier: GPL-2.0-only


#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>

#include "rtl838x-spi.h"

struct rtl838x_nor {
	struct spi_nor nor;
	struct device *dev;
	void __iomem *base;	/* nor flash base address */
	
	// Following need to go into spi_nor->priv
	uint32_t flags;
	uint32_t io_status;
};

static uint32_t spi_prep(struct rtl838x_nor *nor)
{
	/* Needed because of MMU constraints */
	SPI_READY;
	spi_w32w(SPI_CS_INIT, SFCSR);	//deactivate CS0, CS1
	spi_w32w(0, SFCSR); 		//activate CS0,CS1
	spi_w32w(SPI_CS_INIT, SFCSR); 	//deactivate CS0, CS1

	return ( CS0 & nor->flags) ? (SPI_eCS0 & SPI_LEN_INIT) : ((SPI_eCS1 & SPI_LEN_INIT) | SFCSR_CHIP_SEL);
}

static uint32_t rtl838x_nor_get_SR(struct rtl838x_nor *nor)
{
	uint32_t sfcsr, sfdr;
	
	sfcsr = spi_prep(nor);
	sfdr  = (SPINOR_OP_RDSR)<<24;
	
	pr_debug("rtl838x_nor_get_SR: rdid,sfcsr_val = %.8x,SFDR = %.8x \n", sfcsr, sfdr);
	pr_debug("rdid,sfcsr = %.8x\n",sfcsr | SPI_LEN4);
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	spi_w32_mask(0, SPI_LEN4, SFCSR);
	SPI_READY;
	
	return spi_r32(SFDR);
}

static void spi_write_disable(struct rtl838x_nor *nor)
{
	uint32_t sfcsr, sfdr; 
	
	sfcsr = spi_prep(nor);
	sfdr = (SPINOR_OP_WRDI) << 24;
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
        printk("spi_write_disable: sfcsr_val = %.8x,SFDR = %.8x", sfcsr, sfdr);
	
	spi_prep(nor);
}

static void spi_write_enable(struct rtl838x_nor *nor)
{
	uint32_t sfcsr, sfdr; 
	
	sfcsr = spi_prep(nor);
	sfdr = (SPINOR_OP_WREN) << 24;
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
        printk("spi_write_enable: sfcsr_val = %.8x,SFDR = %.8x", sfcsr, sfdr);
	
	spi_prep(nor);
}

static ssize_t rtl838x_do_read(struct rtl838x_nor *nor,loff_t from, size_t length, u_char *buffer, uint8_t command)
{
	uint32_t sfcsr, sfdr;
	uint32_t len = length;
		
	sfcsr = spi_prep(nor);
	sfdr = command << 24;
	
	/* Perform SPINOR_OP_READ: 1 byte command & 3 byte addr*/
	sfcsr |= SPI_LEN4;
	sfdr |= from;
	
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	
	/* Read Data, 4 bytes at a time */
	while (length >= 4){
		SPI_READY;
		*((uint32_t*) buffer) = spi_r32(SFDR);
/*		printk("%.8x  ", *((uint32_t*) buffer)); */
		buffer += 4;
		length -= 4;
	}
	
	/* The rest needs to be read 1 byte a time */
	sfcsr &= SPI_LEN_INIT|SPI_LEN1;
	SPI_READY;
	spi_w32w(sfcsr, SFCSR);
	while (length > 0) {
		SPI_READY;
		*(buffer) = spi_r32(SFDR) >> 24;
/*	    	printk("%.2x  ", *(buffer)); */
		buffer++;
		length--;
	}
	return len;
}

static ssize_t rtl838x_do_write(struct rtl838x_nor *nor,loff_t to, size_t length, const u_char *buffer, uint8_t command)
{
	uint32_t sfcsr, sfdr;
	uint32_t len = length;
	
	printk("In rtl838x_do_write to %llx, length: %x, buffer: %x\n", to, length, (uint32_t)buffer);
	
	if (to < 0x930000) {
		printk("ERRORR\n");
		return 0;
	}
	sfcsr = spi_prep(nor);
	sfdr = command << 24;
	
	/* Perform SPI: 1 byte command & 3 byte addr*/
	sfcsr |= SPI_LEN4;
	sfdr |= to;
	
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	
	/* Read Data, 4 bytes at a time, if we are 4-byte aligned */
	if (!(((long)buffer) % 4) ){ 
		while (length >= 4){
			SPI_READY;
			spi_w32(*((uint32_t*)buffer), SFDR);
			buffer += 4;
			length -= 4;
		}
	}
	
	/* The rest needs to be read 1 byte a time */
	sfcsr &= SPI_LEN_INIT | SPI_LEN1;
	SPI_READY;
	spi_w32w(sfcsr, SFCSR);
	while (length > 0) {
		SPI_READY;
		spi_w32(*(buffer) << 24, SFDR);
		buffer++;
		length--;
	}
	return len;
}

static ssize_t rtl838x_nor_write(struct spi_nor *nor, loff_t to, size_t len,
			     const u_char *buffer)
{
	int ret = 0;
	uint32_t offset = 0;
	struct rtl838x_nor *rtl838x_nor = nor->priv;
	size_t l = len;
	uint8_t cmd = SPINOR_OP_PP;
	
	printk("In rtl838x_nor_write %8x to: %llx\n", (unsigned int) rtl838x_nor, to);
	
	if (to >= 0x930000) {
		printk("cmd is %d \n", cmd);
		while (l >= SPI_MAX_TRANSFER_SIZE) {
			printk("Waiting for write enable\n");
			while(rtl838x_nor_get_SR(rtl838x_nor) & SPI_WIP);
			do { spi_write_enable(rtl838x_nor); }
				while (!(rtl838x_nor_get_SR(rtl838x_nor) & SPI_WEL));

			ret = rtl838x_do_write(rtl838x_nor, to + offset, SPI_MAX_TRANSFER_SIZE, buffer+offset, cmd);
			l -= SPI_MAX_TRANSFER_SIZE;
			offset += SPI_MAX_TRANSFER_SIZE;
		}
	
		if (l > 0) {
			printk("Waiting for write enable\n");
			while(rtl838x_nor_get_SR(rtl838x_nor) & SPI_WIP);
			do { spi_write_enable(rtl838x_nor); }
				while (!(rtl838x_nor_get_SR(rtl838x_nor) & SPI_WEL));

			ret = rtl838x_do_write(rtl838x_nor, to+offset , len, buffer+offset, cmd);
		}
	}
	
	return len;
}

static ssize_t rtl838x_nor_read(struct spi_nor *nor, loff_t from, size_t length, u_char *buffer)
{
	uint32_t offset = 0;
	uint8_t cmd = 0;
	size_t l = length;
	struct rtl838x_nor *rtl838x_nor = nor->priv;

	/* TODO: do timeout and return error */
	pr_debug("Waiting for pending writes\n");
	while(rtl838x_nor_get_SR(rtl838x_nor) & SPI_WIP);
	
	cmd = SPINOR_OP_READ;
	pr_debug("cmd is %d \n", cmd);
	pr_debug("rtl838x_nor_read addr %.8llx to addr %.8x, cmd %.8x, size %d\n", from, (u32)buffer, (u32)cmd, length);
    
	while (l >= SPI_MAX_TRANSFER_SIZE) {
		rtl838x_do_read(rtl838x_nor, from + offset, SPI_MAX_TRANSFER_SIZE, buffer+offset, cmd);
		l -= SPI_MAX_TRANSFER_SIZE;
		offset += SPI_MAX_TRANSFER_SIZE;
	}
	
	if (l > 0)
		rtl838x_do_read(rtl838x_nor, from + offset, l, buffer+offset, cmd);
	
	return length;
}

static int rtl838x_nor_read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int length = len;
	u8 *buffer = buf;
	uint32_t sfcsr, sfdr;
	struct rtl838x_nor *rtl838x_nor = nor->priv;
	
	printk("In rtl838x_nor_read_reg: opcode %x, len %x\n", opcode, len);
	
	sfcsr = spi_prep(rtl838x_nor);
	sfdr = opcode << 24;
	
	sfcsr |= SPI_LEN1;
	
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	
	while (length > 0) {
		SPI_READY;
		*(buffer) = spi_r32(SFDR) >> 24;
	    	printk("%.2x  ", *(buffer));
		buffer++;
		length--;
	}
	
	return len;
}

static int rtl838x_nor_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len) 
{
	uint32_t sfcsr, sfdr;
	struct rtl838x_nor *rtl838x_nor = nor->priv;
	
	printk("In rtl838x_nor_write_reg, opcode %x, len %x\n", opcode, len);
	sfcsr = spi_prep(rtl838x_nor);
	sfdr = opcode << 24;
	
	if (len == 1) { /* SPINOR_OP_WRSR */
		sfdr |= buf[0];
		sfcsr |= SPI_LEN2;
		printk("  data: %x\n", buf[0]);
	}
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	return 0;
}

static int spi_read_id(struct rtl838x_nor *nor)
{
	uint32_t sfcsr, sfdr;
	uint32_t size = 4;
	int ret;
	uint8_t buf[8];
	uint8_t *buffer = buf;
	
	sfcsr = spi_prep(nor);
	sfdr = (SPINOR_OP_RDID)<<24;
		
	printk("spi_read_id: rdid,sfcsr_val = %.8x,SFDR = %.8x \n", sfcsr, sfdr);
	printk("rdid,sfcsr_val = %.8x\n",sfcsr | SPI_LEN4);
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);
	spi_w32_mask(0, SPI_LEN4, SFCSR);
	SPI_READY;
	
	ret = spi_r32(SFDR);
	
	while (size >= 4) {
		SPI_READY;
		*((uint32_t*) buffer) = spi_r32(SFDR);
		buffer += 4;
		size -= 4;
	}
	
	printk("Buffer %0x\n", *((uint32_t *)buf));
	
	return ret;
}

static int spi_enter_sio(struct spi_nor *nor)
{	
	uint32_t sfcsr, sfcr2, sfdr; 
	uint32_t ret = 0, tmp = 0;
	struct rtl838x_nor *rtl838x_nor = nor->priv;
	
	printk("In spi_enter_sio\n");
	rtl838x_nor->io_status = 0;
	sfdr = (SPI_C_RSTQIO)<<24;
	sfcsr = spi_prep(rtl838x_nor);
	
	tmp = spi_r32(SFCR2);
	sfcr2 = SFCR2_HOLD_TILL_SFDR2 | SFCR2_SIZE(SFCR2_GETSIZE( tmp )) | (tmp & SFCR2_RDOPT)
		| SFCR2_CMDIO( 0 ) | SFCR2_ADDRIO( 0 ) |  SFCR2_DUMMYCYCLE( 4 )
		| SFCR2_DATAIO( 0 ) | SFCR2_SFCMD(SPINOR_OP_READ_FAST);
	SPI_READY;
	spi_w32w(sfcr2, SFCR2);
	spi_w32w(sfcsr, SFCSR);
	spi_w32w(sfdr, SFDR);

	spi_w32_mask(SFCR2_HOLD_TILL_SFDR2, 0, SFCR2);
	rtl838x_nor->io_status &= ~IOSTATUS_CIO_MASK;
	rtl838x_nor->io_status |= CIO1;
	
	spi_prep(rtl838x_nor);
	
	return ret;
}

int rtl838x_spi_nor_scan(struct spi_nor *nor, const char *name, const struct spi_nor_hwcaps *hwcaps)
{
	uint32_t flash_id;
	struct rtl838x_nor *rtl838x_nor = nor->priv;
	
	printk("In spi_init\n");
	
	spi_w32_mask(0, SFCR_EnableWBO, SFCR);
	spi_w32_mask(0, SFCR_EnableRBO, SFCR);
	
	rtl838x_nor->flags = CS0 |R_MODE;
	
	spi_enter_sio(nor);
	spi_write_disable(rtl838x_nor);
	
	flash_id = spi_read_id(rtl838x_nor)>>8;
	printk("Flash ID: %x\n", flash_id);
	
	spi_nor_scan(nor, NULL, hwcaps);
	printk("------------- Got size: %llx\n", nor->mtd.size);
	
	return 0;
}

int rtl838x_nor_init(struct rtl838x_nor *rtl838x_nor,
			struct device_node *flash_node)
{
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ | SNOR_HWCAPS_PP
	};
	int ret;
	struct spi_nor *nor;

	printk("rtl838x_nor_init called\n");
	nor = &rtl838x_nor->nor;
	nor->dev = rtl838x_nor->dev;
	nor->priv = rtl838x_nor;
	spi_nor_set_flash_node(nor, flash_node);

	nor->read_reg = rtl838x_nor_read_reg;
	nor->write_reg = rtl838x_nor_write_reg;
	nor->read = rtl838x_nor_read;
	nor->write = rtl838x_nor_write;

	nor->mtd.name = "rtl838x_nor";
	/* initialized with NULL */
	ret = rtl838x_spi_nor_scan(nor, NULL, &hwcaps);
	if (ret)
		return ret;

	ret = mtd_device_parse_register(&nor->mtd, NULL, NULL, NULL, 0);
	return ret;
}


static int rtl838x_nor_drv_probe(struct platform_device *pdev)
{
	struct device_node *flash_np;
	struct resource *res;
	int ret;
	struct rtl838x_nor *rtl838x_nor;

	printk("Initializing rtl838x_nor_driver\n");
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}
	
	printk("Initializing rtl838x_nor_driver 1\n");
	rtl838x_nor = devm_kzalloc(&pdev->dev, sizeof(*rtl838x_nor), GFP_KERNEL);
	if (!rtl838x_nor)
		return -ENOMEM;
	printk("Initializing rtl838x_nor_driver 2\n");
	platform_set_drvdata(pdev, rtl838x_nor);

	printk("Initializing rtl838x_nor_driver 3\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtl838x_nor->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtl838x_nor->base))
		return PTR_ERR(rtl838x_nor->base);

	printk("Initializing rtl838x_nor_driver 4\n");
	rtl838x_nor->dev = &pdev->dev;

	/* only support one attached flash */
	flash_np = of_get_next_available_child(pdev->dev.of_node, NULL);
	if (!flash_np) {
		dev_err(&pdev->dev, "no SPI flash device to configure\n");
		ret = -ENODEV;
		goto nor_free;
	}
	
	printk("Initializing rtl838x_nor_driver 5\n");
	ret = rtl838x_nor_init(rtl838x_nor, flash_np);

nor_free:
	
	return ret;
}

static int rtl838x_nor_drv_remove(struct platform_device *pdev)
{
/*	struct rtl8xx_nor *rtl838x_nor = platform_get_drvdata(pdev); */
	return 0;
}

static const struct of_device_id rtl838x_nor_of_ids[] = {
	{ .compatible = "realtek,rtl838x-nor"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl838x_nor_of_ids);

static struct platform_driver rtl838x_nor_driver = {
	.probe = rtl838x_nor_drv_probe,
	.remove = rtl838x_nor_drv_remove,
	.driver = {
		.name = "rtl838x-nor",
		.pm = NULL,
		.of_match_table = rtl838x_nor_of_ids,
	},
};

module_platform_driver(rtl838x_nor_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("RTL838x SPI NOR Flash Driver");
