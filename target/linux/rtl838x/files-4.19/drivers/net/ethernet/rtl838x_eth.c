// SPDX-License-Identifier: GPL-2.0-only
/*
 * linux/drivers/net/ethernet/rtl838x_eth.c
 * Copyright (C) 2020 B. Koblitz
 */

#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <net/dsa.h>

#include <asm/mach-rtl838x/mach-rtl838x.h>
#include "rtl838x_eth.h"

#define RXRINGS		8
#define RXRINGLEN	32
#define TXRINGS		2
#define TXRINGLEN	20
#define TX_EN		0x8
#define RX_EN		0x4
#define TX_DO		0x2
#define WRAP		0x2

#define RING_BUFFER	1600

struct p_hdr {
	uint8_t		*buf;
	uint16_t	reserved;
	uint16_t	size;   /* buffer size */
	uint16_t	offset;
	uint16_t	len;    /* pkt len */
	uint16_t	reserved2;
	uint16_t	cpu_tag[5];
} __attribute__ ((aligned(1), packed));

struct ring_b {
	uint32_t	rx_r[RXRINGS][RXRINGLEN];
	uint32_t	tx_r[TXRINGS][TXRINGLEN];
	struct	p_hdr	rx_header[RXRINGS][RXRINGLEN];
	struct	p_hdr	tx_header[TXRINGS][TXRINGLEN];
	uint32_t	c_rx[RXRINGS];
	uint32_t	c_tx[TXRINGS];
	uint8_t		rx_space[RXRINGS*RXRINGLEN*RING_BUFFER];
	uint8_t		tx_space[TXRINGLEN*RING_BUFFER];
};

struct rtl838x_eth_priv {
	uint8_t		ports;  /* Number of ports w/o CPU-port */
	struct resource *res;
	struct net_device *netdev;
	struct platform_device *pdev;
	void 		*membase;
	spinlock_t	lock;
	struct mii_bus	*mii_bus;
	struct napi_struct napi;
};

extern int rtl838x_write_phy(u32 port, u32 page, u32 reg, u32 val);
extern int rtl838x_read_phy(u32 port, u32 page, u32 reg, u32 *val);

static irqreturn_t rtl838x_net_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	u32 status = sw_r32(RTL838X_DMA_IF_INTR_STS);
	
/*	printk("i s:%x e:%x\n", status, sw_r32(RTL838X_DMA_IF_INTR_MSK));*/
	
	/*  Ignore TX interrupt */
	if (! (status & 0xffff) ){
		sw_w32(0x000fffff, RTL838X_DMA_IF_INTR_STS);
		return IRQ_HANDLED;
	}
	
	/* Disable RX interrupt */
	sw_w32(0x00000000, RTL838X_DMA_IF_INTR_MSK);
	/* Clear ISR */
	sw_w32(0x000fffff, RTL838X_DMA_IF_INTR_STS);
	napi_schedule(&priv->napi);
	return IRQ_HANDLED;
}

static void rtl838x_hw_reset(void)
{	
	/* Stop TX/RX */
	sw_w32(0x0, RTL838X_MAC_PORT_CTRL(CPU_PORT));
	udelay(50 * 1000);

	/* Reset NIC */
	sw_w32(0x08, RTL838X_RST_GLB_CTRL_0);
	udelay(50 * 1000);
    
	do {
		udelay(20);
	} while (sw_r32(RTL838X_RST_GLB_CTRL_0));

	/* Restart TX/RX to CPU_PORT */
	sw_w32(0x03, RTL838X_MAC_PORT_CTRL(CPU_PORT));
	
	/* Link up, also 
	sw_w32_mask(0, 0x03, RTL838X_MAC_FORCE_MODE_CTRL(CPU_PORT)); */

	/* Set Speed, duplex, flow control */
	sw_w32(0x6192F, RTL838X_MAC_FORCE_MODE_CTRL(CPU_PORT));

	/* allow CRC errors on CPU-port */
	sw_w32_mask(0, 0x8, RTL838X_MAC_PORT_CTRL(CPU_PORT));
	
	/* Disable and clear interrupts */
	sw_w32(0x00000000, RTL838X_DMA_IF_INTR_MSK);
	sw_w32(0xffffffff, RTL838X_DMA_IF_INTR_STS);
}

static void rtl838x_hw_ring_setup(struct rtl838x_eth_priv *priv)
{
	int i;
	struct ring_b *ring = priv->membase;

	for (i = 0; i < RXRINGS; i++)
		sw_w32(CPHYSADDR(&ring->rx_r[i]), RTL838X_DMA_RX_BASE(i));

	for (i = 0; i < TXRINGS; i++)
		sw_w32(CPHYSADDR(&ring->tx_r[i]), RTL838X_DMA_TX_BASE(i));
}

static void rtl838x_hw_en_rxtx(void)
{
	/* Disable Head of Line features for all RX rings */
	sw_w32(0xffffffff, RTL838X_DMA_IF_RX_RING_SIZE(0));
	
	/* Truncate RX buffer to 0x640 (1600 bytes), pad TX */
	sw_w32(0x64000020, RTL838X_DMA_IF_CTRL);
	
	/* Enable RX done, RX overflow and TX done interrupts */
	sw_w32(0xfffff, RTL838X_DMA_IF_INTR_MSK);

	/* Enable traffic, engine expects empty FCS field */
	sw_w32_mask(0, RX_EN | TX_EN, RTL838X_DMA_IF_CTRL);
	
}

static void rtl838x_setup_ring_buffer(struct ring_b *ring)
{
	int i, j;

	struct p_hdr *h;

	for (i = 0; i < RXRINGS; i++) {
		for (j=0; j < RXRINGLEN; j++) {
			h = &ring->rx_header[i][j];
			h->buf = (u8 *)CPHYSADDR(ring->rx_space + i * j * RING_BUFFER);
			h->reserved = 0;
			h->size = RING_BUFFER;
			h->offset = 0;
			h->len = 0;
			/* All rings owned by switch, last one wraps */
			ring->rx_r[i][j] = CPHYSADDR(h) | 0x1 | (j == (RXRINGLEN - 1)? WRAP : 0x0);
		}
		ring->c_rx[i] = 0;
	}
	
	for (i = 0; i < TXRINGS; i++) {
		for (j=0; j < TXRINGLEN; j++) {
			h = &ring->tx_header[i][j];
			h->buf = (u8 *)CPHYSADDR(ring->tx_space + i * j * RING_BUFFER);
			h->reserved = 0;
			h->size = RING_BUFFER;
			h->offset = 0;
			h->len = 0;
			ring->tx_r[i][j] = CPHYSADDR(&ring->tx_header[i][j]);
		}
		/* Last header is wrapping around */
		ring->tx_r[i][j-1] |= 0x2; 
		ring->c_tx[i] = 0;
	}
	
	for (i = 0; i < TXRINGS; i++) {
		for (j=0; j < TXRINGLEN; j++) {

		}
	}
}

static int rtl838x_eth_open(struct net_device *dev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	struct ring_b *ring = priv->membase;
	int ret;
	
	printk("rtl838x_eth_open called %x, ring %x\n", (uint32_t)priv, (uint32_t)ring);
	spin_lock_irqsave(&priv->lock, flags);
	rtl838x_hw_reset();
	rtl838x_setup_ring_buffer(ring);
	rtl838x_hw_ring_setup(priv);
	
	ret = request_irq(dev->irq, rtl838x_net_irq, IRQF_SHARED,
			dev->name, dev);
	if (ret)
		return ret;
	
	napi_enable(&priv->napi);
	
	netif_start_queue(dev);
	
	rtl838x_hw_en_rxtx();
	spin_unlock_irqrestore(&priv->lock, flags);
	
	return 0;
}

static void rtl838x_hw_stop(void)
{
	sw_w32(0x00000000, RTL838X_DMA_IF_INTR_MSK);
	sw_w32(0x000fffff, RTL838X_DMA_IF_INTR_STS);
	sw_w32(0x00000000, RTL838X_DMA_IF_CTRL);
}

static int rtl838x_eth_stop(struct net_device *dev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	printk("in rtl838x_eth_stop %x\n", (uint32_t)priv);
	spin_lock_irqsave(&priv->lock, flags);
	rtl838x_hw_stop();
	
	free_irq(dev->irq, dev);
	napi_disable(&priv->napi);
	netif_stop_queue(dev);
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;
}

static void rtl838x_eth_set_multicast_list(struct net_device *dev)
{
	//struct rtl838x_eth_priv *priv = netdev_priv(dev);

//	printk("in rtl838x_set_multicast_list %x\n", (uint32_t)priv);
	/* set loopback mode if requested
	if (dev->flags & IFF_LOOPBACK)
		printk("Enable loopback\n");
	else
		printk("Disable loopback\n");
*/
	/* receive broadcast frames if requested
	if (dev->flags & IFF_BROADCAST)
		printk("Enable receive broadcast frames\n");
	else
		printk("Disable receive broadcast frames\n");
*/
	/* enable promiscuous mode if requested
	if (dev->flags & IFF_PROMISC)
		printk("Enable promiscuous mode\n");
	else
		printk("Disable promiscuous mode\n");

*/
}

static void rtl838x_eth_tx_timeout(struct net_device *dev)
{
	unsigned long flags;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	printk("in rtl838x_eth_tx_timeout %x\n", (uint32_t)priv);
	spin_lock_irqsave(&priv->lock, flags);
	rtl838x_hw_stop();
	rtl838x_hw_ring_setup(priv);
	rtl838x_hw_en_rxtx();
	netif_trans_update(dev);
	netif_start_queue(dev);
	spin_unlock_irqrestore(&priv->lock, flags);
}

/*
static void dump_pkt(unsigned char *pkt, int len)
{
    int i;

    for (i=0; i<len; i++)
    {
        if (i%16 == 0) {
            printk(KERN_CONT "[%04X] ", i);
        }

        printk(KERN_CONT"%02X ", *(pkt + i));

        if (i%16 == 15) {
            printk(" ");
        }
    }

    printk("\n");
}
*/

static int rtl838x_eth_tx(struct sk_buff *skb, struct net_device *dev)
{
	int len, i;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	struct ring_b *ring = priv->membase;
	uint32_t val;
	static int num = 0;
	int ret;
	unsigned long flags;
	struct p_hdr *h;
	int dest_port = -1;
	
	len = skb->len;
	/* Check for DSA tagging at the end of the buffer */
	if (netdev_uses_dsa(dev) && skb->data[len-4] == 0x80 && skb->data[len-3] >0 
			&& skb->data[len-3] < 28 &&  skb->data[len-2] == 0x10 
			&&  skb->data[len-1] == 0x00) {
		/* Reuse tag space for CRC */
		dest_port = skb->data[len-3];
/*		printk("TX DSA detected, reducing len: %d, dest_port: \n", len, dest_port); */
		len -= 4;
	}
	if (len < ETH_ZLEN)
		len = ETH_ZLEN;
	/* ASIC expects that packet includes CRC, so we extend 4 bytes */
	len += 4;

/*	if (num < 10)
		printk("TX Len: %d c_tx: %d\n", len, ring->c_tx[0]);*/
	if (skb_padto(skb, len ))
		return NETDEV_TX_OK;

/*	if (num < 10)
		dump_pkt(skb->data, len);*/
	num++;
	
/*	if (num < 11)
		printk("TX Before: %x", (u32)ring->tx_r[0][ring->c_tx[0]]); */

	spin_lock_irqsave(&priv->lock, flags);
	/* We can send this packet if CPU owns the descriptor */
	if (!(ring->tx_r[0][ring->c_tx[0]] & 0x1)) {
		/* Set descriptor for tx */
		h = &ring->tx_header[0][ring->c_tx[0]];
		h->buf = (u8 *)CPHYSADDR(ring->tx_space);
		h->size = len;
		h->len = len;

		/* Create cpu_tag */
		if (dest_port > 0) {
			h->cpu_tag[0] = 0x0400; 
			h->cpu_tag[1] = 0x0200;
			h->cpu_tag[2] = 0x0000;
			h->cpu_tag[3] = (1 << dest_port) >> 16;
			h->cpu_tag[4] = (1 << dest_port) & 0xffff;
		} else {
			h->cpu_tag[0] = 0;
			h->cpu_tag[1] = 0;
			h->cpu_tag[2] = 0;
			h->cpu_tag[3] = 0;
			h->cpu_tag[4] = 0;
		}

		/* Copy packet data to tx buffer */
/*		printk("Ack # %x\n", *(u32 *)(skb->data+0x2a));*/
		memcpy((void *)KSEG1ADDR(h->buf), skb->data, len);
		mb(); /* wmb() works, too */
/*		if(num < 11)
			dump_pkt((void *)((u32)h->buf| 0xa0000000), len); */
		/* Hand over to switch */
		ring->tx_r[0][ring->c_tx[0]] = ring->tx_r[0][ring->c_tx[0]] | 0x1; 
/*		if (num < 11)
			printk("TX After: %x", (u32)ring->tx_r[0][ring->c_tx[0]]); */
		/* BUG: before tx fetch, need to make sure right data is accessed */
		for(i = 0; i < 10; i++) {
			val = sw_r32(RTL838X_DMA_IF_CTRL);
			if( (val & 0xc) == 0xc )
				break;
		}

		/* Tell switch to send data */
		sw_w32_mask(0, TX_DO, RTL838X_DMA_IF_CTRL);

		dev->stats.tx_packets++;
		dev->stats.tx_bytes += len;
		dev_kfree_skb(skb);
		ring->c_tx[0] = (ring->c_tx[0] + 1) % TXRINGLEN;
		ret = NETDEV_TX_OK;
	} else {
		dev_warn(&priv->pdev->dev, "Data is owned by switch\n");
		ret = NETDEV_TX_BUSY;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static int rtl838x_hw_receive(struct net_device *dev, int r)
{
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	struct ring_b *ring = priv->membase;
	struct sk_buff *skb;
	unsigned long flags;
	int i, j, len, work_done = 0;
	u8 *data, *skb_data;
/*	static int num = 0; */
	unsigned int val;
	u32	*last;
	struct p_hdr *h;
	
	spin_lock_irqsave(&priv->lock, flags);
	last = (u32 *)KSEG1ADDR(sw_r32(RTL838X_DMA_IF_RX_CUR(r)));
/*	printk("ring %2x (index %2x): current %x, last: %x\n", r, ring->c_rx[r], (u32) &ring->rx_r[r][ring->c_rx[r]], (u32) last); */
	
	if ( &ring->rx_r[r][ring->c_rx[r]] == last ) {
		spin_unlock_irqrestore(&priv->lock, flags);
		return 0;
	}
	do {
		if ((ring->rx_r[r][ring->c_rx[r]] & 0x1)) {
			printk("WARNING: %x, %x, ISR %x\n", r, (uint32_t)priv, sw_r32(RTL838X_DMA_IF_INTR_STS));

			for (i = 0; i < RXRINGS; i++) {
				printk(KERN_CONT "%x ", KSEG1ADDR(sw_r32(RTL838X_DMA_IF_RX_CUR(i))));
			}
			printk(" ");
	
			for (j = 0; j < RXRINGLEN; j++)
				printk(KERN_CONT "%x ", (u32)&ring->rx_r[r][j]);
			printk("--\n");
			break;
		}
		h = &ring->rx_header[r][ring->c_rx[r]];
/*		printk("RX: CPU-Tag: %x %x %x %x %x\n", h->cpu_tag[0], h->cpu_tag[1], h->cpu_tag[2],
			h->cpu_tag[3],h->cpu_tag[4]);
*/		
		data = (u8 *)KSEG1ADDR(h->buf);
		len = h->len;
/*		printk("pkt: %x %x\n", (u32)data, (u32)len);*/
		if (!len)
			break;
		
		h->buf = (u8 *)CPHYSADDR(&ring->rx_space);
		h->size = RING_BUFFER;
		h->len = 0;
		work_done++;
/*		printk("buffer now: %x %x %x\n", (u32)ring->rx_header.buf, (u32)ring->rx_r[0], (u32)ring->rx_r[1]);*/
		
		len -= 4; /* strip the CRC */
		/* Add 4 bytes for cpu_tag */
		if (netdev_uses_dsa(dev))
			len += 4;
		
		skb = alloc_skb(len + 4, GFP_KERNEL);
		skb_reserve(skb, NET_IP_ALIGN);

		if (likely(skb)) {
			/* BUG: Prevent bug */
			sw_w32(0xffffffff, RTL838X_DMA_IF_RX_RING_SIZE(0));
			for(i = 0; i < RXRINGS; i++) {
				/*clear every ring cnt to 0x0*/
				val = sw_r32(RTL838X_DMA_IF_RX_RING_CNTR(i));
				sw_w32(val, RTL838X_DMA_IF_RX_RING_CNTR(i));
			}

			skb_data = skb_put(skb, len);
			mb();
			memcpy(skb->data, (u8 *)KSEG1ADDR(data), len);
			/* Overwrite CRC with cpu_tag */
			if (netdev_uses_dsa(dev)) {
				skb->data[len-4] = 0x80;
				skb->data[len-3] = h->cpu_tag[0] & 0x1f;
				skb->data[len-2] = 0x10;
				skb->data[len-1] = 0x00;
			}
			
/*			if (num < 10) {
				num++;
				dump_pkt(data, len);
				printk("RX Got:\n");
				dump_pkt(skb->data, len);
			} */
			skb->protocol = eth_type_trans(skb, dev);
			dev->stats.rx_packets++;
			dev->stats.rx_bytes += len;

			netif_receive_skb(skb);
		} else {
			if (net_ratelimit())
				dev_warn(&dev->dev,
				    "low on memory - packet dropped\n");
			dev->stats.rx_dropped++;
		}
		ring->rx_r[r][ring->c_rx[r]] 
			= CPHYSADDR(h) | 0x1 | (ring->c_rx[r] == (RXRINGLEN-1)? WRAP : 0x1);
		ring->c_rx[r] = (ring->c_rx[r] + 1) % RXRINGLEN;
	} while (&ring->rx_r[r][ring->c_rx[r]] != last);
	
	spin_unlock_irqrestore(&priv->lock, flags);
	/* Clear ISR */
	sw_w32(0x000fffff, RTL838X_DMA_IF_INTR_STS);
	
	return work_done;
}

static int rtl838x_poll_rx(struct napi_struct *napi, int budget)
{
	struct rtl838x_eth_priv *priv = container_of(napi, struct rtl838x_eth_priv, napi);
	int work_done = 0, r = 0;

/*	printk("in rtl838x_poll_rx\n");*/
	while (work_done < budget && r < RXRINGS) {
		work_done += rtl838x_hw_receive(priv->netdev, r);
		r++;
	}

	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		/* Enable RX interrupt */
		sw_w32(0xfffff, RTL838X_DMA_IF_INTR_MSK);
	}
	return work_done;
}

static int rtl838x_set_mac_address(struct net_device *dev, void *p)
{
	const struct sockaddr *addr = p;
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	unsigned long flags;
	
	u8 *mac = (u8*) (p);
	
	if(IS_ERR(p))
		mac = dev->dev_addr;
	
	printk("In rtl838x_set_mac_address %x %x\n", (uint32_t)priv, (uint32_t) addr);
        spin_lock_irqsave(&priv->lock, flags);
        sw_w32((mac[0] << 8) | mac[1], RTL838X_MAC);
        sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5], RTL838X_MAC + 4);

	/* 2 more registers, ALE/MAC block */
        sw_w32((mac[0] << 8) | mac[1], RTL838X_MAC_ALE);
        sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5], (RTL838X_MAC_ALE + 4));
	
        sw_w32((mac[0] << 8) | mac[1], RTL838X_MAC2);
        sw_w32((mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5], RTL838X_MAC2 + 4);
	
	ether_addr_copy(dev->dev_addr, p);
	spin_unlock_irqrestore(&priv->lock, flags);
	
	return 0;
}

static const struct net_device_ops rtl838x_eth_netdev_ops = {
	.ndo_open = rtl838x_eth_open,
	.ndo_stop = rtl838x_eth_stop,
	.ndo_start_xmit = rtl838x_eth_tx,
	.ndo_set_mac_address = rtl838x_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_rx_mode = rtl838x_eth_set_multicast_list,
	.ndo_tx_timeout = rtl838x_eth_tx_timeout,
};

static int rtl838x_mdio_write(struct mii_bus *bus, int phy_addr,
			  int phy_reg, u16 val)
{
	printk("PHY-write: addr %d %x %x\n", phy_addr, phy_reg, val);
	
	return rtl838x_write_phy(phy_addr, 0, phy_reg, val);
}

static int rtl838x_mdio_read(struct mii_bus *bus, int phy_addr, int phy_reg)
{
//	struct rtl838x_eth_priv *priv = bus->priv;
	u32 val;
	
/*	printk("PHY-read: addr %d reg: %x\n", phy_addr, phy_reg); */
	rtl838x_read_phy(phy_addr, 0, phy_reg, &val);
/*	printk("Result: %x\n", val); */
	return val;
}

static void rtl838x_mdio_cleanup(struct rtl838x_eth_priv *priv)
{
	if (!priv->mii_bus)
		return;

	mdiobus_unregister(priv->mii_bus);
}

static int rtl838x_mdio_init(struct rtl838x_eth_priv *priv)
{
	struct device_node *mii_np;
	int ret;

	mii_np = of_get_child_by_name(priv->pdev->dev.of_node, "mdio-bus");
	if (!mii_np) {
		dev_err(&priv->pdev->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}
	printk("Found mdio node\n");
	if (!of_device_is_available(mii_np)) {
		ret = -ENODEV;
		goto err_put_node;
	}

	priv->mii_bus = devm_mdiobus_alloc(&priv->pdev->dev);
	if (!priv->mii_bus) {
		ret = -ENOMEM;
		goto err_put_node;
	}
	printk("Allocated mdio bus\n");
	priv->mii_bus->name = "mdio";
	priv->mii_bus->read = rtl838x_mdio_read;
	priv->mii_bus->write = rtl838x_mdio_write;
	priv->mii_bus->priv = priv;
	priv->mii_bus->parent = &priv->pdev->dev;

	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%pOFn", mii_np);
	ret = of_mdiobus_register(priv->mii_bus, mii_np);

err_put_node:
	of_node_put(mii_np);
	return ret;
}

static int __init rtl838x_eth_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct rtl838x_eth_priv *priv;
	struct resource *res, *mem;
	const void *mac;
	int err;

	pr_info("Probing RTL838X eth device\n");
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}
	dev = alloc_etherdev(sizeof(struct rtl838x_eth_priv));
	if (!dev) {
		err = -ENOMEM;
		goto err_free;
	}
	SET_NETDEV_DEV(dev, &pdev->dev);
	priv = netdev_priv(dev);
	
	/* obtain buffer memory space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		mem = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), res->name);
		if (!mem) {
			dev_err(&pdev->dev, "cannot request memory space\n");
			err = -ENXIO;
			goto err_free;
		}

		dev->mem_start = mem->start;
		dev->mem_end   = mem->end;
	}	
	
	/* Allocate buffer memory */
	priv->membase = dmam_alloc_coherent(&pdev->dev,
				sizeof(struct ring_b), (void *)&dev->mem_start,
				GFP_KERNEL);
	if (!priv->membase) {
		dev_err(&pdev->dev, "cannot allocate %dB buffer\n",
		sizeof(struct ring_b));
		err = -ENOMEM;
		goto err_free;
	}
		
	spin_lock_init(&priv->lock);
	
	/* obtain device IRQ number */
	/* res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot obtain IRQ\n");
		err = -ENXIO;
		goto err_free;
	}
	dev->irq = res->start;
	*/
	dev->irq = 32;
	mac = of_get_mac_address(pdev->dev.of_node);
	if (!IS_ERR(mac)) {
		ether_addr_copy(dev->dev_addr, mac);
	} else {
		printk("Could not read MAC address\n");
		eth_hw_addr_random(dev);
	}
	rtl838x_set_mac_address(dev, (void *)mac);
	
	strcpy(dev->name, "eth%d");
	dev->netdev_ops = &rtl838x_eth_netdev_ops;
	priv->res = res;
	priv->pdev = pdev;
	priv->netdev = dev;
	
	err = rtl838x_mdio_init(priv);
	if (err)
		goto err_free;
	
	
	err = register_netdev(dev);
	if (err)
		goto err_free;
	
	netif_napi_add(dev, &priv->napi, rtl838x_poll_rx, 64);
	platform_set_drvdata(pdev, dev);
	return 0;

err_free:
	printk("Error setting up netdev, freeing it again.\n");
	free_netdev(dev);
	return err;
}

static int rtl838x_eth_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct rtl838x_eth_priv *priv = netdev_priv(dev);
	if (dev) {
		printk("Removing platform driver for rtl838x-eth\n");
		rtl838x_mdio_cleanup(priv);
		rtl838x_hw_stop();
		netif_stop_queue(dev);
		netif_napi_del(&priv->napi);
		unregister_netdev(dev);
		free_netdev(dev);
	}
	return 0;
}

static const struct of_device_id rtl838x_eth_of_ids[] = {
	{ .compatible = "realtek,rtl838x-eth"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtl838x_eth_of_ids);

static struct platform_driver rtl838x_eth_driver = {
	.probe = rtl838x_eth_probe,
	.remove = rtl838x_eth_remove,
	.driver = {
		.name = "rtl838x-eth",
		.pm = NULL,
		.of_match_table = rtl838x_eth_of_ids,
	},
};

module_platform_driver(rtl838x_eth_driver);

MODULE_AUTHOR("B. Koblitz");
MODULE_DESCRIPTION("RTL838X SoC Ethernet Driver");
MODULE_LICENSE("GPL");
