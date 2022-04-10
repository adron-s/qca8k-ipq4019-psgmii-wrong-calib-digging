#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/module.h>
#include <net/ip.h>
#include <net/icmp.h>
#include <net/udp.h>
#include <net/route.h>
#include <linux/pkt_sched.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter/xt_tcpudp.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/if_arp.h>
#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/of_mdio.h>
#include <linux/mfd/syscon.h>
#include <net/protocol.h>
#include <net/dsa.h>
#include <../drivers/net/dsa/qca/qca8k-ipq4019.h>
#include <linux/synclink.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sergey Sergeev <sergey.sergeev@yapic.net>");
MODULE_DESCRIPTION("kernel test");

static unsigned long owl = 0;
module_param(owl, ulong, 0);
MODULE_PARM_DESC(owl, "An OwL");

static int yyy = 2;
module_param(yyy, int, 0);
MODULE_PARM_DESC(yyy, "An YYY");

static int owl_phy = -1;
module_param(owl_phy, int, 0);
MODULE_PARM_DESC(owl_phy, "phy for override default value");

static void bmcr_pdown(struct qca8k_priv *priv, int down)
{
	struct mii_bus *bus = priv->bus;
	int a, phy_addr, bmcr, res1, res2;
	for (a = 0; a < 5; a++) {
		phy_addr = a;
		bmcr = bus->read(bus, phy_addr, MII_BMCR);
		if (down)
			bmcr |= BMCR_PDOWN;
		else
			bmcr &= ~BMCR_PDOWN;
		bus->write(bus, phy_addr, MII_BMCR, bmcr);
	}
}

static int psgmii_vco_calibrate(struct qca8k_priv *priv)
{
	int val, ret, a;

	if (!priv->psgmii_ethphy) {
		pr_err("PSGMII eth PHY missing, calibration failed!\n");
		return -ENODEV;
	}
	//bmcr_pdown(priv, 1);

	/* Fix PSGMII RX 20bit */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
	//for (a = 0; a < 1; a++) {
		/* Reset PSGMII PHY */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x1b);
		/* Release reset */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
	//}
	/* Poll for VCO PLL calibration finish */
	ret = phy_read_mmd_poll_timeout(priv->psgmii_ethphy,
					MDIO_MMD_PMAPMD,
					0x28, val,
					(val & BIT(0)),
					10000, 1000000,
					false);
	if (ret) {
		pr_err("QCA807x PSGMII VCO calibration PLL not ready\n");
		return ret;
	}
	/* Check malibu(qca8075) PSGMII calibration done end ... */

	/* Freeze PSGMII RX CDR */
	ret = phy_write(priv->psgmii_ethphy, MII_RESV2, 0x2230);

	/* Start PSGMIIPHY VCO PLL calibration */
	ret = regmap_set_bits(priv->psgmii,
			PSGMIIPHY_VCO_CALIBRATION_CONTROL_REGISTER_1,
			PSGMIIPHY_REG_PLL_VCO_CALIB_RESTART);

	/* Poll for PSGMIIPHY PLL calibration finish */
	ret = regmap_read_poll_timeout(priv->psgmii,
				       PSGMIIPHY_VCO_CALIBRATION_CONTROL_REGISTER_2,
				       val, val & PSGMIIPHY_REG_PLL_VCO_CALIB_READY,
				       10000, 1000000);
	if (ret) {
		pr_err("PSGMIIPHY VCO calibration PLL not ready\n");
		return ret;
	}
	/* Check dakota(ipq40xx) PSGMII calibration done end ... */

	/* Release PSGMII RX CDR */
	ret = phy_write(priv->psgmii_ethphy, MII_RESV2, 0x3230);

	/* Release PSGMII RX 20bit */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5f);

	//bmcr_pdown(priv, 0);
	return ret;
}

#define AR40XX_MII_ATH_MMD_ADDR		0x0d
#define AR40XX_MII_ATH_MMD_DATA		0x0e
static u16
ar40xx_phy_mmd_read(struct qca8k_priv *priv, u32 phy_id,
		    u16 mmd_num, u16 reg_id)
{
	u16 value;
	struct mii_bus *bus = priv->bus;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_ADDR, mmd_num);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_DATA, reg_id);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_ADDR,
			0x4000 | mmd_num);
	value = bus->read(bus, phy_id, AR40XX_MII_ATH_MMD_DATA);
	mutex_unlock(&bus->mdio_lock);
	return value;
}
static void
ar40xx_phy_mmd_write(struct qca8k_priv *priv, u32 phy_id,
		     u16 mmd_num, u16 reg_id, u16 reg_val)
{
	struct mii_bus *bus = priv->bus;

	mutex_lock(&bus->mdio_lock);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_ADDR, mmd_num);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_DATA, reg_id);
	bus->write(bus, phy_id,
			AR40XX_MII_ATH_MMD_ADDR,
			0x4000 | mmd_num);
	bus->write(bus, phy_id,
		AR40XX_MII_ATH_MMD_DATA, reg_val);
	mutex_unlock(&bus->mdio_lock);
}
static int
qca8k_read(struct qca8k_priv *priv, u32 reg, u32 *val)
{
	return regmap_read(priv->regmap, reg, val);
}

static int
qca8k_write(struct qca8k_priv *priv, u32 reg, u32 val)
{
	return regmap_write(priv->regmap, reg, val);
}

static int
qca8k_rmw(struct qca8k_priv *priv, u32 reg, u32 mask, u32 write_val)
{
	return regmap_update_bits(priv->regmap, reg, mask, write_val);
}

#define   QCA8K_PHY_SPEC_STATUS_LINK		BIT(10)
#define   QCA8K_PORT_LOOKUP_LOOPBACK		BIT(21)
static void all_switch_ports_loopback_on_off(struct qca8k_priv *priv, int on)
{
	int i;
	u32 val = QCA8K_PORT_LOOKUP_LOOPBACK;
	if (on == 0) {
		pr_info("switch ports loopback is OFF\n");
		val = 0;
	} else {
		pr_info("switch ports loopback is ON\n");
	}
	for (i = 0; i < QCA8K_NUM_PORTS; i++) {
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(i),
			QCA8K_PORT_LOOKUP_LOOPBACK,
			val);
	}
}

static int wait_for_phy_link(struct qca8k_priv *priv, int phy, int need_status)
{
	struct mii_bus *bus = priv->bus;
	int a;
	u16 status;
	for (a = 0; a < 100; a++) {
		status = mdiobus_read(bus, phy, 0x11);
		status &= QCA8K_PHY_SPEC_STATUS_LINK;
		status = !!status;
		if (status == need_status) {
			pr_info("phy #%d link is %s\n", phy, status ? "UP" : "DOWN");
			return 0;
		}
		mdelay(8);
	}
	pr_info("phy #%d link is %s !!!\n", phy, status ? "UP" : "DOWN");
	return -1;
}

static void loopback_on_off(struct qca8k_priv *priv, int phy, int on)
{
	struct mii_bus *bus = priv->bus;
	u32 val = mdiobus_read(bus, phy, 0x0);
	/* switch to access MII reg for copper */
	mdiobus_write(bus, 4, 0x1f, 0x8500);
	if (on) {
		/* force no link by power down and reset phy */
		mdiobus_write(bus, phy, 0x0, 0x1840);
		wait_for_phy_link(priv, phy, 0);
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(phy + 1), 0);
		/* fix mdi status */
		mdiobus_write(bus, phy, 0x10, 0x6800);
		mdiobus_write(bus, phy, 0x0, 0x9000);
		mdiobus_write(bus, phy, 0x0, 0x4140);
		pr_info("phy #%d loopback is ON", phy);
		//set_current_state(TASK_INTERRUPTIBLE);
		//отпускаем ядро погулять :-) чтобы линк упал !!!
		//TODO: !!! тут нужно вставить ожидалку падения линка !!!
		//schedule_timeout(HZ);
		wait_for_phy_link(priv, phy, 1);
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(phy + 1), 0x7e);
		qca8k_write(priv, QCA8K_PORT_LOOKUP_CTRL(phy + 1), 0x40001);
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(phy + 1),
			QCA8K_PORT_LOOKUP_STATE_FORWARD,
			QCA8K_PORT_LOOKUP_STATE_FORWARD);
	} else { /* off */
		/* force no link by power down and reset phy */
		//mdiobus_write(bus, phy, 0x0, 0x1840);
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(phy + 1), 0);
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(phy + 1),
			QCA8K_PORT_LOOKUP_STATE_DISABLED,
			QCA8K_PORT_LOOKUP_STATE_DISABLED);
		/* disable phy internal loopback */
		mdiobus_write(bus, phy, 0x10, 0x6860);
		mdiobus_write(bus, phy, 0x0, 0x9040);
		pr_info("phy #%d loopback is OFF", phy);
	}
}

static void pkt_gen_on_off(struct qca8k_priv *priv, int phy, int on)
{
	struct mii_bus *bus = priv->bus;
	if (on) {
		wait_for_phy_link(priv, phy, 1);
		/* packet number */
		ar40xx_phy_mmd_write(priv, phy, 7, 0x8021, 0x1000);
		/* pkt size - 1504 bytes + 20 bytes */
		ar40xx_phy_mmd_write(priv, phy, 7, 0x8062, 0x05e0);
		/* start traffic */
		ar40xx_phy_mmd_write(priv, phy, 7, 0x8020, 0xa000);
		/* wait for all traffic end
	  * 4096(pkt num)*1524(size)*8ns(125MHz)=49.9ms
	  */
	  pr_info("phy(%d) pkt gen is ON\n", phy);
	} else { /* off */
		/* configuration recover */
		/* packet number */
		ar40xx_phy_mmd_write(priv, phy, 7, 0x8021, 0x0);
		/* disable traffic */
		ar40xx_phy_mmd_write(priv, phy, 7, 0x8020, 0x0);
		pr_info("phy(%d) pkt gen is OFF\n", phy);
	}
}

//*********************************************************
//выполняется при загрузке модуля
static int __init test_m_module_init(void)
{
	struct qca8k_priv *priv = (void *)owl;
	pr_info("priv: 0x%x\n", (unsigned int)priv);
	if (yyy == 69 && priv) {
		struct phy_device *phy = priv->psgmii_ethphy;
		struct mii_bus *bus = priv->bus;
		pr_info("phy: 0x%x, phy_id: %08x\n", (unsigned int)phy, phy->phy_id);
		pr_info("mii_bus: 0x%x, bus->name: %s\n", (unsigned int)bus, bus->name);
		pr_info("Doing PSGMII PHYs calibration\n");
		psgmii_vco_calibrate(priv);
		pr_info("Calibration is DONE\n");
	} else {
		pr_err("owl(priv) is NULL !!!\n");
	}
	if (0) {
		struct phy_device *phy = priv->psgmii_ethphy;
		struct mii_bus *bus = priv->bus;
		int a, phy_addr, bmcr, res1, res2;

		phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
		phy_write(priv->psgmii_ethphy, MII_BMCR, 0x1b);
		if (yyy != 0) {
			phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
			phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5f);
		}
		for (a = 0x0; a <= 0x5; a++) {
			phy_addr = a;
			res1 = bus->read(bus, phy_addr, MII_PHYSID1);
			res2 = bus->read(bus, phy_addr, MII_PHYSID2);
			pr_info("mdio_read(0x%x) = 0x%x, 0x%x\n", phy_addr, res1, res2);
			/* bmcr = bus->read(bus, phy_addr, MII_BMCR);
			pr_info("cur bmcr = 0x%x\n", bmcr);
			if (yyy != 2) {
				if (yyy == 0)
					bmcr |= BMCR_PDOWN;
				else if (yyy == 1)
					bmcr &= ~BMCR_PDOWN;
				bus->write(bus, phy_addr, MII_BMCR, bmcr);
			} */
		}
		//psgmii_vco_calibrate(priv);
	}
	if (yyy < 69) {
		int a;
		for (a = 0; a <= 4; a++) {
			int phy = a;
			u32 tx_ok, tx_error;
			u32 rx_ok, rx_error;
			u32 tx_ok_high16;
			u32 rx_ok_high16;
			u32 tx_all_ok, rx_all_ok;
			if (yyy == 12) {
				/* Enable CRC checker and packet counter to record ingress and egress packets */
				ar40xx_phy_mmd_write(priv, phy, 7, 0x8029, 0x0000);
				ar40xx_phy_mmd_write(priv, phy, 7, 0x8029, 0x0003);
				pr_info("CRC and packet counter record is enabled\n");
			}
			/* check counter */
			tx_ok = ar40xx_phy_mmd_read(priv, phy, 7, 0x802e);
			tx_ok_high16 = ar40xx_phy_mmd_read(priv, phy, 7, 0x802d);
			tx_error = ar40xx_phy_mmd_read(priv, phy, 7, 0x802f);
			rx_ok = ar40xx_phy_mmd_read(priv, phy, 7, 0x802b);
			rx_ok_high16 = ar40xx_phy_mmd_read(priv, phy, 7, 0x802a);
			rx_error = ar40xx_phy_mmd_read(priv, phy, 7, 0x802c);
			tx_all_ok = tx_ok + (tx_ok_high16<<16);
			rx_all_ok = rx_ok + (rx_ok_high16<<16);
			pr_info("*** phy: %d ***\n", phy);
			pr_info("tx_ok: %d, tx_error: %d\n", tx_all_ok, tx_error);
			pr_info("rx_ok: %d, rx_error: %d\n", rx_all_ok, rx_error);
		}
	}
	if (1) {
		int phy = 0x3;
		if (yyy == 70) {
			loopback_on_off(priv, phy, 1); //As far as I know, you need to enable loopback in the QCA8075 as well for the PRBS to work.
			//all_switch_ports_loopback_on_off(priv, 1);
			ar40xx_phy_mmd_write(priv, phy, 1, 0x54, 0x0000);
			ar40xx_phy_mmd_write(priv, phy, 1, 0x54, 0x0002); //PQSGMII_PRBS_EN
			ar40xx_phy_mmd_write(priv, phy, 1, 0x54, 0x0006); //PQSGMII_PRBS_BERT_EN
			pr_info("PRBS is ON\n");
		}
		if (yyy == 71) {
			ar40xx_phy_mmd_write(priv, phy, 1, 0x54, 0x0000);
			loopback_on_off(priv, phy, 0);
			//all_switch_ports_loopback_on_off(priv, 0);
			pr_info("PRBS is OFF\n");
		}
		if (yyy == 72) {
			u32 sync, ecr1, ecr2, ctrl_val;
			ecr1 = ar40xx_phy_mmd_read(priv, phy, 1, 0x53);
			ecr2 = ar40xx_phy_mmd_read(priv, phy, 1, 0x52);
			ctrl_val = ar40xx_phy_mmd_read(priv, phy, 1, 0x54);
			sync = ecr1 >> 15;
			//ecr1 &= 0x7FFF;
			pr_info("PRBS results: sync: %d, ecr1: %u, ecr2: %u, ctrl: 0x%x\n", sync, ecr1, ecr2, ctrl_val);
		}
	}

	if (yyy == 170) {
		int ret;
		/* Fix PSGMII RX 20bit */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
		/* Reset PSGMII PHY */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x1b);
		pr_info("reset the psgmii\n");
	}
	if (yyy == 171) {
		int ret;
		/* Release reset */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
		/* Release PSGMII RX 20bit */
		ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5f);
		pr_info("release the psgmii reset\n");
	}
	if (1) {
		int phy = owl_phy >= 0 ? owl_phy : 0x3;
		if (yyy == 180) {
			pkt_gen_on_off(priv, phy, 0);
			all_switch_ports_loopback_on_off(priv, 0);
			loopback_on_off(priv, phy, 0);
		}
		if (yyy == 181) {
			loopback_on_off(priv, phy, 1);
			all_switch_ports_loopback_on_off(priv, 1);
			pkt_gen_on_off(priv, phy, 1);
		}
		if (yyy == 189) {
			int a;
			for (a = 0; a < QCA8K_NUM_PORTS; a++) {
				u32 status = 0, ctrl = 0;
				qca8k_read(priv, QCA8K_REG_PORT_STATUS(a), &status);
				qca8k_read(priv, QCA8K_PORT_LOOKUP_CTRL(a), &ctrl);
				pr_info("port %d status = 0x%x, ctrl = 0x%x\n", a, status, ctrl);
			}
		}
		if (yyy == 190) {
			qca8k_write(priv, QCA8K_REG_PORT_STATUS(phy + 1), 0);
		}
		if (yyy == 191) {
			qca8k_write(priv, QCA8K_REG_PORT_STATUS(phy + 1), 0xc7e);
		}
		if (1) {
			struct mii_bus *bus = priv->bus;
			if (yyy == 200) {
				//mdiobus_write(bus, phy, 0x0, 0x9040);
				//loopback_on_off(priv, phy, 0);
				all_switch_ports_loopback_on_off(priv, 0);
			}
			if (yyy == 201) {
				//mdiobus_write(bus, phy, 0x0, 0x4140);
				//loopback_on_off(priv, phy, 1);
				all_switch_ports_loopback_on_off(priv, 1);
			}
		}
	}

	return -ENOMEM;
}//--------------------------------------------------------

//*********************************************************
//выполняется при выгрузке модуля
static void __exit test_m_module_exit (void){
}//--------------------------------------------------------

module_init(test_m_module_init);
module_exit(test_m_module_exit);
