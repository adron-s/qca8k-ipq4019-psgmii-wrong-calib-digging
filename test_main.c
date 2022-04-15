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
#include <linux/reset.h>
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

static int psgmii_vco_calibrate(struct qca8k_priv *priv)
{
	int val, ret;

	if (!priv->psgmii_ethphy) {
		dev_err(priv->dev, "PSGMII eth PHY missing, calibration failed!\n");
		return -ENODEV;
	}

	/* Fix PSGMII RX 20bit */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
	/* Reset PHY PSGMII */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x1b);
	/* Release PHY PSGMII reset */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);

	/* Poll for VCO PLL calibration finish - Malibu(QCA8075) */
	ret = phy_read_mmd_poll_timeout(priv->psgmii_ethphy,
					MDIO_MMD_PMAPMD,
					0x28, val,
					(val & BIT(0)),
					10000, 1000000,
					false);
	if (ret) {
		dev_err(priv->dev, "QCA807x PSGMII VCO calibration PLL not ready\n");
		return ret;
	}
	mdelay(50);

	/* Freeze PSGMII RX CDR */
	ret = phy_write(priv->psgmii_ethphy, MII_RESV2, 0x2230);

	/* Start PSGMIIPHY VCO PLL calibration */
	ret = regmap_set_bits(priv->psgmii,
			PSGMIIPHY_VCO_CALIBRATION_CONTROL_REGISTER_1,
			PSGMIIPHY_REG_PLL_VCO_CALIB_RESTART);

	/* Poll for PSGMIIPHY PLL calibration finish - Dakota(IPQ40xx) */
	ret = regmap_read_poll_timeout(priv->psgmii,
				       PSGMIIPHY_VCO_CALIBRATION_CONTROL_REGISTER_2,
				       val, val & PSGMIIPHY_REG_PLL_VCO_CALIB_READY,
				       10000, 1000000);
	if (ret) {
		dev_err(priv->dev, "IPQ PSGMIIPHY VCO calibration PLL not ready\n");
		return ret;
	}
	mdelay(50);

	/* Release PSGMII RX CDR */
	ret = phy_write(priv->psgmii_ethphy, MII_RESV2, 0x3230);
	/* Release PSGMII RX 20bit */
	ret = phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5f);
	mdelay(200);

	return ret;
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
//#undef QCA8K_PSGMII_CALB_NUM
//#define	QCA8K_PSGMII_CALB_NUM							1

#define	QCA8K_PSGMII_CALB_NUM							100
#define QCA8K_PORT_LOOKUP_LOOPBACK				BIT(21)
#define	MII_QCA8075_SSTATUS								0x11
#define QCA8075_PHY_SPEC_STATUS_LINK			BIT(10)
#define QCA8075_MMD7_CRC_AND_PKTS_COUNT		0x8029
#define QCA8075_MMD7_PKT_GEN_PKT_NUMB			0x8021
#define QCA8075_MMD7_PKT_GEN_PKT_SIZE			0x8062
#define QCA8075_MMD7_PKT_GEN_CTRL					0x8020
#define QCA8075_MMD7_CNT_SELFCLR 					BIT(1)
#define QCA8075_MMD7_CNT_FRAME_CHK_EN 		BIT(0)
#define QCA8075_MMD7_PKT_GEN_START 				BIT(13)
#define QCA8075_MMD7_PKT_GEN_INPROGR 			BIT(15)
#define QCA8075_MMD7_IG_FRAME_RECV_CNT_HI 0x802a
#define QCA8075_MMD7_IG_FRAME_RECV_CNT_LO	0x802b
#define QCA8075_MMD7_IG_FRAME_ERR_CNT			0x802c
#define QCA8075_MMD7_EG_FRAME_RECV_CNT_HI	0x802d
#define QCA8075_MMD7_EG_FRAME_RECV_CNT_LO	0x802e
#define QCA8075_MMD7_EG_FRAME_ERR_CNT			0x802f
#define QCA8075_MMD7_MDIO_BRDCST_WRITE		0x8028
#define QCA8075_MMD7_MDIO_BRDCST_WRITE_EN BIT(15)
#define QCA8075_MDIO_BRDCST_PHY_ADDR			0x1f

static void qca8k_switch_port_loopback_on_off(
struct qca8k_priv *priv, int port, int on)
{
	u32 val = QCA8K_PORT_LOOKUP_LOOPBACK;

	if (on == 0) {
		pr_info("switch port #%d loopback is OFF\n", port);
		val = 0;
	} else {
		pr_info("switch port #%d loopback is ON\n", port);
	}

	qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(port),
		QCA8K_PORT_LOOKUP_LOOPBACK,	val);
}

static int qca8k_wait_for_phy_link_state(
struct phy_device *phy, int need_status)
{
	int a;
	u16 status;

	for (a = 0; a < 100; a++) {
		status = phy_read(phy, MII_QCA8075_SSTATUS);
		status &= QCA8075_PHY_SPEC_STATUS_LINK;
		status = !!status;
		if (status == need_status) {
			pr_info("phy #%d link is %s - OK\n", phy->mdio.addr, status ? "UP" : "DOWN");
			return 0;
		}
		mdelay(8);
	}

	pr_info("phy #%d link is %s - WRONG !!!\n", phy->mdio.addr, status ? "UP" : "DOWN");
	return -1;
}

static void qca8k_phy_loopback_on_off(struct qca8k_priv *priv,
struct phy_device *phy, int sw_port, int on)
{
	if (on) {
		phy_write(phy, MII_BMCR, BMCR_ANENABLE | BMCR_RESET);
		phy_modify(phy, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
		qca8k_wait_for_phy_link_state(phy, 0);
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(sw_port), 0);
		phy_write(phy, MII_BMCR,
			BMCR_SPEED1000 |
			BMCR_FULLDPLX |
			BMCR_LOOPBACK);
		qca8k_wait_for_phy_link_state(phy, 1);
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(sw_port),
			QCA8K_PORT_STATUS_SPEED_1000 |
			QCA8K_PORT_STATUS_TXMAC |
			QCA8K_PORT_STATUS_RXMAC |
			QCA8K_PORT_STATUS_DUPLEX);
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(sw_port),
			QCA8K_PORT_LOOKUP_STATE_FORWARD,
			QCA8K_PORT_LOOKUP_STATE_FORWARD);
		pr_info("phy #%d loopback is ON", phy->mdio.addr);
	} else { /* off */
		qca8k_write(priv, QCA8K_REG_PORT_STATUS(sw_port), 0);
		qca8k_rmw(priv, QCA8K_PORT_LOOKUP_CTRL(sw_port),
			QCA8K_PORT_LOOKUP_STATE_DISABLED,
			QCA8K_PORT_LOOKUP_STATE_DISABLED);
		phy_write(phy, MII_BMCR, BMCR_SPEED1000 | BMCR_ANENABLE | BMCR_RESET);
		pr_info("phy #%d loopback is OFF", phy->mdio.addr);
	}
}

static void qca8k_phy_pkt_gen_prep(struct qca8k_priv *priv,
struct phy_device *phy, int pkts_num, int on)
{
	if (on) {
		/* Enable CRC checker and packets counters */
		phy_write_mmd(phy, 7, QCA8075_MMD7_CRC_AND_PKTS_COUNT,
			0x0000);
		phy_write_mmd(phy, 7, QCA8075_MMD7_CRC_AND_PKTS_COUNT,
			QCA8075_MMD7_CNT_FRAME_CHK_EN | QCA8075_MMD7_CNT_SELFCLR);
		qca8k_wait_for_phy_link_state(phy, 1);
		/* packet number */
		phy_write_mmd(phy, 7, QCA8075_MMD7_PKT_GEN_PKT_NUMB, pkts_num);
		/* pkt size - 1504 bytes + 20 bytes */
		phy_write_mmd(phy, 7, QCA8075_MMD7_PKT_GEN_PKT_SIZE, 1504);
	  pr_info("phy(%02x) pkt gen is ready to launch\n", phy->mdio.addr);
	} else { /* off */
		/* packet number */
		phy_write_mmd(phy, 7, QCA8075_MMD7_PKT_GEN_PKT_NUMB, 0x0);
		/* disable CRC checker and packet counter */
		phy_write_mmd(phy, 7, QCA8075_MMD7_CRC_AND_PKTS_COUNT, 0x0);
		/* disable traffic gen */
		phy_write_mmd(phy, 7, QCA8075_MMD7_PKT_GEN_CTRL, 0x0);
		pr_info("phy(%02x) pkt gen is OFF\n", phy->mdio.addr);
	}
}

static void qca8k_wait_for_phy_pkt_gen_fin(struct qca8k_priv *priv,
struct phy_device *phy)
{
	int val;
	pr_info("phy(%02x): waiting for traffic end ...\n", phy->mdio.addr);
		/* wait for all traffic end: 4096(pkt num)*1524(size)*8ns(125MHz)=49938us */
	phy_read_mmd_poll_timeout(phy, 7,	QCA8075_MMD7_PKT_GEN_CTRL,
		val, !(val & QCA8075_MMD7_PKT_GEN_INPROGR),
		50000, 1000000,	true);
	pr_info("phy(%02x): traffic is end\n", phy->mdio.addr);
}

static void qca8k_start_phy_pkt_gen(struct phy_device *phy)
{
	pr_info("phy(%x): start pkt_gen\n", phy->mdio.addr);
	/* start traffic gen */
	phy_write_mmd(phy, 7, QCA8075_MMD7_PKT_GEN_CTRL,
		QCA8075_MMD7_PKT_GEN_START | QCA8075_MMD7_PKT_GEN_INPROGR);
}

static int qca8k_start_all_phys_pkt_gens(struct qca8k_priv *priv)
{
	struct phy_device *phy;
	phy = phy_device_create(priv->bus, QCA8075_MDIO_BRDCST_PHY_ADDR,
		0, 0, NULL);
	if (!phy) {
		dev_err(priv->dev, "unable to create mdio broadcast PHY(0x%x)\n",
			QCA8075_MDIO_BRDCST_PHY_ADDR);
		return -ENODEV;
	}

	qca8k_start_phy_pkt_gen(phy);

	phy_device_free(phy);
	return 0;
}

static int qca8k_get_phy_pkt_gen_test_result(
struct phy_device *phy, int pkts_num)
{
	u32 tx_ok, tx_error;
	u32 rx_ok, rx_error;
	u32 tx_ok_high16;
	u32 rx_ok_high16;
	u32 tx_all_ok, rx_all_ok;

	/* check counter */
	tx_ok = phy_read_mmd(phy, 7, QCA8075_MMD7_EG_FRAME_RECV_CNT_LO);
	tx_ok_high16 = phy_read_mmd(phy, 7, QCA8075_MMD7_EG_FRAME_RECV_CNT_HI);
	tx_error = phy_read_mmd(phy, 7, QCA8075_MMD7_EG_FRAME_ERR_CNT);
	rx_ok = phy_read_mmd(phy, 7, QCA8075_MMD7_IG_FRAME_RECV_CNT_LO);
	rx_ok_high16 = phy_read_mmd(phy, 7, QCA8075_MMD7_IG_FRAME_RECV_CNT_HI);
	rx_error = phy_read_mmd(phy, 7, QCA8075_MMD7_IG_FRAME_ERR_CNT);
	tx_all_ok = tx_ok + (tx_ok_high16 << 16);
	rx_all_ok = rx_ok + (rx_ok_high16 << 16);

	pr_info("*** phy: %d ***\n", phy->mdio.addr);
	pr_info("tx_ok: %d, tx_error: %d\n", tx_all_ok, tx_error);
	pr_info("rx_ok: %d, rx_error: %d\n", rx_all_ok, rx_error);

	if (tx_all_ok < pkts_num)
		return -1;
	if(rx_all_ok < pkts_num)
		return -2;
	if(tx_error)
		return -3;
	if(rx_error)
		return -4;
	return 0; /* test is ok */
}

static void qca8k_phy_broadcast_write_on_off(struct qca8k_priv *priv,
struct phy_device *phy, int on)
{
	u32 val;

	val = phy_read_mmd(phy, 7, QCA8075_MMD7_MDIO_BRDCST_WRITE);

	if (on == 0)
		val &= ~QCA8075_MMD7_MDIO_BRDCST_WRITE_EN;
	else
		val |= QCA8075_MMD7_MDIO_BRDCST_WRITE_EN;

	pr_info("phy(%02x): mdio brdcst write = 0x%04x\n", phy->mdio.addr, val);
	phy_write_mmd(phy, 7, QCA8075_MMD7_MDIO_BRDCST_WRITE, val);
}

static int qca8k_test_dsa_port_for_errors(struct qca8k_priv *priv,
struct phy_device *phy, int port, int test_phase)
{
	int res = 0;
	const int test_pkts_num = 4096;

	if (owl == 0) {
		if (test_phase == 1) { /* start test preps */
			qca8k_phy_loopback_on_off(priv, phy, port, 1);
			qca8k_switch_port_loopback_on_off(priv, port, 1);
			qca8k_phy_broadcast_write_on_off(priv, phy, 1);
			qca8k_phy_pkt_gen_prep(priv, phy, test_pkts_num, 1);
		} else if (test_phase == 2) {
			/* wait for test results, collect it and cleanup */
			qca8k_wait_for_phy_pkt_gen_fin(priv, phy);
			res = qca8k_get_phy_pkt_gen_test_result(phy, test_pkts_num);
			qca8k_phy_pkt_gen_prep(priv, phy, test_pkts_num, 0);
			qca8k_phy_broadcast_write_on_off(priv, phy, 0);
			qca8k_switch_port_loopback_on_off(priv, port, 0);
			qca8k_phy_loopback_on_off(priv, phy, port, 0);
		}
	}

	if (test_phase == 1) {
		if (owl == 500) {
			/* Enable CRC checker and packets counters */
			phy_write_mmd(phy, 7, QCA8075_MMD7_CRC_AND_PKTS_COUNT,
				0x0000);
			phy_write_mmd(phy, 7, QCA8075_MMD7_CRC_AND_PKTS_COUNT,
				QCA8075_MMD7_CNT_FRAME_CHK_EN | QCA8075_MMD7_CNT_SELFCLR);
		}
		if (owl == 501) {
			qca8k_get_phy_pkt_gen_test_result(phy, 0);
		}
		if (owl == 502) {
			//phy_modify(phy, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
			//phy_write(phy, MII_BMCR, BMCR_ANENABLE | BMCR_RESET);
			qca8k_phy_pkt_gen_prep(priv, phy, test_pkts_num, 1);
			qca8k_get_phy_pkt_gen_test_result(phy, 0);
			qca8k_phy_pkt_gen_prep(priv, phy, test_pkts_num, 0);
		}
	}

	return res;
}

static int qca8k_do_dsa_sw_ports_self_test(struct qca8k_priv *priv, int parallel_test)
{
	struct device_node *dn = priv->dev->of_node;
	struct device_node *ports, *port;
	struct device_node *phy_dn;
	struct phy_device *phy;
	int reg, err = 0, test_phase;
	u32 tests_result = 0;

	ports = of_get_child_by_name(dn, "ports");
	if (!ports) {
		dev_err(priv->dev, "no ports child node found\n");
			return -EINVAL;
	}

	for (test_phase = 1; test_phase <= 2; test_phase++) {
		if (parallel_test && test_phase == 2) {
			err = qca8k_start_all_phys_pkt_gens(priv);
			if (err)
				goto error;
		}
		for_each_available_child_of_node(ports, port) {
			err = of_property_read_u32(port, "reg", &reg);
			if (err)
				goto error;
			if (reg >= QCA8K_NUM_PORTS) {
				err = -EINVAL;
				goto error;
			}
			phy_dn = of_parse_phandle(port, "phy-handle", 0);
			if (phy_dn) {
				phy = of_phy_find_device(phy_dn);
				of_node_put(phy_dn);
				if (phy) {
					int result;
					result = qca8k_test_dsa_port_for_errors(priv, phy, reg, test_phase);
					if (!parallel_test && test_phase == 1)
						qca8k_start_phy_pkt_gen(phy);
					put_device(&phy->mdio.dev);
					if (test_phase == 2) {
						pr_info("port(%d): %s_test_result: %d\n", reg,
							parallel_test ? "parallel" : "serial", result);
						tests_result <<= 1;
						if (result)
							tests_result |= 1;
					}
				}
			}
		}
	}

end:
	of_node_put(ports);
	//qca8k_fdb_flush(priv);
	return tests_result;
error:
	tests_result |= 0xf000;
	goto end;
}

static void ipq_psgmii_do_reset(struct qca8k_priv *priv, int how)
{
	struct reset_control *rst;
	const char rst_name[ ] = "psgmii_rst";
	rst = devm_reset_control_get(priv->dev, rst_name);
	if (IS_ERR(rst)) {
		dev_err(priv->dev, "Failed to get %s control!\n", rst_name);
		return;
	}

	if (how == 0 || how >= 10) {
		pr_info("Doing %s assert\n", rst_name);
		/* Fix PSGMII RX 20bit */
		phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5b);
		/* Freeze PSGMII RX CDR */
		phy_write(priv->psgmii_ethphy, MII_RESV2, 0x2230);
		mdelay(50);
		reset_control_assert(rst);
	}
	if (how >= 10) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(100 * how));
	}
	if (how == 1 || how >= 10) {
		reset_control_deassert(rst);
		pr_info("Doing %s deassert\n", rst_name);
		/* Release PSGMII RX CDR */
		phy_write(priv->psgmii_ethphy, MII_RESV2, 0x3230);
		/* Release PSGMII RX 20bit */
		phy_write(priv->psgmii_ethphy, MII_BMCR, 0x5f);
		mdelay(200);
	}

	reset_control_put(rst);
}

static void *get_priv_from_if_name(const char *dev_name)
{
	struct net_device *dev;
	dev = dev_get_by_name(&init_net, dev_name);
	if(!dev){
		pr_err("Can't get dev %s!\n", dev_name);
		return NULL;
	}
	pr_info("Found dev with name = %s and index = %d\n", 
		dev->name, dev->ifindex);
	if (dev->dsa_ptr) {
		struct dsa_port *dp = dev->dsa_ptr;
		struct dsa_switch	*ds = dp->ds;
		pr_info("DSA port_ptr: 0x%x, switch_ptr: 0x%x\n", (u32)dp, (u32)ds);
		return ds->priv;
	}
	dev_put(dev);
	return NULL;
}

//*********************************************************
//выполняется при загрузке модуля
static int __init test_m_module_init(void)
{
	struct qca8k_priv *priv;
	priv = get_priv_from_if_name("eth0");
	pr_info("priv: 0x%x\n", (unsigned int)priv);
	if (!priv) {
		pr_err("owl(priv) is NULL !!!\n");
		return -ENOENT;
	}
	if (yyy == 69) {
		struct phy_device *phy = priv->psgmii_ethphy;
		struct mii_bus *bus = priv->bus;
		pr_info("phy: 0x%x, phy_id: %08x\n", (unsigned int)phy, phy->phy_id);
		pr_info("mii_bus: 0x%x, bus->name: %s\n", (unsigned int)bus, bus->name);
		pr_info("Doing PSGMII PHYs calibration\n");
		psgmii_vco_calibrate(priv);
		pr_info("Calibration is DONE\n");
	}
	if (0) {
		struct mii_bus *bus = priv->bus;
		int a, phy_addr, res1, res2;

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

	if (yyy == 189) {
		int a;
		for (a = 0; a < QCA8K_NUM_PORTS; a++) {
			u32 status = 0, ctrl = 0;
			qca8k_read(priv, QCA8K_REG_PORT_STATUS(a), &status);
			qca8k_read(priv, QCA8K_PORT_LOOKUP_CTRL(a), &ctrl);
			pr_info("port %d status = 0x%x, ctrl = 0x%x\n", a, status, ctrl);
		}
	}

	if (1) {
		if (yyy == 200) {
			int a, result;
			for (a = 0; a <= QCA8K_PSGMII_CALB_NUM; a++) {
				pr_info("Doing QCA8075 and PSGMII calibration\n");
				psgmii_vco_calibrate(priv);
				result = qca8k_do_dsa_sw_ports_self_test(priv, 0); /* serial test */
				if (!result)
					result = qca8k_do_dsa_sw_ports_self_test(priv, 1); /* parallel test */
				pr_info("qca8k dsa_sw_ports post calibration test is %s(0x%x)\n",
					result ? "FAULT!!!" : "Ok", result);
				if (!result) {
					if (a > 0) {
						dev_warn(priv->dev,	"PSGMII work was stabilized after %d "
						"calibration retries !\n", a);
					}
					break;
				} else {
					schedule();
					if (a > 0 && a % 10 == 0) {
						ipq_psgmii_do_reset(priv, a);
						set_current_state(TASK_INTERRUPTIBLE);
						schedule_timeout(msecs_to_jiffies(a * 100));
					}
				}
			}
		}
		if (yyy == 201) {
			struct mii_bus *bus = priv->bus;
			int phy = 0x3;
			//mdiobus_modify(bus, phy, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
			//set_current_state(TASK_INTERRUPTIBLE);
			//schedule_timeout(HZ);
			mdiobus_write(bus, phy, MII_BMCR, BMCR_ANENABLE | BMCR_RESET);
			mdiobus_modify(bus, phy, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
		}
		if (yyy == 210 || yyy == 211)
			ipq_psgmii_do_reset(priv, yyy - 210);
		if (yyy == 212)
			ipq_psgmii_do_reset(priv, 100);
		if (yyy == 220)
			qca8k_start_all_phys_pkt_gens(priv);
		if (yyy == 300 || yyy == 301) {
			u32 __iomem *reg;
			reg = ioremap(0x1812008, 4);
			if (reg) {
				*reg = yyy - 300;
				pr_info("0x1812008 val: 0x%x\n", *reg);
			}
			iounmap(reg);
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
