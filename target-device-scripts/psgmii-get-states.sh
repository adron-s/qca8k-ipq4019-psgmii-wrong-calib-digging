		cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "09c:|0a0:" #PSGMIIPHY_VCO_CALIBRATION_CONTROL_REGISTER_1 and 2
		echo "*"
		cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1b4:|288:" #PSGMIIPHY_MODE_CONTROL and PSGMIIPHY_TX_CONTROL
		echo "*"
    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep "7c8:"
    echo "*"
    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1bc:|1d4:|1ec:|204:|21c:"
    echo "*"
    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1c0:|1d8:|1f0:|208:|220:"
    echo "*"
		cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "078:|0fc:|100:|104:|194:|1f0:|230:"