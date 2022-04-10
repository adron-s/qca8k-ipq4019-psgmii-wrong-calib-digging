    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep "7c8:"
    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1bc:|1d4:|1ec:|204:|21c:"
    cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1c0:|1d8:|1f0:|208:|220:"
