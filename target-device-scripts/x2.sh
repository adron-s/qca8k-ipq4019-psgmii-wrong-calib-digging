COUNT=0
while [ 1 ]; do
  if ping -c 2 7.70.129.12; then
    echo "OK - $COUNT"
    COUNT=$((COUNT+1))
    insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt) $@ 2>/dev/null yyy=69
    sleep 1
    #cat /sys/kernel/debug/regmap/c000000.switch/registers | grep "0264:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep "7c8:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1bc:|1d4:|1ec:|204:|21c:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1c0:|1d8:|1f0:|208:|220:"
  else
    echo "FAIL - $COUNT"
    break
  fi
done

