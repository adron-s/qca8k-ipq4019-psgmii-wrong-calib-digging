COUNT=0
ip link set up dev port3
while [ 1 ]; do
  if ping -c 2 7.70.129.14; then
  	dmesg -c > /dev/null
    COUNT=$((COUNT+1))
    ip link set down dev port3
    sleep 1
    insmod /tmp/test_main.ko $@ 2>/dev/null yyy=200
    RETR_COUNT=`dmesg | grep "calibration retries" | sed -n 's/.*after \([0-9]\+\) calib.*/\1/p'`
    if [ -n "${RETR_COUNT}" ]; then
    	echo "OK with retryes(${RETR_COUNT}) - $COUNT"
	    echo "OK with retryes(${RETR_COUNT}) - $COUNT" | nc 172.20.1.77 1112
	  else
	  	echo "OK - $COUNT"
	  	echo "OK - $COUNT" | nc 172.20.1.77 1112
	  fi
	  ip link set up dev port3
    sleep 5
    #cat /sys/kernel/debug/regmap/c000000.switch/registers | grep "0264:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep "7c8:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1bc:|1d4:|1ec:|204:|21c:"
    #cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers | grep -E "1c0:|1d8:|1f0:|208:|220:"
  else
  	echo "FAIL - $COUNT"
    echo "FAIL - $COUNT" | nc 172.20.1.77 1112
    break
  fi
done

