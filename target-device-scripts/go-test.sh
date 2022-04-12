cd /tmp
#[ -f /tmp/priv_addr.txt ] || 
#  dmesg | grep OWL | grep psgmii_vco_calibrate | sed 's/.* priv = //' > /tmp/priv_addr.txt
#nc 172.20.1.5 1111 > /tmp/test_main.ko && \
#	insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt)

nc 172.20.1.5 1111 > /tmp/test_main.ko && \
	insmod /tmp/test_main.ko

