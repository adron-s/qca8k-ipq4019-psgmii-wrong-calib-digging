dmesg -c >/dev/null
insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt) yyy=12
insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt) yyy=181 $@
sleep 0.1
dmesg -c
insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt)
dmesg -c
insmod /tmp/test_main.ko owl=$(cat /tmp/priv_addr.txt) yyy=180 $@
#sleep 5
