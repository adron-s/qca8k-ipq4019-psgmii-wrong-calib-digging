#!/bin/sh

SDIR=$1
WORK_DIR=/tmp/regmap-dumps/${SDIR}

[ -z "$SDIR" ] && {
  echo "Please pass sdir number"
  exit 0
}

save_log() {
  eval $1 > ${WORK_DIR}/"${1////-}".log
}
write_logs() {
  save_log 'cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/registers'
  save_log 'cat /sys/kernel/debug/regmap/c000000.switch-psgmii-phy/access'
  save_log 'cat /sys/kernel/debug/regmap/c000000.switch/registers'
  save_log 'cat /sys/kernel/debug/regmap/c000000.switch/access'
}
rm -Rf ${WORK_DIR}
mkdir -p ${WORK_DIR}
write_logs

