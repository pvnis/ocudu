#!/bin/bash
# One gNB run for RX delivery-lag work.
# Usage: lagrun.sh <label> <seconds>
# Env: DELAY  = OCUDU_LPHY_RX_TO_TX_DELAY_US (empty -> not set = stock 1 ms)
#      RXIRQ / TXIRQ = kernel buffers per interrupt (sysfs, applied at stream start)
#      DEVX   = extra device args (e.g. rx_poll=busy)
#      STATS  = worker lag report period in seconds (default 10), EVERY = measure every n-th buffer
#      YML    = config (default gnb_soapy_m2sdr_hwtimed.yml)
B=/home/dmd/ocudu/build; C=$B/captures; LBL=$1; SECS=$2
CON=$C/lag_$LBL.log
U=${M2SDR_UTIL:-/home/dmd/m2sdr/litex_m2sdr/software/user/m2sdr_util}
reg(){ echo $((16#$($U reg-read $1 2>/dev/null | tail -1 | grep -o "0x[0-9a-f]*$" | cut -c3-))); }
timeout 20 sudo pkill -INT -x gnb; for i in $(seq 12); do pgrep -x gnb >/dev/null||break; sleep 1; done; pgrep -x gnb >/dev/null && sudo pkill -9 -x gnb; sleep 1
sudo truncate -s0 /tmp/gnb.log 2>/dev/null; sudo rm -f /tmp/ocudu_soapy_rx_ts_shift
[ -n "$RXIRQ" ] && echo $RXIRQ | sudo tee /sys/module/m2sdr/parameters/rx_irq_period >/dev/null
[ -n "$TXIRQ" ] && echo $TXIRQ | sudo tee /sys/module/m2sdr/parameters/tx_irq_period >/dev/null
YML=${YML:-$B/gnb_soapy_m2sdr_hwtimed.yml}
[ -n "$NOWORKER" ] && { sed "s/rx_worker_packets=2048/rx_worker_packets=0/" $YML > $C/lag_tmp0.yml; YML=$C/lag_tmp0.yml; }
if [ -n "$DEVX" ]; then sed "s|timed_tx=hardware|timed_tx=hardware,$DEVX|" $YML > $C/lag_tmp.yml; YML=$C/lag_tmp.yml; fi
irq0=$(awk '/m2sdr/{s=0; for(i=2;i<=7;i++) s+=$i; print s}' /proc/interrupts)
cd $B && timeout 20 sudo -b sh -c "env ${DELAY:+OCUDU_LPHY_RX_TO_TX_DELAY_US=$DELAY} OCUDU_SOAPY_RX_TS_SHIFT=-42 M2SDR_RX_LAG_STATS=${STATS:-10} M2SDR_RX_LAG_EVERY=${EVERY:-1} $EXTRAENV nohup $B/apps/gnb/gnb -c $YML > $CON 2>&1 < /dev/null"
sleep 12
l0=$(reg 0x1580c); s0=$(reg 0x15810); i1=$(awk '/m2sdr/{s=0; for(i=2;i<=7;i++) s+=$i; print s}' /proc/interrupts); t1=$(date +%s.%N)
sleep $((SECS-12))
l1=$(reg 0x1580c); s1=$(reg 0x15810); i2=$(awk '/m2sdr/{s=0; for(i=2;i<=7;i++) s+=$i; print s}' /proc/interrupts); t2=$(date +%s.%N)
echo "[$LBL] delay=${DELAY:-stock} rxirq=$(cat /sys/module/m2sdr/parameters/rx_irq_period) txirq=$(cat /sys/module/m2sdr/parameters/tx_irq_period) devx=$DEVX gnb=$(pgrep -x gnb >/dev/null && echo up || echo DOWN)"
echo "  irq/s=$(echo "($i2-$i1)/($t2-$t1)" | bc) gate-late=+$((l1-l0)) gate-stale=+$((s1-s0)) PUxCH-late=$(sudo grep -ac 'PUxCH request late' /tmp/gnb.log) UL-busy=$(sudo grep -ac 'UL processor is busy' /tmp/gnb.log) PRACH-late=$(sudo grep -ac 'PRACH request late' /tmp/gnb.log) RF-late=$(sudo grep -ac 'RF: late\|Real-time failure in RF' /tmp/gnb.log) lates-console=$(grep -aci 'late\b' $CON)"
echo "  $(grep -a 'M2SDR RX delivery lag' $CON | tail -1)"
echo "  $(grep -a 'Soapy RX lag' $CON | tail -1)"
echo "  $(grep -a 'Soapy TX lead' $CON | tail -1)"
echo "  lag windows: $(grep -a "M2SDR RX delivery lag" $CON | sed -E "s/.*p99=([0-9]+) p99.9=([0-9]+) max=([0-9]+).*/\1\/\2\/\3/" | tr "\n" " ")  resyncs=$(grep -ac "TX ring resync" $CON) rx-disc=$(grep -ac "RX discontinuity" $CON)"
