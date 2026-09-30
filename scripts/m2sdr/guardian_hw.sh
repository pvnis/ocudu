#!/bin/bash
# Guardian for the hardware-timed TX mode (m2sdr branch hw-timed-tx, device_args timed_tx=hardware).
# No emission-offset measurement or RX shift juggling: the DL emission offset is a constant, applied once
# through OCUDU_SOAPY_RX_TS_SHIFT. Keeps the gNB alive, restarts it on real-time trouble, logs on change.
# Usage: guardian_hw.sh [shift_samples]   (default -42 = minus the align.py RX-minus-TX offset)
# Stop:  touch /home/dmd/ocudu/build/captures/guardian_hw.stop
set +e
B=/home/dmd/ocudu/build; C=$B/captures; cd $B
SHIFT=${1:--42}
G=$C/guardian_hw.log; L=$C/gnb_hwtimed_console.log; STOP=$C/guardian_hw.stop
U=/home/dmd/m2sdr/litex_m2sdr/software/user/m2sdr_util
A(){ timeout 20 adb "$@" 2>/dev/null; }
reg(){ $U reg-read $1 2>/dev/null | tail -1 | grep -o "0x[0-9a-f]*$"; }
log(){ echo "$(date -u +%H:%M:%S) $*" >> $G; }
start_gnb(){
  timeout 20 sudo pkill -INT -x gnb; for i in $(seq 15); do pgrep -x gnb >/dev/null||break; sleep 1; done; pgrep -x gnb >/dev/null && timeout 10 sudo pkill -9 -x gnb; sleep 1
  [ -s $L ] && cp $L $C/gnb_hwtimed_console.prev.log
  sudo rm -f /tmp/ocudu_soapy_rx_ts_shift
  timeout 20 sudo -b sh -c "env OCUDU_LPHY_RX_TO_TX_DELAY_US=5000 OCUDU_SOAPY_RX_TS_SHIFT=$SHIFT nohup $B/apps/gnb/gnb -c $B/gnb_soapy_m2sdr_hwtimed.yml > $L 2>&1 < /dev/null"
  sleep 20; log "gNB started (shift $SHIFT): $(pgrep -x gnb >/dev/null && echo up || echo FAILED)"
}
# The gNB warns that DRM KMS connector polling "may hinder performance"; disable it (reversible, resets
# on reboot). This is host hygiene, not the cure for the UL storms -- those came from non-sample-exact
# RX timestamps and are fixed in the m2sdr driver (docs/m2sdr_soapy_bringup.md 8.4).
echo N | sudo tee /sys/module/drm_kms_helper/parameters/poll >/dev/null 2>&1 && log "DRM KMS polling disabled"
rm -f $STOP; log "guardian_hw start"; start_gnb
last_reg=""; last_puxch=0; last_busy=0; last_late=$((16#$(reg 0x1580c | cut -c3-))); last_health=$(date +%s)
while [ ! -f $STOP ]; do
  sleep 30
  if ! pgrep -x gnb >/dev/null; then log "gNB died -> restart"; start_gnb; continue; fi
  puxch=$(grep -ac "PUxCH request late" /tmp/gnb.log 2>/dev/null); rflate=$(grep -ac "RF: late" /tmp/gnb.log 2>/dev/null); busy=$(grep -ac "UL processor is busy" /tmp/gnb.log 2>/dev/null)
  late=$((16#$(reg 0x1580c | cut -c3-))); dl=$(( late - last_late )); last_late=$late
  dp=$(( puxch - last_puxch )); last_puxch=$puxch; db=$(( busy - last_busy )); last_busy=$busy
  reg_state=$(A shell dumpsys telephony.registry | grep -m1 -o -E "registrationState=[A-Z_]+" | cut -d= -f2)
  [ "$reg_state" != "$last_reg" ] && { log "phone: $reg_state (PUxCH-late +$dp, gate late +$dl, RF late total $rflate)"; last_reg=$reg_state; }
  # Unhealthy: sustained PUxCH-late storm or the gate dropping the host's frames.
  # "UL processor is busy" at a sustained rate = the UL PDU slot repositories are jammed (never released after
  # a late request at start); the UL is dead even though the phone stays registered.
  if [ "$dp" -gt 2000 ] || [ "$dl" -gt 500 ] || [ "$db" -gt 1500 ]; then log "unhealthy: PUxCH-late +$dp/30s, UL-busy +$db/30s, gate late +$dl/30s -> restart"; start_gnb; fi
done
log "guardian_hw stopped"
