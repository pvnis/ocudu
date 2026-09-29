#!/bin/bash
# Low-frequency overnight guardian. Keeps the gNB alive on a good-timing start, watches phone registration.
# Logs only on change/success. Stop by: touch /home/dmd/ocudu/build/captures/guardian.stop
set +e
B=/home/dmd/ocudu/build; C=$B/captures; cd $B
G=$C/guardian.log
A(){ timeout 20 adb "$@" 2>/dev/null; }
align_off(){ local g=${1:-0}; local TXTS=$(grep "TX dump: wrote.*g_tx.cs16.$g\b" $C/gnb_console.log|grep -o "ts=[0-9]*"|cut -d= -f2); local RXTS=$(grep "RX dump: wrote.*g_rx.cs16.$g\b" $C/gnb_console.log|grep -o "ts=[0-9]*"|cut -d= -f2); [ -z "$TXTS" ]&&{ echo 999999; return; }; (cd $C && timeout 60 python3 align.py g_tx.cs16.$g g_rx.cs16.$g "$TXTS" "$RXTS" -176 2>/dev/null|grep -o "offset -\?[0-9]* samples"|grep -o "\-\?[0-9]*")||echo 999999; }
# Apply the measured RX-minus-TX offset as a runtime RX timestamp shift (label = hw - off), then re-measure on dump gen 1.
apply_shift(){ local off=$1; echo $(( -off )) | sudo tee /tmp/ocudu_soapy_rx_ts_shift >/dev/null; sudo pkill -USR2 -x gnb; sleep 1.5; sudo pkill -USR1 -x gnb
  for i in $(seq 40); do grep -q "RX dump: wrote.*g_rx.cs16.1\b" $C/gnb_console.log && grep -q "TX dump: wrote.*g_tx.cs16.1\b" $C/gnb_console.log && break; sleep 1; done
  local post=$(align_off 1); echo "$(date -u +%H:%M:%S) applied rx_ts_shift=$(( -off )) -> post-shift off=$post" >> $G; POST=$post; }
start_good(){
  for r in $(seq 1 8); do
    timeout 20 sudo pkill -INT -x gnb; for i in $(seq 15); do pgrep -x gnb >/dev/null||break; sleep 1; done; pgrep -x gnb >/dev/null && timeout 10 sudo pkill -9 -x gnb; sleep 1
    [ -s $C/gnb_console.log ] && cp $C/gnb_console.log $C/gnb_console.prev.log
    rm -f $C/g_tx.cs16.* $C/g_rx.cs16.*; sudo rm -f /tmp/ocudu_soapy_rx_ts_shift
    timeout 20 sudo -b sh -c "env OCUDU_LPHY_RX_TO_TX_DELAY_US=5000 OCUDU_SOAPY_TX_DUMP=$C/g_tx.cs16 OCUDU_SOAPY_TX_DUMP_SKIP_S=6 OCUDU_SOAPY_RX_DUMP=$C/g_rx.cs16 OCUDU_SOAPY_RX_DUMP_SKIP_S=6 nohup $B/apps/gnb/gnb -c $B/gnb_soapy_m2sdr.yml > $C/gnb_console.log 2>&1 < /dev/null"
    for i in $(seq 60); do grep -q "RX dump: wrote.*g_rx.cs16.0" $C/gnb_console.log && grep -q "TX dump: wrote.*g_tx.cs16.0" $C/gnb_console.log && break; sleep 1; done
    local o=$(align_off 0); echo "$(date -u +%H:%M:%S) gNB restart try $r off=$o" >> $G
    if [ -n "$o" ] && [ "$o" -ge -40000 ] 2>/dev/null && [ "$o" -le 40000 ] 2>/dev/null; then
      apply_shift "$o"
      [ -n "$POST" ] && [ "$POST" -ge -30 ] 2>/dev/null && [ "$POST" -le 30 ] 2>/dev/null && { $C/boost_radio.sh >> $G; return 0; }
      echo "$(date -u +%H:%M:%S) post-shift residual $POST out of +-30 samples, restarting" >> $G
    fi
  done; return 1
}
# Re-measure the RX-vs-TX offset on a fresh dump and correct the runtime shift if the residual exceeds +-30 samples.
# Needed whenever the plugin re-anchors its TX timeline (after a TX underflow), which changes the DL emission offset.
realign(){ local g=$(grep -c "Soapy RX dump: wrote" $C/gnb_console.log); sudo pkill -USR1 -x gnb
  for i in $(seq 40); do grep -q "RX dump: wrote.*g_rx.cs16.$g\b" $C/gnb_console.log && grep -q "TX dump: wrote.*g_tx.cs16.$g\b" $C/gnb_console.log && break; sleep 1; done
  local r=$(align_off $g); local cur=$(sudo cat /tmp/ocudu_soapy_rx_ts_shift 2>/dev/null || echo 0)
  if [ -n "$r" ] && [ "$r" -ge -30 ] 2>/dev/null && [ "$r" -le 30 ] 2>/dev/null; then echo "$(date -u +%H:%M:%S) realign gen $g: residual $r, shift $cur kept" >> $G; return 0; fi
  if [ -z "$r" ] || [ "$r" -gt 4000 ] 2>/dev/null || [ "$r" -lt -4000 ] 2>/dev/null; then echo "$(date -u +%H:%M:%S) realign gen $g: residual $r out of range -> restart" >> $G; return 1; fi
  local new=$(( cur - r )); echo $new | sudo tee /tmp/ocudu_soapy_rx_ts_shift >/dev/null; sudo pkill -USR2 -x gnb; sleep 1.5
  g=$(grep -c "Soapy RX dump: wrote" $C/gnb_console.log); sudo pkill -USR1 -x gnb
  for i in $(seq 40); do grep -q "RX dump: wrote.*g_rx.cs16.$g\b" $C/gnb_console.log && grep -q "TX dump: wrote.*g_tx.cs16.$g\b" $C/gnb_console.log && break; sleep 1; done
  local r2=$(align_off $g); echo "$(date -u +%H:%M:%S) realign: residual $r -> shift $cur -> $new, post $r2" >> $G
  [ -n "$r2" ] && [ "$r2" -ge -30 ] 2>/dev/null && [ "$r2" -le 30 ] 2>/dev/null; }
prev=""; last_anc=-1
$C/boost_radio.sh >> $G
end=$(( $(date +%s) + 8*3600 ))
while [ $(date +%s) -lt $end ]; do
  [ -f $C/guardian.stop ] && { echo "$(date -u +%H:%M:%S) stopped by flag" >> $G; break; }
  pgrep -x gnb >/dev/null || { echo "$(date -u +%H:%M:%S) gNB down -> restarting good" >> $G; start_good; }
  # Silent failure modes: TX stamps far from hardware time, or the MAC/PHY flooding "PUxCH request late".
  lead=$(grep -a "TX lead" $C/gnb_console.log 2>/dev/null | tail -1 | sed -E 's/.*are (-?[0-9]+) us.*/\1/')
  late=$(sudo tail -c 4000000 /tmp/gnb.log 2>/dev/null | awk -v t="$(date -u -d '-2 minutes' +%Y-%m-%dT%H:%M:%S)" '$1>t' | grep -a -c 'PUxCH request late')
  if { [ -n "$lead" ] && { [ "$lead" -lt -20000 ] || [ "$lead" -gt 20000 ]; } 2>/dev/null; } || [ "${late:-0}" -gt 2000 ]; then
    echo "$(date -u +%H:%M:%S) unhealthy run: TX lead=${lead}us, PUxCH-late/2min=${late} -> restarting good" >> $G; start_good; continue
  fi
  anc=$(grep -c "TX timeline anchored to hardware" $C/gnb_console.log 2>/dev/null)
  if [ "$anc" != "$last_anc" ]; then
    [ "$last_anc" -ge 0 ] 2>/dev/null && echo "$(date -u +%H:%M:%S) TX re-anchor detected (count $last_anc -> $anc)" >> $G
    realign || { echo "$(date -u +%H:%M:%S) realign failed -> restarting good" >> $G; start_good; }
    anc=$(grep -c "TX timeline anchored to hardware" $C/gnb_console.log 2>/dev/null); last_anc=$anc
  fi
  reg=$(A shell dumpsys telephony.registry | grep -m1 -o -E "registrationState=[A-Z_]+")
  pci=$(A shell dumpsys telephony.registry | grep -m1 -o "mPci = [0-9]*")
  L=/tmp/gnb.log; rn=$(sudo awk -v t="$(date -u -d '-6 minutes' +%Y-%m-%dT%H:%M:%S)" '$1>t' $L 2>/dev/null|grep -a -o 'rnti=0x46[0-9a-f]*'|sort -u|wc -l)
  cur="$reg | $pci | rntis=$rn"
  if [ "$cur" != "$prev" ]; then echo "$(date -u +%H:%M:%S) $cur" >> $G; prev="$cur"; fi
  if echo "$reg" | grep -q "REGISTERED\|HOME" || [ "$rn" -gt 0 ]; then echo "$(date -u +%H:%M:%S) *** ATTACH/RACH: $cur" >> $G; fi
  sleep 60
done
echo "$(date -u +%H:%M:%S) guardian exit" >> $G
