#!/bin/bash
# Stress phases against the running gNB (guardian_hw.sh) with the phone attached.
# Per phase: gate late/stale deltas, lower-PHY failures, TX lead minimum, ping loss, phone state, throughput.
# Usage: [GNB_CONSOLE=<console log>] stress.sh [phase_seconds]   -> results in build/captures/stress_result.txt
# Stop guardian_hw.sh first: it restarts the gNB when the gate reports > 500 late frames per 30 s, which the CPU-load phases trigger.
D=${1:-60}; B=/home/dmd/ocudu/build; C=$B/captures; L=${GNB_CONSOLE:-$C/gnb_hwtimed_console.log}; R=$C/stress_result.txt
U=/home/dmd/m2sdr/litex_m2sdr/software/user/m2sdr_util; HOST=192.168.1.20; PH=192.168.1.28
reg(){ echo $((16#$($U reg-read $1 2>/dev/null | tail -1 | grep -o "0x[0-9a-f]*$" | cut -c3-))); }
state(){ timeout 10 adb shell dumpsys telephony.registry 2>/dev/null | grep -o -E "registrationState=[A-Z_]+" | tail -1 | cut -d= -f2; }
adbq(){ timeout $(( D + 20 )) adb shell "$@" 2>&1; }
: > $R
phase(){ # name, background-load-start-cmd, background-load-stop-cmd
  local n=$1 l0 s0 nl0 t0 lines0 pid
  l0=$(reg 0x1580c); s0=$(reg 0x15810); lines0=$(wc -l < $L); nl0=$(grep -ac 'TX fine gate' $L)
  eval "$2"
  ( adbq "ping -c $((D*3)) -i 0.33 -s 600 $HOST" > $C/stress_ping.txt ) &
  pid=$!
  sleep $D; wait $pid 2>/dev/null; eval "$3"
  local l1=$(reg 0x1580c) s1=$(reg 0x15810)
  local leads=$(tail -n +$((lines0+1)) $L | grep -a 'Soapy TX lead' | sed -E 's/.*are (-?[0-9]+) us.*/\1/' | sort -n | head -3 | tr '\n' ' ')
  local disc=$(tail -n +$((lines0+1)) $L | grep -ac 'RX discontinuity')
  local pl=$(grep -o "[0-9]*% packet loss" $C/stress_ping.txt)
  local rtt=$(grep -o "min/avg/max.*" $C/stress_ping.txt | cut -d= -f2)
  echo "[$n] gate late +$((l1-l0)) stale +$((s1-s0)) | ping loss ${pl:-?} rtt ${rtt:-?} | lowest TX leads(5s): $leads | rx-disc $disc | phone $(state) | gnb $(pgrep -x gnb >/dev/null && echo up || echo DOWN)" | tee -a $R
  [ -n "$4" ] && echo "     $4" | tee -a $R
}
spin(){ for i in $(seq $1); do ( while :; do :; done ) & echo $! ; done > $C/stress_spin.pids; }
unspin(){ xargs -r kill < $C/stress_spin.pids 2>/dev/null; sleep 1; }
echo "stress $(date -u +%T) phase=${D}s" | tee -a $R
phase baseline ":" ":"
phase "host cpu: 6 busy loops (all cores, normal prio)" "spin 6" "unspin"
phase "host cpu: 6 busy loops nice -n -5 (compete with TS threads)" "for i in 1 2 3 4 5 6; do ( sudo nice -n -5 sh -c 'while :; do :; done' & echo \$! ) ; done > $C/stress_spin.pids" "sudo xargs -r kill < $C/stress_spin.pids; sleep 1"
phase "host disk+memory: dd/gzip loop" "( while :; do dd if=/dev/urandom bs=1M count=200 2>/dev/null | gzip -c > /dev/null; done ) & echo \$! > $C/stress_bg.pid" "kill \$(cat $C/stress_bg.pid) 2>/dev/null; pkill -x gzip; sleep 1"
phase "DL throughput (host -> phone, tcp)" "( nc -l -p 5001 -q0 < /dev/zero > /dev/null & echo \$! > $C/stress_bg.pid ); ( sleep 2; timeout $((D-4)) adb shell 'nc $HOST 5001 > /dev/null' ) &" "kill \$(cat $C/stress_bg.pid) 2>/dev/null; pkill -f 'nc -l -p 5001'; sleep 1"
phase "UL throughput (phone -> host, tcp)" "( timeout $((D)) nc -l -p 5002 > $C/stress_rx.bin & echo \$! > $C/stress_bg.pid ); ( sleep 2; timeout $((D-4)) adb shell 'nc $HOST 5002 < /dev/zero' ) &" "kill \$(cat $C/stress_bg.pid) 2>/dev/null; pkill -f 'nc -l -p 5002'; echo \"UL bytes: \$(stat -c %s $C/stress_rx.bin) (\$(( \$(stat -c %s $C/stress_rx.bin) * 8 / $((D-4)) / 1000 )) kbit/s)\" > $C/stress_ul.txt; rm -f $C/stress_rx.bin" ""
cat $C/stress_ul.txt | tee -a $R
phase "Wi-Fi + CPU: host bulk download" "( timeout $D curl -s -o /dev/null http://speedtest.tele2.net/1GB.zip & ) " ":"
phase "baseline again" ":" ":"
echo "done $(date -u +%T)" | tee -a $R
