#!/bin/bash
cd /home/dmd/ocudu/build
echo "before: gnb=$(pgrep -x gnb >/dev/null && echo up || echo down) guardian_pids=[$(pgrep -f 'bash captures/guardian[.]sh' | tr '\n' ' ')]"
for p in $(pgrep -f 'bash captures/guardian[.]sh'); do kill $p 2>/dev/null && echo "killed guardian $p"; done
timeout 20 sudo pkill -INT -x gnb
for i in $(seq 15); do pgrep -x gnb >/dev/null || break; sleep 1; done
pgrep -x gnb >/dev/null && sudo pkill -9 -x gnb && sleep 1
echo "gnb stopped: $(pgrep -x gnb >/dev/null && echo NO || echo yes)"
echo "$(date -u +%H:%M:%S) === manual restart ===" >> captures/guardian.log
S=captures/guardian
setsid nohup bash $S.sh >/dev/null 2>&1 < /dev/null &
disown
sleep 2
echo "after: guardian_pids=[$(pgrep -f 'bash captures/guardian[.]sh' | tr '\n' ' ')]"
