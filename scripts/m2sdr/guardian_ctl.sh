#!/bin/bash
# stop | patch-boost | start | status — never invoke with the pattern in an interactive command line.
C=/home/dmd/ocudu/build/captures; B=/home/dmd/ocudu/build
case "$1" in
  stop) for p in $(pgrep -f 'bash captures/guardian[.]sh'); do kill $p && echo "stopped guardian $p"; done ;;
  patch-boost)
    python3 - <<'EOF'
p='/home/dmd/ocudu/build/captures/guardian.sh'; s=open(p).read()
if 'boost_radio.sh' in s:
    print("already patched")
else:
    old='      [ -n "$POST" ] && [ "$POST" -ge -30 ] 2>/dev/null && [ "$POST" -le 30 ] 2>/dev/null && return 0\n'
    assert s.count(old)==1
    s=s.replace(old,'      [ -n "$POST" ] && [ "$POST" -ge -30 ] 2>/dev/null && [ "$POST" -le 30 ] 2>/dev/null && { $C/boost_radio.sh >> $G; return 0; }\n')
    old2='prev=""; last_anc=-1\n'
    assert s.count(old2)==1
    s=s.replace(old2, old2+'$C/boost_radio.sh >> $G\n')
    open(p,'w').write(s); print("guardian: boost after start + at launch")
EOF
    bash -n $C/guardian.sh && echo "syntax ok" ;;
  start) cd $B && S=captures/guardian && (setsid nohup bash $S.sh >/dev/null 2>&1 < /dev/null &) ; sleep 2; echo "guardian pids: $(pgrep -f 'bash captures/guardian[.]sh' | tr '\n' ' ')" ;;
  status) echo "guardian pids: $(pgrep -f 'bash captures/guardian[.]sh' | tr '\n' ' ')"; tail -3 $C/guardian.log ;;
esac
