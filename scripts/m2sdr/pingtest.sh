#!/bin/bash
# pingtest.sh <label> : waits for the phone to register on the running gNB, then 150 small pings to the LAN gateway.
for i in $(seq 40); do st=$(timeout 10 adb shell dumpsys telephony.registry 2>/dev/null | grep -m1 -o -E "registrationState=[A-Z_]+" | cut -d= -f2); [ "$st" = HOME ] && break; sleep 3; done
sleep 8
r=$(timeout 80 adb shell "ping -c 150 -i 0.3 192.168.1.1" 2>&1 | tail -2 | tr '\n' ' ')
echo "[$1] reg=$st $r | PUxCH-late=$(sudo grep -ac 'PUxCH request late' /tmp/gnb.log) UL-busy=$(sudo grep -ac 'UL processor is busy' /tmp/gnb.log) RF-late=$(sudo grep -ac 'Real-time failure in RF' /tmp/gnb.log) crcKO=$(sudo grep -ac 'crc=KO' /tmp/gnb.log)"
