#!/usr/bin/env python3
"""Sustained DL / UL / bidirectional TCP load through the phone (adb + host sockets), sampled every 10 s.
Usage: sustained.py <seconds_per_phase> <gnb_console_log>   (run with the gNB up, guardian stopped)
Per sample: DL/UL throughput from the phone's rmnet_data1 byte counters, gate late/stale deltas (m2sdr_util),
lowest TX lead in the window, link-metric rows (HARQ) from the console, phone registration state."""
import socket, subprocess, sys, threading, time, re
PH = "192.168.1.28"; HOST = "192.168.1.20"
U = "/home/dmd/m2sdr/litex_m2sdr/software/user/m2sdr_util"
secs = int(sys.argv[1]); LOG = sys.argv[2]

def sh(cmd, t=15):
    try: return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=t).stdout
    except Exception: return ""
def reg(a):
    m = re.search(r"0x([0-9a-f]+)\s*$", sh(f"{U} reg-read {a} | tail -1")); return int(m.group(1), 16) if m else 0
def ctr():
    m = re.search(r"(?m)^\s*rmnet_data1:\s*(\d+)\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+(\d+)", sh("adb shell cat /proc/net/dev", 10))
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)   # phone rx (=DL), tx (=UL)
def state():
    m = re.findall(r"registrationState=([A-Z_]+)", sh("adb shell dumpsys telephony.registry", 10)); return m[-1] if m else "?"

stop = threading.Event()

def phase(name, dl, ul):
    procs = []
    if dl: procs.append(subprocess.Popen(f"exec nc -l -p 5101 -q0 < /dev/zero > /dev/null", shell=True))
    if ul: procs.append(subprocess.Popen(f"exec nc -l -p 5102 > /dev/null", shell=True))
    time.sleep(1)
    if dl: procs.append(subprocess.Popen(f"adb shell 'sleep 100000 | nc {HOST} 5101 > /dev/null'", shell=True))
    if ul: procs.append(subprocess.Popen(f"adb shell 'nc {HOST} 5102 < /dev/zero'", shell=True))
    time.sleep(4)
    print(f"== {name}: {secs}s  ({time.strftime('%H:%M:%S')})", flush=True)
    t0 = time.time(); c0 = ctr(); l0, s0 = reg("0x1580c"), reg("0x15810"); n0 = sum(1 for _ in open(LOG, errors="replace"))
    tot_late = 0; tprev = t0
    while time.time() - t0 < secs:
        time.sleep(10)
        t1 = time.time(); c1 = ctr(); l1, s1 = reg("0x1580c"), reg("0x15810")
        lines = open(LOG, errors="replace").read().splitlines()[n0:]; n0 += len(lines)
        leads = [int(x) for x in re.findall(r"Soapy TX lead: stamps are (-?\d+) us", "\n".join(lines))]
        dt = t1 - t0
        dl_k = (c1[0] - c0[0]) * 8 / (t1 - tprev) / 1000
        ul_k = (c1[1] - c0[1]) * 8 / (t1 - tprev) / 1000
        print(f"  t+{dt:4.0f}s DL {dl_k:7.0f} kbit/s UL {ul_k:7.0f} kbit/s | gate late +{l1-l0} stale +{s1-s0} | TX lead min {min(leads) if leads else 'n/a'} us | phone {state()}", flush=True)
        tot_late += l1 - l0; c0, l0, s0, tprev = c1, l1, s1, t1
    for p in procs: p.terminate()
    sh("pkill -x nc; adb shell pkill -x nc", 10)
    print(f"   {name} total late frames {tot_late}; gNB uptime ok={bool(sh('pgrep -x gnb').strip())}", flush=True)
    time.sleep(8)

phase("DL only", True, False)
phase("UL only", False, True)
phase("DL + UL simultaneously", True, True)
print("done", time.strftime("%H:%M:%S"), flush=True)
