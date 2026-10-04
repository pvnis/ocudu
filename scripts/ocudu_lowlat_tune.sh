#!/bin/bash
# ocudu_lowlat_tune.sh - host tuning for LOW-LATENCY / LOW-JITTER OCUDU operation.
#
# Supersets scripts/ocudu_performance (governor, KMS poll, net buffers - the "don't throttle"
# layer) and adds the DETERMINISM layer that the stock script omits and that the event-driven
# downlink (M2) / small prepare-lead (H) depend on: C-state control, RT-throttle removal, IRQ
# affinity, watchdog/timer/THP, Wi-Fi power-save, and a boot CPU-isolation helper.
#
# Why each knob matters for the DL lead budget (see docs/m2_event_driven_dl.md): the TX lead
# must cover grant-delivery + modulation + HOST JITTER. These settings cut the jitter terms
# (C-state wakeups, preemption, IRQs, SMIs) that overflow a small lead.
#
# Usage:
#   sudo ./ocudu_lowlat_tune.sh                 apply the safe runtime tuning (default)
#   sudo ./ocudu_lowlat_tune.sh --pin-threads   also pin the running gNB threads to PHY cores
#   sudo ./ocudu_lowlat_tune.sh --grub          also add CPU isolation to GRUB (needs reboot)
#   sudo ./ocudu_lowlat_tune.sh --status        report current state only, change nothing
#   sudo ./ocudu_lowlat_tune.sh --revert        undo the runtime tuning
#
# Core layout (6-core i7-9750H default; override with env): housekeeping gets the OS + IRQs,
# PHY cores are reserved for the gNB real-time threads.
HOUSEKEEPING_CPUS="${HOUSEKEEPING_CPUS:-0-1}"
PHY_CPUS="${PHY_CPUS:-2-5}"
PIDFILE=/run/ocudu_cpu_dma_latency.pid

set -o nounset
GREEN=$'\e[32m'; YEL=$'\e[33m'; RED=$'\e[31m'; RST=$'\e[0m'
ok(){   echo "${GREEN}[ok]${RST}   $*"; }
warn(){ echo "${YEL}[warn]${RST} $*"; }
skip(){ echo "${YEL}[skip]${RST} $*"; }
info(){ echo "       $*"; }

need_root(){ [ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }; }

# ---------------------------------------------------------------------------
# "Don't throttle" layer (same as ocudu_performance).
# ---------------------------------------------------------------------------
apply_governor(){
  echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 \
    && ok "CPU governor = performance (no frequency-scaling latency)" || warn "could not set governor"
}
apply_kms(){
  if echo N | tee /sys/module/drm_kms_helper/parameters/poll >/dev/null 2>&1; then
    ok "DRM KMS polling disabled (removes a periodic poll stall)"
  else skip "DRM KMS polling (module not present)"; fi
}
apply_net(){
  # NOTE: data-plane only (qcore / Ethernet). The M2SDR is PCIe; this does NOT affect lower-PHY timing.
  sysctl -q -w net.core.wmem_max=33554432 net.core.rmem_max=33554432 \
                net.core.wmem_default=33554432 net.core.rmem_default=33554432
  ok "net buffers = 32 MB (data-plane only; irrelevant to PCIe M2SDR timing)"
}

# ---------------------------------------------------------------------------
# Determinism layer (the gap vs ocudu_performance).
# ---------------------------------------------------------------------------

# Hold /dev/cpu_dma_latency = 0 so CPUs never enter deep C-states (deterministic wakeups).
# Must keep an fd open for the lifetime of the setting; a background holder does that.
apply_cstates(){
  if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE" 2>/dev/null)" 2>/dev/null; then
    ok "C-states already pinned (holder pid $(cat "$PIDFILE"))"; return
  fi
  setsid bash -c 'exec 3<>/dev/cpu_dma_latency || exit 1; printf "\x00\x00\x00\x00" >&3; while :; do sleep 86400; done' &
  echo $! > "$PIDFILE"
  sleep 0.2
  kill -0 "$(cat "$PIDFILE")" 2>/dev/null \
    && ok "C-states pinned: /dev/cpu_dma_latency = 0 (holder pid $(cat "$PIDFILE"))" \
    || warn "could not pin cpu_dma_latency"
}

# Remove the RT bandwidth throttle (default throttles SCHED_FIFO to 95% / 1 s). A busy-waiting
# FIFO thread (the M2 event-driven poll) must not be throttled.
apply_rt_throttle(){
  sysctl -q -w kernel.sched_rt_runtime_us=-1 \
    && ok "RT throttle removed (kernel.sched_rt_runtime_us=-1; FIFO threads can run 100%)" \
    || warn "could not set sched_rt_runtime_us"
}

# Per-core lockup watchdogs and timer migration add periodic timer work on every core.
apply_watchdog_timer(){
  sysctl -q -w kernel.watchdog=0 2>/dev/null && ok "kernel lockup watchdog disabled" || skip "watchdog"
  sysctl -q -w kernel.timer_migration=0 2>/dev/null && ok "timer migration disabled" || skip "timer_migration"
  if [ -w /proc/sys/kernel/nmi_watchdog ]; then
    echo 0 > /proc/sys/kernel/nmi_watchdog && ok "NMI watchdog disabled"
  fi
}

# khugepaged compaction causes sporadic multi-100us stalls.
apply_thp(){
  if [ -w /sys/kernel/mm/transparent_hugepage/enabled ]; then
    echo never > /sys/kernel/mm/transparent_hugepage/enabled
    echo never > /sys/kernel/mm/transparent_hugepage/defrag 2>/dev/null
    ok "transparent hugepages = never (no khugepaged stalls)"
  else skip "THP (not present)"; fi
}

# Steer the M2SDR interrupt(s) onto the housekeeping cores so they never fire on a PHY core.
apply_irq_affinity(){
  local irqs; irqs=$(grep -i m2sdr /proc/interrupts | awk -F: '{print $1}' | tr -d ' ')
  if [ -z "$irqs" ]; then skip "m2sdr IRQ affinity (no m2sdr IRQ found)"; return; fi
  for i in $irqs; do
    if echo "$HOUSEKEEPING_CPUS" > "/proc/irq/$i/smp_affinity_list" 2>/dev/null; then
      ok "IRQ $i (m2sdr) pinned to cores $HOUSEKEEPING_CPUS"
    else warn "could not pin IRQ $i (threaded-IRQ kernels steer via the irq/ thread affinity instead)"; fi
  done
  # On PREEMPT_RT the m2sdr IRQ is a threaded handler (irq/NNN-m2sdr); pin that thread too.
  local tids; tids=$(ps -eLo tid,comm | awk '/irq\/[0-9]+-m2sdr/{print $1}')
  for t in $tids; do taskset -pc "$HOUSEKEEPING_CPUS" "$t" >/dev/null 2>&1 \
    && ok "IRQ thread $t pinned to $HOUSEKEEPING_CPUS"; done
}

# Wi-Fi power-save and roaming cause whole-host stalls (a reassoc was measured at 11.6 ms).
apply_wifi(){
  local w; w=$(ls /sys/class/net | grep -E '^wl' | head -1)
  [ -z "$w" ] && { skip "Wi-Fi power-save (no wireless iface)"; return; }
  if command -v iw >/dev/null && iw dev "$w" set power_save off 2>/dev/null; then
    ok "Wi-Fi power-save off on $w (consider wired to avoid reassoc stalls)"
  else skip "Wi-Fi power-save ($w; iw not available or unsupported)"; fi
}

# Optional: pin the RUNNING gNB real-time threads onto the PHY cores. Kept opt-in because a
# bad CPU partition has previously triggered PUxCH-late storms - verify after applying.
pin_gnb_threads(){
  local pid; pid=$(pgrep -x gnb | head -1)
  [ -z "$pid" ] && { warn "pin-threads: no gNB running"; return; }
  local n=0
  for tid in $(ls "/proc/$pid/task"); do
    comm=$(cat "/proc/$pid/task/$tid/comm" 2>/dev/null)
    case "$comm" in
      lower_phy_*|main_pool*|radio) taskset -pc "$PHY_CPUS" "$tid" >/dev/null 2>&1 && n=$((n+1));;
    esac
  done
  ok "pinned $n gNB RT threads to PHY cores $PHY_CPUS"
  warn "verify UL health after pinning (watch PUxCH-late / UL-busy); unpin by restarting the gNB"
}

# Boot-only CPU isolation. Reports the gap; with --grub, appends to GRUB (backed up) + update-grub.
ISO_ARGS="isolcpus=${PHY_CPUS} nohz_full=${PHY_CPUS} rcu_nocbs=${PHY_CPUS}"
check_isolation(){
  if grep -q "isolcpus=" /proc/cmdline; then
    ok "CPU isolation active: $(tr ' ' '\n' < /proc/cmdline | grep -E 'isolcpus|nohz_full|rcu_nocbs' | tr '\n' ' ')"
  else
    warn "NO CPU isolation on the kernel cmdline (timers/kernel threads/other procs can preempt PHY cores)"
    info "recommended boot args:  $ISO_ARGS"
    info "apply with:             sudo $0 --grub   (then reboot, and pin threads with --pin-threads)"
  fi
}
apply_grub(){
  local f=/etc/default/grub
  [ -f "$f" ] || { warn "no $f; add '$ISO_ARGS' to your bootloader manually"; return; }
  if grep -q "isolcpus=" "$f"; then ok "GRUB already has isolcpus; not modifying"; return; fi
  cp "$f" "$f.ocudu.bak.$(date +%s)" && info "backed up $f"
  sed -i "s/\(GRUB_CMDLINE_LINUX_DEFAULT=\"[^\"]*\)\"/\1 $ISO_ARGS\"/" "$f"
  if grep -q "isolcpus=" "$f"; then
    update-grub 2>/dev/null || grub-mkconfig -o /boot/grub/grub.cfg 2>/dev/null
    ok "added CPU isolation to GRUB: $ISO_ARGS"
    warn "REBOOT required; after reboot run '$0 --pin-threads' so the gNB lands on the isolated cores"
  else warn "could not edit $f automatically; add '$ISO_ARGS' by hand"; fi
}

revert(){
  need_root
  if [ -f "$PIDFILE" ]; then kill "$(cat "$PIDFILE")" 2>/dev/null; rm -f "$PIDFILE"; ok "released cpu_dma_latency (deep C-states allowed again)"; fi
  sysctl -q -w kernel.sched_rt_runtime_us=950000; ok "RT throttle restored (950000/1000000)"
  sysctl -q -w kernel.watchdog=1 kernel.timer_migration=1 2>/dev/null; ok "watchdog + timer migration restored"
  echo madvise > /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null; ok "THP restored (madvise)"
  info "governor/KMS/net buffers and any GRUB change are left as-is (re-run ocudu_performance or edit GRUB to revert those)"
}

status(){
  echo "== OCUDU low-latency tuning status =="
  echo "governor:        $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
  echo "kms poll:        $(cat /sys/module/drm_kms_helper/parameters/poll 2>/dev/null)"
  echo "cpu_dma_latency: $([ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE" 2>/dev/null)" 2>/dev/null && echo 'pinned=0 (holder '"$(cat "$PIDFILE")"')' || echo 'not pinned (deep C-states allowed)')"
  echo "sched_rt_runtime:$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null)"
  echo "watchdog:        $(cat /proc/sys/kernel/watchdog 2>/dev/null)  timer_migration: $(cat /proc/sys/kernel/timer_migration 2>/dev/null)"
  echo "THP:             $(cat /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null)"
  echo "net wmem/rmem:   $(cat /proc/sys/net/core/wmem_max) / $(cat /proc/sys/net/core/rmem_max)"
  local irqs; irqs=$(grep -i m2sdr /proc/interrupts | awk -F: '{print $1}' | tr -d ' ')
  for i in $irqs; do echo "irq $i affinity:  $(cat /proc/irq/$i/smp_affinity_list 2>/dev/null)"; done
  check_isolation
}

# ---------------------------------------------------------------------------
case "${1:-}" in
  --status) status; exit 0;;
  --revert) revert; exit 0;;
esac
need_root
apply_governor
apply_kms
apply_net
apply_cstates
apply_rt_throttle
apply_watchdog_timer
apply_thp
apply_irq_affinity
apply_wifi
[ "${1:-}" = "--pin-threads" ] && pin_gnb_threads
[ "${1:-}" = "--grub" ] && apply_grub
echo
check_isolation
echo
echo "Done. Run '$0 --status' to review, '$0 --revert' to undo the runtime bits."
