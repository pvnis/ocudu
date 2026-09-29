# OCUDU gNB on the LiteX-M2SDR (SoapySDR) — bring-up notes and checkpoint

Checkpoint 2026-09-29. A COTS phone (OnePlus Nord CE3 Lite, test SIM 001010123456741) attaches to an
OCUDU gNB running on a LiteX-M2SDR (AD9361, PCIe) through the SoapySDR radio driver, registers with
qcore, gets a PDU session and reaches the internet. Speedtest from the phone: 43 Mbit/s down,
18 Mbit/s up (20 MHz TDD n78, 1T1R, default 6D/1S/3U pattern; theoretical PHY peak ≈ 70/30).

Everything below is on the `soapy` branch of OCUDU and the `working-ue` branch of `~/m2sdr`
(HEAD 8b0244b5, formerly `main-port`). The operational log of the debugging session is in `build/captures/STATUS.md`
(untracked); this document is the durable summary.

## 1. Cell configuration

`configs/gnb_soapy_m2sdr.yml`:

| item | value | why |
|---|---|---|
| band / carrier | n78, `dl_arfcn 640000` = 3600.00 MHz, 20 MHz, SCS 30 kHz, 23.04 MSps | the EU-firmware OnePlus operating in the US only scans 3.55–3.98 GHz; at 3489 MHz it never listed the PLMN although the DL was bit-exact |
| PLMN / TAC / PCI | 001-01 / 1 / 101 | qcore `--mcc 001 --mnc 01` |
| `device_args` | `driver=LiteXM2SDR,ad9361_fir_profile=bypass,rx_worker_packets=2048,freq_corr_ppm=1.327` | FIR bypass; plugin RX decoupling worker; 1.327 ppm XO correction applied to both LOs in the OCUDU driver |
| gains | `tx_gain -10` (= 10 dB attenuation), `rx_gain 40` | the Soapy driver maps OCUDU TX gain to AD9361 attenuation |
| `expert_cfg.tx_mode` | `continuous` | the plugin's software-timed TX needs a gap-free DMA timeline |
| `prach.ra_resp_window` | 20 slots (10 ms, the validator's maximum) | the scheduler runs ~13 slots ahead of RX (see §3.3); the default 5 ms window closed before the PRACH indication arrived |
| `expert_phy.max_proc_delay` | 3 | with the 5 ms DL lead (below) keeps PRACH requests in time |
| `expert_phy.allow_request_on_empty_uplink_slot` | true | diagnostic, harmless: exercises the PUxCH request path on every UL slot |
| `log` | all warning | PHY/MAC `info` was used for diagnosis; too chatty with an attached UE |
| environment | `OCUDU_LPHY_RX_TO_TX_DELAY_US=5000` | DL is generated 5 ms ahead of RX instead of 1 ms so the DMA ring never runs dry |

Do **not** add `expert_execution` affinities/partitions or change thread priorities (§6).

## 2. What changed in OCUDU

All in the working tree of branch `soapy`, committed with this document.

### 2.1 `lib/radio/soapy/` — the SoapySDR radio driver

* **Sample format.** The M2SDR carries 12-bit samples LSB-aligned in int16. TX scales `>>4` with
  clamping (`to_device_sample()`, `DEVICE_SAMPLE_SHIFT = 4`), RX scales `<<4`. Without this the DL
  was 24 dB too quiet and clipped.
* **Gain mapping.** OCUDU TX gain in dB is applied as AD9361 attenuation (`-gain_dB`); RX gain is set
  as-is. Frequencies and gains are re-applied after stream start (the plugin resets them), and
  `get_rf_settings()` reads them back for the log.
* **Frequency correction.** `freq_corr_ppm=<ppm>` in `device_args` tunes both LOs to
  `f·(1 − ppm·1e-6)`; the measured XO error of this board is +1.327 ppm.
* **Continuous TX mode** is accepted (the validator used to reject it).
* **TX write retries.** A timed-out `writeStream` (ring backpressure) is retried up to
  `MAX_WRITE_TIMEOUT_RETRIES = 100` × 200 µs; other errors drop the buffer and report an event.
* **RX start-up backlog drain.** `start()` reads and discards whatever the plugin has queued so the
  lower PHY starts near real time.
* **Runtime RX timestamp shift (the key fix, §3.4).** `SIGUSR2` makes the RX stream read
  `/tmp/ocudu_soapy_rx_ts_shift` (absolute shift in samples; path from
  `OCUDU_SOAPY_RX_TS_SHIFT_FILE`) and relabel RX samples as `hardware_ts + shift`. A negative shift
  drops that many samples once, a positive one zero-fills, so the labels handed to the lower PHY stay
  continuous and the lower PHY never sees a jump.
* **RX hardware-timestamp discontinuity detector**: logs any jump > 2 samples between consecutive
  reads (±1–2 samples is ns→sample rounding jitter of the plugin's timestamps).
* **Debug capture.** `OCUDU_SOAPY_TX_DUMP=<base>` / `OCUDU_SOAPY_RX_DUMP=<base>` write
  `OCUDU_SOAPY_*_DUMP_MS` (default 100 ms) of CS16 samples after `OCUDU_SOAPY_*_DUMP_SKIP_S` seconds
  to `<base>.<generation>`, printing the first sample's hardware timestamp; `SIGUSR1` starts a new
  generation. Console prints of TX lead / RX lag versus hardware time every 5 s. Optional
  `OCUDU_SOAPY_TX_TRACE` / `OCUDU_SOAPY_RX_TRACE` timing traces.
* SoapySDR's log handler prints to stdout with a timestamp instead of feeding the OCUDU logger from
  plugin threads (that caused stalls).

### 2.2 `lib/phy/lower/lower_phy_factory.cpp`

* `OCUDU_LPHY_RX_TO_TX_DELAY_US` (default 1000, minimum 1000) sets how far ahead of the last received
  sample the DL is generated (`rx_to_tx_max_delay`). 5 ms is needed here.
* RX buffer pool floor raised from 4 to 16 buffers.

### 2.3 `lib/phy/lower/processors/resource_request_pool.h`

`request_array_size` 16 → 64. The lower PHY keeps pending PUxCH requests in a ring indexed by slot.
With a 5 ms lead + `max_proc_delay 3` the MAC issues UL requests ~13 slots ahead; on the 16-entry
ring a request landed on the same index as an older slot and was evicted ("PUxCH request late",
"Discarded uplink slot", then "Invalid UL CRC … Nonexistent tc-rnti" 128 ms later). Msg3 was never
demodulated at all before this change.

### 2.4 `apps/units/flexible_o_du/o_du_high/du_high/` — `additional_bands`

New cell option listing extra NR bands to advertise in SIB1's `frequencyBandList`. It was a
diagnostic for the phone's band scan and is not needed for the attach; kept because it is harmless
and occasionally useful.

### 2.5 `scripts/m2sdr/` and `configs/gnb_soapy_m2sdr.yml`

Copies of the operational tooling that ran in `build/captures/` (paths inside still point there):

* `guardian.sh` — starts the gNB with dumps at 6 s, measures the TX→RX offset with `align.py`,
  applies the RX timestamp shift, re-measures and accepts only a residual within ±30 samples; then
  every 60 s keeps the gNB alive, detects plugin re-anchors (`TX timeline anchored to hardware` on
  the console) and re-aligns in place (residual ≤ 4000 samples) or restarts, and restarts on silent
  failures (|TX lead| > 20 ms or > 2000 "PUxCH request late" per 2 min). Stop with
  `touch build/captures/guardian.stop`.
* `align.py <tx.cs16> <rx.cs16> <txts> <rxts> [ssb_off]` — correlates the PSS in the TX and RX dumps
  and prints the RX-minus-TX offset in samples.
* `ulscan.py`, `ulalign.py` — per-slot UL burst finder / spectrum, and burst start versus the TX
  slot grid on the shared clock (the tools that located the phone's Msg3 in the captures).
* `restart_gnb.sh`, `guardian_ctl.sh` — safe stop/start of guardian and gNB. They exist because
  `pkill -f`/`pgrep -f` with a pattern that appears in your own command line kills the shell
  (exit 144); never do that inline.
* `pin_threads_DISABLED.sh` — kept as a record of what not to do (§6).

## 3. The problems, in the order they were found

### 3.1 Nothing decoded on the phone side (fixed earlier)

12-bit sample scaling, TX-gain-as-attenuation and gain re-apply after stream start. The DL was then
verified bit-exact over the air (own RX capture decoded against OCUDU's and srsRAN's encoders:
PSS/SSS→PCI, MIB, SIB1, PDCCH).

### 3.2 Phone never saw the cell at 3489 MHz

US scan range of the EU firmware. Moving to 3600 MHz made the phone list "Test PLMN 1-1 5G" / camp.

### 3.3 gNB received no preambles: "PRACH request late" on every occasion

The DL lead (`rx_to_tx_max_delay`) and `max_proc_delay` set how far ahead of RX the MAC runs. At
1 ms lead the DMA ring underran; at 5 ms lead with the default `max_proc_delay 5` the PRACH
requests arrived after the lower PHY had passed the occasion. 5 ms + `max_proc_delay 3` gives
< 1 % late. Consequences of the resulting ~13-slot scheduler lead had to be fixed too: the RAR
window (10 ms) and the request ring (64 slots).

### 3.4 Msg3 always failed: the TX emission offset, hidden by PRACH aliasing

This is what blocked the attach for two days and what the "GPSDO is mandatory" conclusion got wrong.

The plugin's software-timed TX radiates the DL at a per-start offset from the stamped time (measured
with `align.py`: −7587, −767, 715, 3074, 5500, 17407 samples …, occasionally a whole 22.76 ms DMA
ring lap). The phone synchronises to the *radiated* DL, so its uplink arrives that offset late with
respect to the gNB's RX slot grid. PRACH format B4 is twelve repetitions of a 768-sample symbol,
so the detector aliases delays modulo 768 samples: 3074 ≈ 4×768 was reported as `ta=0.26 µs`, the
RAR went out with TA≈0 and the PUSCH (CP 2.3 µs) was demodulated ~133 µs early → `crc=KO
sinr=−25 dB`. IQ captures during a RACH storm showed the SSB leakage at TX-grid+1656+3074 and the
phone's 14-symbol Msg3 bursts at TX-grid+3074−300−TA, where 300 samples is exactly N_TA_offset
(25600 Tc), i.e. the phone did everything right. The contention-resolution timer expiring exactly
64 ms after the last Msg3 grant proved it was decoding RAR and DCI 0_0 and transmitting.

Fix: measure the offset at start-up and apply it as a runtime RX timestamp shift (§2.1). This is a
per-start `time_alignment_calibration`; the static YAML knob cannot follow a random offset. The
first shifted start attached within 2 s (`off=−297 → post-shift 0 → registrationState=HOME`).

### 3.5 Re-anchors, storms and silent lags (ongoing, plugin side)

After a host hiccup the plugin re-anchors its TX timeline, which changes the emission offset and
drops the UE. The guardian catches it within 60 s and re-aligns in place (the phone usually stays
registered) or restarts (1–2 min). Rarer and worse: the plugin's TX worker stalls in ~1 s chunks and
re-anchors on every write (5000–7000/min) until the gNB is restarted; and some starts come up with
the TX thread ~0.8 s behind hardware time, which makes every UL request late without any re-anchor.
The guardian's lead / PUxCH-late health check restarts those. All three need the plugin fix (§4.3).

### 3.6 Why this radio needs the work-around and a USRP does not

OCUDU's lower PHY makes two assumptions about the radio, both of which UHD/USRP (and LimeSDR,
bladeRF in metadata mode) satisfy by construction and which the M2SDR plugin on this branch only
approximates in software:

1. **A sample stamped T is on the air at T** (up to a fixed, device-specific group delay, which is
   what the small constant `ru_sdr.time_alignment_calibration` in the stock configs corrects and
   which never changes between runs).
2. **TX and RX are stamped by the same clock**, so "UL slot n arrives at RX time = TX time of slot n
   − N_TA_offset" holds to the sample and the timing-advance loop only has to remove propagation
   delay.

*Hardware-timed radios.* Every TX packet carries a timestamp in its header; the FPGA holds the samples
until its tick counter reaches that time and emits them on exactly that sample. RX packets are
stamped by the same counter as captured. Lateness is handled without moving anything: a packet that
arrives after its time is dropped and reported as an async late/underflow event, and the next
on-time packet is emitted exactly when stamped. A host hiccup costs one slot of DL, never the
alignment.

*The M2SDR plugin (branch `working-ue`).* The gateware's TX DMA reader streams a free-running
256 × 2048-sample ring (LOOP mode) with no timestamp compare in hardware. The plugin emulates timing:
it observes "the hardware has consumed N buffers at board time T" once (or again after an underflow)
and from that anchor decides where a stamped buffer must land in the ring. Three consequences, none
of which a USRP shows:

* **A per-start emission offset.** The anchor has one-buffer granularity (2048 samples ≈ 89 µs)
  plus the race of when the DMA reader actually started, plus a whole-lap ambiguity (22.76 ms).
  Commit 8b0244b5 reduced it to about ±100 µs when no lap intervenes, but it is still a random
  constant per start. OCUDU measures it from the SSB loopback (`align.py`) and applies it as the
  runtime RX timestamp shift — effectively a per-start `time_alignment_calibration`.
* **The hardware cannot hold samples.** If the writer is late the ring is emitted regardless
  (stale or zero data), so the plugin's only recourse is to re-anchor, which moves the timeline and
  instantly de-aligns an attached UE. The guardian detects re-anchors and re-measures.
* **Late-write storms.** Once behind, every subsequent write is late and the plugin re-anchors on
  each one (5000–7000/min observed) until the process is restarted; a hardware-timed stream has no
  equivalent failure mode.

RX on the M2SDR is hardware-stamped, so RX and true air time agree; it is TX that floats — and PRACH
format B4's twelve identical symbol repetitions alias that float modulo 33 µs, which is why random
access looked healthy while every PUSCH missed (§3.4).

Other radios for comparison: PlutoSDR/libiio has no timestamps at all and is in the same class or
worse; the M2SDR gateware previously had a hardware TimedTX arbiter (CSRs at 0x148xx) that this branch
removed in favour of the software timeline. The durable fix is to put the timestamp compare back in
hardware — a TX header the DMA reader honours (hold until time, drop if late) with PROG-mode DMA so
the lap race disappears — after which the OCUDU-side shift and guardian become unnecessary.

## 4. What changed in m2sdr (`~/m2sdr`, branch `working-ue`, by the m2sdr agent session)

The installed plugin `/usr/local/lib/SoapySDR/modules0.8/libSoapyLiteXM2SDR.so` is from 8b0244b5
(2026-09-28 19:48). The kernel module is insmod'd from `litex_m2sdr/software/kernel` (not DKMS —
rebuild and reload after a reboot). CPU governor must be `performance` (not persistent).

### 4.1 Plugin commits

* **8b0244b5 anchor the timed-TX timeline to measured DMA emission.** `initTimedTxTimeline()` used
  to anchor on a bare `getHardwareTime()` and assume the next sample was emitted then, giving an
  unmeasured per-start offset of several ms of either sign. It now anchors lazily against
  `(hw_count, sw_count)` from `m2sdr_get_stream_stats()` once the DMA reader has consumed a buffer,
  biased one buffer early so the residual is a positive delay (−120…+102 µs measured), and
  re-anchors whenever the underflow counter advances. It explicitly does *not* remove the one-lap
  race of the LOOP-mode 256-buffer ring (22.76 ms); that needs PROG-mode DMA with descriptor refill
  in the kernel driver. (OCUDU's measured offsets after this commit are consistent with it: a few
  hundred to a few thousand samples, sometimes a lap.)
* **f797e423 don't offset the TX destination twice.** `interleaveCS16/CF32/CS8` advanced the DMA
  destination by the caller's offset as well as the source; any write spanning more than one
  2048-sample DMA buffer (OCUDU writes 11520-sample slots) wrote past the buffer and segfaulted on
  the second slot. This was the first blocker of the OCUDU bring-up.
* **c70ea05f optional RX decoupling worker (`rx_worker_packets`).** A worker thread drains the DMA
  ring into a userspace ring and releases DMA buffers immediately, so an application stall no
  longer overflows the hardware ring (NR-Scope logged 1.1 M overflows in 110 s before).
  OCUDU uses `rx_worker_packets=2048`.
* **57c980bf carry user args through device enumeration** — `device_args` such as
  `ad9361_fir_profile`/`rx_worker_packets` now reach the device.

### 4.2 Gateware / kernel

* **c35504ec run clk10 MMCM DPS in the sys domain** and **ed9d2c49 si5351: synchronize
  quasi-static control bits** — clocking fixes; regenerated `csr.h`/`mem.h`/`soc.h` for the kernel
  module.

### 4.3 Still open on the plugin side (reported to the m2sdr agent)

1. Exact TX emission time (the one-lap race and the residual offset) so OCUDU's shift becomes 0.
2. On a late write, re-anchor once and resynchronise (or drop/zero-fill to catch up) instead of
   re-anchoring on every write; look for the ~1 s stalls of the TX worker.
3. Consider making the RX timestamps and hardware time consistent at start so a run cannot begin
   with the TX thread 0.8 s behind.

## 5. Core network

Two working qcore modes (both verified 2026-09-29; the gNB re-establishes NGAP by itself after a qcore
restart, the phone needs an airplane-mode toggle to re-register):

* **DHCP / LAN mode (preferred, UE gets a real LAN address):**
  `cd ~/qcore && ./setup-routing wlp5s0` once after boot (it sets `ip_forward`, `proxy_arp` on the LAN
  interface, `rp_filter` off, creates `qcoretun`/`qcore_br0`/veths; "File exists"/"already assigned"
  messages on re-runs are harmless), then
  `sudo ./target/debug/qcore --mcc 001 --mnc 01 --local-ip 127.0.0.1 --lan-interface-name wlp5s0`.
  The phone gets 192.168.1.x from the LAN's DHCP server through qcore's relay and the host answers
  ARP for it. Without `proxy_arp` the LAN router has no return path and only host↔UE traffic works —
  that was the "no internet" symptom.
* **Self-managed / NAT mode:** `... --no-dhcp` (UEs on 10.255.0.0/24 via `veth2`) plus
  `sudo iptables -t nat -A POSTROUTING -s 10.255.0.0/24 -o wlp5s0 -j MASQUERADE` — qcore installs its
  own MASQUERADE for `eth0`, which does not exist on this host.

Neither the sysctls nor the iptables rule survive a reboot. SIM keys are in `~/qcore/sims.toml`
(corrected by the user on 2026-09-29; the defaults were wrong). The phone also requests an `ims` DNN,
which qcore declines with a 5GMM Status — harmless.

## 6. Things that made it worse (do not repeat)

* `chrt` SCHED_FIFO on the plugin worker threads → RX 12 ms behind hardware, TX worker frozen,
  7000 re-anchors/min after 17 min.
* `taskset` pinning of the plugin workers, and the YAML `expert_execution` partition
  (`main_pool_cpus 3-5`, `ru_cpus 1-2`) → the same TX freeze within 10 min; the partition alone →
  21 000 "PUxCH request late" in 30 min and the UE dropped. Leave scheduling default on this
  6-core host.
* Capping the TX write retries at 10 changed nothing for the better.
* The "GPSDO/10 MHz reference is mandatory" conclusion: it was the aliasing above.

## 7. How to bring it up from cold

1. Load the M2SDR kernel module, set the CPU governor to `performance`, check
   `SoapySDRUtil --find` sees the board.
2. Start qcore (§5): `./setup-routing wlp5s0` then qcore in DHCP mode.
3. `cd build && bash captures/guardian.sh` (or the copy in `scripts/m2sdr/` after fixing the paths)
   — it starts the gNB, aligns it and keeps it healthy; log in `build/captures/guardian.log`.
4. Phone: NR-SA only (`adb shell cmd phone set-allowed-network-types-for-users -s 0
   10000000000000000000`, reset by reboot); it camps and registers on 001-01 by itself. After
   several failed attempts it bars the cell for 300 s; a SIM reinsert or network-settings reset
   clears the forbidden-PLMN state.
