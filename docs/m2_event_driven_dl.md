# M2: event-driven downlink, and the H-vs-busy-wait stability limit

This documents the M2 ("event-driven symbol transforming") port into OCUDU's lower
PHY, and — in detail — **why combining it with a small prepare-lead (H=1) is
unstable**, as measured on the M2SDR / PREEMPT-RT testbed (n78 TDD 20 MHz 30 kHz,
DDSU 2 ms, COTS OnePlus UE).

## Background: the two knobs

Per the aygong srsRAN latency model, the gNB DL one-way latency is roughly

    DL_oneway ≈ (H + M + 1) · 0.5 ms + UE_processing

- **H** = `radio_heads_prep_time` — how many slots ahead of the radio the lower PHY
  prepares DL samples. Env `OCUDU_LPHY_RADIO_HEADS_PREP_TIME` (default 3 ≈ 1 ms lead).
  Sets `rx_to_tx_max_delay` in `lower_phy_factory.cpp`.
- **M** = `nof_slot_tti_in_advance` (= `max_processing_delay_slots`) — how many slots
  ahead the MAC is notified. M2 drives this to 0.

**M2** (env `OCUDU_LPHY_EVENT_DRIVEN_DL=1`, `OCUDU_LPHY_DL_TTI_IN_ADVANCE=0`): instead
of pre-fetching the modulated grid a whole slot ahead, the DL baseband processor
notifies at M=0 and **busy-waits inside `process()`** for the modulated grid to land
in the PDxCH requests pool (`downlink_processor_baseband_impl::process`). Bounded by
`OCUDU_LPHY_EVENT_DRIVEN_MAX_US` (default = one slot) and gated to DL slots via the
TDD pattern (`is_dl_enabled`).

## Measured results (end-to-end DL one-way, four-timestamp probe)

| Config        | DL min | DL p50 | gate-late | Clean |
|---------------|--------|--------|-----------|-------|
| H=3, M=1 (pre-M2) | ~4.0 ms | 4.60 ms | 0 | yes |
| H=3 + M2      | 3.56 ms | 3.96 ms | 0 | yes |
| H=2 + M2      | 3.33 ms | 3.95 ms | 0 | yes |
| H=1 + M2      | 2.40 ms | 3.16 ms | storm | **no** |

M2 alone (at H=3) cuts the gNB M-offset from **520 µs → 60 µs p50** (measured directly
by `OCUDU_DL_PIPELINE_STATS`), with 0 loss. Reducing H from 3→2 gives no end-to-end
benefit; only H=1 shaves DL meaningfully — but gate-lates.

## Why H=1 + M2 is unstable (instrumented)

The busy-wait accounting (`gNB M2 busy-wait:` line, enabled with
`OCUDU_DL_PIPELINE_STATS`) at H=1, cap 300 µs, idle cell:

    entries=15001 timeouts=12438 (cap=300 us) avg=266 us max=487 us total=3983480 us/period
    Soapy TX lead: min 66-83 us        (vs ~437 us at H=1 WITHOUT M2)
    TX fine gate:  688-2748 late words/s discarded
    OCUDU RF-late: ~2.5/s

Mechanism, step by step:

1. **1500 DL slots/s enter the busy-wait.** DDSU has 3 DL-enabled slots per 4-slot
   period (D, D, and the special slot S with 6 DL symbols) → 1500 of 2000 slots/s.
2. **83 % (12438/15001) time out.** They are *idle* DL slots — no PDSCH is scheduled,
   so no grid ever arrives and the wait spins all the way to the cap. Pure lead burn.
3. **Each wait averages 266 µs, max 487 µs.** The max *exceeds* the 300 µs cap because
   `std::this_thread::sleep_for(poll_us)` overshoots badly under scheduler contention —
   an extra, uncontrolled jitter source on top of the cap.
4. **~40 % of the DL thread's wall-clock is spent spinning** (3.98 s of busy-wait per
   10 s). The busy-wait sits *inside* `process()`, before `transmit()`, so every µs
   burned is a µs of TX lead lost.
5. **TX lead collapses from ~437 µs (H=1 alone) to ~70 µs min.**
6. **With ~70 µs residual lead**, any host hiccup (SMI spikes up to ~224 µs seen in
   hwlat, or the sleep overshoot itself) pushes the transmit past the gate → the
   hardware TX fine gate discards late words and OCUDU logs RF-late.

At H=3 the ~1444 µs lead absorbs the same ~266 µs burn, so it is stable. The lead must
exceed (worst-case busy-wait burn + host jitter); H=1's budget is too small.

### Two compounding root causes

- **(A) Busy-waiting on idle DL slots.** 83 % of entries are a guaranteed full-cap burn
  for a grid that will never come. The lower PHY cannot tell in advance which DL slots
  the MAC populated, so `is_dl_enabled` (TDD-based) is too coarse — it says "DL slot",
  not "DL slot with data".
- **(B) `sleep_for` overshoot.** The poll sleep overshoots the cap under load
  (max 487 µs vs a 300 µs cap), so even the bounded wait is not actually bounded.

## Fix implemented: the grant-present hint

Root cause (A) — busy-waiting on idle DL slots — is addressed by a per-slot
**grant-present hint**. `pdxch_processor_impl::handle_request` marks the slot of each
non-empty transmission request (`request_seen_marker`, before enqueuing modulation);
`pdxch_processor_baseband::request_seen(slot)` exposes it. The busy-wait then uses:

- a short **grace** window (`OCUDU_LPHY_EVENT_DRIVEN_GRACE_US`, default 200) for a
  request to appear — if none does, the slot is idle and the wait stops; and
- the full **cap** (`..._MAX_US`) for modulation to finish *once a request is seen*.

This decouples "wait for the grant to show up" (short; idle slots stop here) from "wait
for modulation once granted" (full cap; real grids never clipped). The stats line gains
`idle_stops` and a `seen:` latency (notify->request) to tune the grace.

### Measured effect (H=1)

Idle, grace=150, cap=300:

    entries=15001 idle_stops=12438 timeouts=0 avg=139 (was 266) total=2.08M (was 3.98M) | seen: avg=39 max=140 us
    RF-fail 164 -> 0, TX lead min 70 -> ~200 us

The hint works: idle burn roughly halved, `timeouts=0`, RF-fail eliminated at idle.

**But H=1 is still not clean under load**, and tuning the grace is a lose-lose:

| grace | false idle-stops           | idle burn | RF-fail/load | probe loss |
|-------|----------------------------|-----------|--------------|------------|
| 150   | yes (seen max 156 > 150)   | 139 us    | 36           | 22/500     |
| 220   | no  (seen max 141 < 220)   | 194 us    | 42           | 50/500     |

- low grace -> real grants arriving after the grace are dropped (false idle-stop);
- high grace -> more lead burned per idle slot -> more gate-late.

### Why H=1 cannot be rescued (the budget)

The decisive datum: at M=0 the DL grant is not available until **~140 us** (max ~156 us
under load) after the slot boundary — the upper-PHY schedule->FAPI->grid path
(`seen: avg ~45 / max ~156 us`). The H=1 TX lead is only ~437 us, and it must cover

    grant delivery (~140 us) + idle-slot grace burn (~150-220 us) + modulation + SMI jitter (~224 us)

which overflows. **H=1 + M=0 is physically incompatible on this upper PHY** — a lead
budget, not a bug. The grant hint cannot manufacture lead that H=1 does not have.

At H=2 (lead ~938 us) and H=3 (~1444 us) the same terms fit, so M2 is clean there. The
grant hint remains worthwhile at any H (less wasted CPU, `timeouts=0`).

### Busy-spin option (OCUDU_LPHY_EVENT_DRIVEN_POLL_US=0)

`sleep_for` overshoots the cap/grace under scheduler contention (max 487 us for a 300 us
cap). Setting the poll interval to 0 switches the wait to a tight PAUSE busy-spin that
re-checks `steady_clock` each iteration, so the deadline is honored to sub-us.
**Only safe with CPU isolation** (a dedicated core): without `isolcpus` the spinning
FIFO TX thread can starve the lower-priority modulation thread if they share a core
(RT throttle is also removed by the tuning), so the grid never arrives and the slot is
lost. Use poll>0 (sleep) until the PHY cores are isolated.

### Host determinism tuning (scripts/ocudu_lowlat_tune.sh)

Applying the full low-jitter tuning (cpu_dma_latency=0, RT-throttle off, IRQ affinity,
watchdog/timer off, Wi-Fi power-save off; isolcpus staged for reboot) measurably cut the
jitter: RX-lag max 199 -> ~22 us. At H=1 under load it improved the DL *latency*
(p50 3.6 -> 2.90 ms, min 2.20 ms, fewer lost) **but did not reduce the gate-late**
(~38 RF-fail, unchanged). This confirms the H=1 overflow is dominated by the ~140 us
upper-PHY grant-delivery latency, not host jitter — host tuning tightens the
distribution but cannot create lead.

### Remaining options to actually reach H=1 (ranked)

1. **Decouple scheduling from the PHY pull (Tier 4):** notify the MAC one slot early so
   the grant is already in flight, but keep the sample pull event-driven. Removes the
   ~140 us grant-delivery latency from the busy-wait budget - the only change that
   attacks the dominant term. Upper-PHY work (splits the single M knob).
2. `isolcpus` + busy-spin (poll=0): removes preemption and sleep overshoot; helps the
   tail, does not remove the grant latency. Staged in GRUB; needs a reboot + --pin-threads.
3. Reduce host SMI jitter below ~50 us (firmware/BIOS; a laptop i7-9750H cannot - this is
   the platform ceiling).
4. Accept H=2/H=3 + M2 as the clean floor (DL ~3.95 ms p50 on this UE); use the SIM8200
   UE for the paper's 2 ms, which needs neither H=1 nor the risk.

## The 2 ms question

The paper's 2 ms DL used a **SIM8200EA-M2** (discrete Qualcomm X55 modem, lean and
deterministic DL floor), not a COTS phone. On the OnePlus the modem floor is ~1.5 ms,
so the clean DL floor here is ~3.3–3.95 ms (H=2/3 + M2). H=1 can touch 2.4 ms min but
only with DL impairment on this host. With a SIM8200 UE + M2 at H=2 (safe lead), this
gNB should reach ~2 ms cleanly — the gNB side is no longer the bottleneck; the UE is.

## Env reference

| Env | Meaning | Default |
|-----|---------|---------|
| `OCUDU_LPHY_RADIO_HEADS_PREP_TIME` | H, prepare-lead in slots | 3 |
| `OCUDU_LPHY_EVENT_DRIVEN_DL` | enable the M2 busy-wait | off |
| `OCUDU_LPHY_DL_TTI_IN_ADVANCE` | override M (0 = event-driven) | = max_proc_delay |
| `OCUDU_LPHY_EVENT_DRIVEN_POLL_US` | busy-wait poll interval | 10 |
| `OCUDU_LPHY_EVENT_DRIVEN_GRACE_US` | grace for a request to appear before a DL slot is deemed idle | 200 |
| `OCUDU_LPHY_EVENT_DRIVEN_MAX_US` | busy-wait cap per slot (modulation wait once a request is seen) | one slot |
| `OCUDU_DL_PIPELINE_STATS` | report period (s) for M-offset + busy-wait stats | off |

`scripts/m2sdr/guardian_hw.sh` passes these through via its `EXTRA_ENV` variable.
