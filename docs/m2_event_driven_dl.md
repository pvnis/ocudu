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

## Fix directions (not yet implemented)

- Bound the idle burn hard: a much smaller cap *and* a busy-spin (`pause`/`yield`)
  instead of `sleep_for`, so the wait cannot overshoot. Trades a little CPU for a
  deterministic bound.
- Skip the wait on slots the MAC did not populate: pass a per-slot "DL grant present"
  hint from the MAC/scheduler down to the lower PHY so the busy-wait only arms on slots
  that actually carry a grid (removes the 83 % idle-slot burn entirely).
- Keep H ≥ 2 and accept M2's clean operating point (DL ~3.95 ms p50 on this UE); H only
  helps at H=1, which this host's jitter cannot support.

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
| `OCUDU_LPHY_EVENT_DRIVEN_MAX_US` | busy-wait cap per slot | one slot |
| `OCUDU_DL_PIPELINE_STATS` | report period (s) for M-offset + busy-wait stats | off |

`scripts/m2sdr/guardian_hw.sh` passes these through via its `EXTRA_ENV` variable.
