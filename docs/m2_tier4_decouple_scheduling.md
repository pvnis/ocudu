# Tier-4 plan: decouple scheduling-lead from the PHY pull (robustly clean H=1)

## UPDATE (2026-10-04): the decouple idea does NOT hold — implemented as fractional H instead

Tracing the code before implementing killed the premise below. In
`downlink_processor_baseband_impl::process()` the **pull is always the current slot**
(`process_slot(slot)`); `nof_slot_tti_in_advance` (M) only shifts the **notify**
(`on_tti_boundary(slot + M)`). There is only one timeline knob — *when the MAC is asked
to schedule* — and notify→pull = M×0.5 ms. So a `sched_lead` ≥ 1 is not a decouple; it is
exactly M=1 and gives back the 0.5 ms M2 bought. And a "fast empty-slot signal" doesn't
help either: at M=0 the MAC's dl_tti_request (even an empty one) only arrives ~140 µs
after the notify — that IS the scheduler+FAPI decision latency, and it can't be known
sooner without scheduling earlier (= M≥1 = the latency back). The ~140 µs is irreducible
at M=0 from the lower PHY; the only way to lower it is to speed up the MAC/FAPI path
itself (a separate upper-PHY effort).

**What we did instead — fractional H (the lead-budget knee).** Rather than remove the
~140 µs, give the event-driven pipeline *just enough lead* to absorb it. `H`
(`radio_heads_prep_time`) is now a float (`OCUDU_LPHY_RADIO_HEADS_PREP_TIME`,
`lower_phy_factory.cpp`): `rx_to_tx_max_delay = srate_kHz·(H−1)/slots_per_sf + tx_offset`,
so H between 1 and 2 sets a sub-slot lead. Swept at M=0 + busy-spin + grace=200 on
isolated cores:

| H   | TX lead (idle) | RF-fail (load) | DL one-way min |
|-----|----------------|----------------|----------------|
| 1.0 | ~230 µs        | 5              | 2.20 ms        |
| 1.3 | ~400 µs        | 2              | 2.56 ms        |
| 1.4 | ~510 µs        | **0**          | **2.82 ms**    |
| 1.5 | ~616 µs        | 0              | 2.85 ms        |
| 2.0 | ~938 µs        | 0              | 3.33 ms        |

**The clean knee is H≈1.4** — 0 gate-late under load at DL one-way min ~2.82 ms, ~0.5 ms
below H=2 and clean where H=1 was not. H=1.5 is the same with more margin. (p50/loss are
COTS-channel noise per attach; min and RF-fail are the reliable, phone-independent-ish
signals.) Recommended: **H=1.5 operational** (clean + margin), pending a longer soak;
H=1.4 is the aggressive edge. To go below ~2.8 ms cleanly needs the MAC/FAPI ~140 µs
reduced (option 2 below) or the SIM8200 UE.

Everything below is the ORIGINAL plan, kept for the reasoning that led here; the
`sched_lead` approach itself is superseded by fractional H.

## Problem recap

M2 (event-driven downlink) drives the PHY notify to the current slot (M=0) and
busy-waits for the modulated grid. Combined with a small prepare-lead (H=1) this gets
DL one-way down to ~2.2 ms min, but it gate-lates: with the full determinism stack
(isolcpus + busy-spin + IRQ pinning + grace tuning) we cut it from ~38 to **5** RF-fail
under load, not to 0.

The instrumentation (`gNB M2 busy-wait: ... seen: avg=.. max=..`) pinned the residual on
one quantity: **at M=0 the DL grant is not available until ~140 µs (max ~189 µs under
load) after the slot boundary.** That is the upper-PHY path
`on_tti_boundary → MAC schedule → PDSCH build → FAPI → resource grid → RU → pdxch
handle_request`. The busy-wait's grace window must exceed that `seen` latency or real
grants are mis-classified as idle and dropped; but the grace burns the H=1 TX lead
(~440 µs), so grace + modulation + residual jitter overflow it on the worst slots.

No amount of host tuning removes this: the ~140–189 µs is *upper-PHY pipeline* latency,
not host jitter. **Tier-4 removes it from the busy-wait budget by decoupling two things
that OCUDU's single `M` knob currently ties together:**

- **when the MAC is told to schedule a slot** (the "scheduling lead"), and
- **when the lower PHY pulls the modulated grid for that slot** (the "PHY pull").

Today both are `nof_slot_tti_in_advance = max_processing_delay_slots`. M2 set it to 0,
which made the *pull* just-in-time (good) but also made the *notify* just-in-time (bad —
the grant now has to travel the whole pipeline inside the busy-wait).

## The idea

Notify the MAC **one slot early** (scheduling lead = 1) so it decides the grant and
kicks off modulation ~500 µs ahead — the grant is already in flight — but keep the PHY
**pull event-driven** (pull point = 0, busy-wait for the modulated grid). Then:

- the busy-wait no longer waits for *scheduling*; it only waits for *modulation* of a
  grant that was requested a slot ago, so `seen` collapses from ~140 µs to ~0 and the
  grace can shrink to a few tens of µs;
- the required lead drops to `modulation time + jitter` (~60–120 µs) instead of
  `grant-delivery + grace + modulation + jitter`, which **fits H=1's ~440 µs lead with
  margin** → robustly clean, and would also let H go below 1 eventually.

This is the srsRAN/O-RAN split-model timing: the scheduler always runs a fixed lead
ahead; only the *sample generation* is pulled late. M2-as-shipped conflated them.

## Where it lives in the code

Two offsets instead of one. Proposed knobs (env first, config later):

| concept | today | Tier-4 |
|---|---|---|
| MAC notify slot | `slot + nof_slot_tti_in_advance` (=0 under M2) | `slot + sched_lead` (≥1) |
| PHY grid pull | `process_slot(slot)` + busy-wait | unchanged (pull = slot, event-driven) |

### 1. `downlink_processor_baseband_impl::process()` (lib/phy/lower/processors/downlink)

- Add `sched_lead` (env `OCUDU_LPHY_SCHED_LEAD`, default = `nof_slot_tti_in_advance`
  so behavior is unchanged when unset).
- Fire the notify for `slot + sched_lead` (and set the metrics time_point
  `+ sched_lead * slot_ns`) — i.e. restore an advance on the *notify* only.
- Keep `process_slot(slot)` + the event-driven busy-wait on the **current** slot. The
  grant for `slot` was requested `sched_lead` slots ago, so by now it is modulated or
  nearly so → the busy-wait is short and `request_seen(slot)` is almost always already
  true on entry.
- The M-offset instrument already measures the realized pull latency; add nothing.

The subtlety: `on_tti_boundary` currently carries one slot that means both "prepare
this" and "it will be pulled next". Splitting the lead means the notify advances by
`sched_lead` while the pull stays at the current slot. The requests pool is already
keyed by absolute slot, so a grant requested for slot N at time (N−sched_lead) lands in
the pool under N and is pulled by `process_slot(N)` unchanged — no pool changes needed.

### 2. Verify the MAC/upper PHY tolerates sched_lead≥1 with event-driven pull

The upper PHY (`upper_phy_impl::on_tti_boundary`) and the DU-high scheduler already
expect to be notified a slot (or more) ahead — that is the normal `max_processing_delay`
regime. Tier-4 is really "run the scheduler at its normal lead, but generate the
baseband just-in-time", so the MAC side should need no change. Confirm:

- the grid for slot N arrives via `handle_request({slot=N})` before `process_slot(N)` —
  which the `request_seen` marker already lets us measure (expect `seen` ≈ 0 on entry);
- no new "pdxch request late" when the pull is at the current slot (the pool exchange and
  the self-healing late-path from the M2 commit already cover a grant that slips a slot).

### 3. Keep `max_processing_delay_slots` as the scheduler budget

`nof_slot_tti_in_advance` / `max_proc_delay` still sizes RU buffering
(`max_nof_prach_concurrent_requests`, `stop_nof_slots`, `rx_to_tx` elsewhere). Tier-4
adds `sched_lead` as the *notify* advance and leaves M (the pull quantization) at 0.
Cleanest mapping: `sched_lead = max(1, max_proc_delay)`, pull = 0 (event-driven), H free.

## Validation plan

Phone-independent first, with the existing instrument:

1. Set `OCUDU_LPHY_SCHED_LEAD=1`, keep `OCUDU_LPHY_DL_TTI_IN_ADVANCE=0`, event-driven on,
   H=1, isolated cores, busy-spin.
2. Expect in `gNB M2 busy-wait: ... seen:` — **`seen` avg/max drop from ~44/189 µs to
   near 0** (grant already in flight), `idle_stops` unchanged, and the achievable grace
   drops to ~30–50 µs.
3. Expect **RF-fail → ~0 under load** (the 5 residual were the `seen > grace` slips).
4. Then the four-timestamp probe: DL min should stay ~2.2 ms, **p50/tail tighten** (fewer
   late slots), and it should hold under the 656 Mbps UDP blast with 0 gate-late.
5. Regression: run H=3 + sched_lead default (unset) and confirm byte-identical stock
   behavior; run M2 at H=3 and confirm unchanged.

## Risks / open questions

- **MAC assumptions about the notify→grid deadline.** At sched_lead=1 the MAC has one
  slot (500 µs) to produce the grid, same as a normal max_proc_delay=1 cell — low risk,
  but watch for `on_pdxch_request_late` / `modulator is busy` under heavy DL.
- **Does the scheduler actually start work at the notify, or does it have its own
  internal lead?** If the DU-high already runs further ahead, sched_lead=1 may be enough;
  if it needs more, sched_lead=2 trades a little latency for safety. The `seen` stat
  makes this measurable in one run.
- **Interaction with H.** H (prepare-lead) and sched_lead are now independent; the TX
  lead must still cover `modulation + jitter`. With Tier-4, H=1 should fit; H can
  potentially go to a fractional lead later.
- **Scope.** This is a lower-PHY change (one file, two env knobs) *if* the MAC needs no
  change — which the analysis suggests. If the single-slot `on_tti_boundary` contract is
  load-bearing in the upper PHY in a way that assumes pull==notify, it grows.

## Expected payoff

Robustly clean H=1 (gate-late → 0 under load), DL one-way min ~2.2 ms *without*
impairment, and p50 pulled toward the min. On the SIM8200 UE this plus H=2 is the
straightforward path to the paper's clean 2 ms; on the OnePlus it removes the gNB as the
limiter and leaves only the ~1.5 ms modem floor.
