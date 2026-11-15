# Domain Modelling — hexapods

The domain model of the hexapods playground: **seven domains inside one bounded context** (a single native binary). Established during story #8 planning, before the first line of application code, so that module boundaries follow intent instead of habit.

---

## Domain map

| Domain | Definition | Information it owns | Responsibility |
|---|---|---|---|
| **World** | What's physically true right now — bodies, joints, contacts, terrain | Model (from MJCF), sim state, step count, timestep | Step physics forward; reset |
| **Control** | Turns intent into actuation — what joints *should* do | Gait params, gait phase, commands, foot targets, parameter surface | Produce actuator targets each step; switch gaits |
| **Session** | One run of the binary, launch to exit | Fixed-step policy, pause, seed, epochs, active front-end | Convert wall time into ordered steps; mark epoch boundaries |
| **Events** | The domain's memory — physics-affecting inputs in order | Event vocabulary, journal, model fingerprint | Record events; replay event streams |
| **Presentation** | The window — how the world is seen and touched | Window, render context, camera, viewer input, HUD | Draw frames; turn raw input into viewer actions or events |
| **Verification** | Headless proof of behavior | Scenarios, criteria, metrics, verdicts | Run scenario, measure, pass/fail |
| **Evolution** | Search over Control's parameters, scored by Verification | Population, fitness history, champions | Breed, evaluate, retain |

### Domain arrival

- Story #8: World, Session, Events, Presentation
- Story #10: Verification
- Story #11: Control
- Phase 5 (stretch): Evolution

---

## Time: three clocks, three owners

Three different "clocks" live in three different domains. Conflating them is the
primary way this codebase would rot.

| Clock | Owner | What it is |
|---|---|---|
| **Wall time** | Session | Real elapsed seconds; the raw material of scheduling. Never appears in any other domain. |
| **Step clock** | World | The step count, epoch-local (restarts at reset). The only time physics knows. |
| **Gait phase** | Control | A function of steps × frequency driving gait trajectories. Unrelated to the other two. |

---

## Event rules

1. An **event** is a physics-affecting input, stamped by the step at which it
   *took effect* — not by when it occurred.
2. **Viewer actions** (camera, pause) affect seeing, never being. They are
   excluded from the event stream; replays ignore them by design.
3. A **reset** event starts a new **epoch**; step indices are epoch-local.
4. Events are only valid against the **model fingerprint** they were recorded
   with.
5. Keys, perturbations, slider changes, and gait switches are events; Control
   consumes them as commands.

## Boundary scenarios

- **Pause mid-fall** → viewer action, not an event. A ghost replaying your run
  never pauses.
- **Slider dragged while paused** → an event (it changes physics once resumed),
  stamped with the next executed step.
- **Gait switch mid-walk** → an event, consumed by Control as a command.
- **Camera orbit during a ghost replay** → viewer action; the replayed state
  stream is bit-identical regardless.

---

## Language

**Step**: one fixed-timestep advance of the World. *Avoid*: frame, tick
**Frame**: one render pass; Presentation-owned. *Avoid*: tick
**Epoch**: the segment of a session between resets; step indices are epoch-local. *Avoid*: run, attempt
**Event**: a physics-affecting input, stamped by the step it took effect. *Avoid*: input, action
**Viewer action**: camera or pause — affects seeing, never the event stream
**Session**: one execution of the binary, launch to exit
**Front-end**: viewer or scenario runner; two ways to drive one session
**Command**: interpreted intent for Control (speed, turn, gait-select); produced from events
**Parameter surface**: the tunable surface Control exposes; sliders bind to it
**Model fingerprint**: hash of the MJCF file; events are only valid against the model they were recorded with
**Gate**: a pass/fail claim about behavior (scenario + criteria + metrics)
**Ghost**: a recorded event stream replayed alongside live play

---

## Code alignment

Module names follow the language; each module serves exactly one domain.

| Domain | Module(s) |
|---|---|
| World | `World` — model/state ownership, step, reset |
| Session | `Clock` — fixed-step policy, pause, accumulator |
| Events | `InputEvent` + `Journal` — event type and append-only store |
| Presentation | `Viewer` + `Camera` — window, rendering, input glue; camera state |
| Control | gait engine (from story #11) |
| Verification | scenario runner (from story #10) |
| Evolution | CMA-ES search (Phase 5) |

---

## Recorded decision

**Physics-affecting inputs are journaled events; viewer actions are excluded.**

The journal is an append-only event store: replaying an event stream against the
same model fingerprint and seed yields a bit-identical world trajectory — which
is what makes Phase 4 ghosts and fair A/B comparisons possible. The rejected
alternative was journaling all input (including camera/pause), which would make
replays hostage to viewer noise, or polling key-state without a record, which
would make replays impossible. This decision shapes stories #9–#15 and all of
Phase 4; promote to `docs/adr/0001` when that directory is created.
