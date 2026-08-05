#pragma once

// Base contract for a live-switchable control mode, modeled on
// Cheetah-Software's FSM_State<T> (onEnter/run/checkTransition/transition/
// onExit) but compressed to what DASH actually needs today: no mode here
// requires a multi-tick ramped handoff into another, and mode selection is
// a direct keypress (see KeyboardInput/ModeDispatcher), not an RC value
// each state has to interpret via switch. So onExit() folds Cheetah's
// onExit()+transition() together — it may itself block briefly (e.g. a 2s
// smooth-return-to-zero), same as this codebase's existing shutdown
// sequences already do, rather than being polled across multiple ticks by
// the dispatcher. If a real ramped transition is ever needed, this is the
// method that would be split back apart — without touching ModeDispatcher.
class Mode {
public:
    virtual ~Mode() = default;

    virtual const char* name() const = 0;

    // Called once when this mode becomes current (fresh switch or initial
    // startup). Establish hold/home targets, arm gains, etc.
    virtual void onEnter() {}

    // Called every control tick while this mode is current.
    virtual void run() = 0;

    // Called once when this mode is being left — either a live switch to
    // another mode, or process shutdown (Ctrl+C). Exactly one call site
    // covers both cases; wrap up safely (e.g. smooth-return-to-zero) here.
    virtual void onExit() {}

    // Safety-gating seam, deliberately unused this phase: needs an
    // orientation estimate this codebase doesn't have yet. Always true for
    // now — mirrors Cheetah's checkSafeOrientation/checkPDesFoot, which a
    // later phase can wire up without restructuring ModeDispatcher.
    virtual bool canExit() const { return true; }
};
