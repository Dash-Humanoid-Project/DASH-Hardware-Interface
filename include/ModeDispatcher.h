#pragma once
#include <map>
#include "Mode.h"
#include "KeyboardInput.h"

// Owns the currently-active Mode and switches it on a mapped keypress.
// Mirrors Cheetah's ControlFSM::runFSM() dispatch loop, simplified per
// Mode.h's comments: all Mode instances are constructed once by the driver
// and persist for the process lifetime (matches Cheetah's FSM_StatesList —
// no per-transition allocation); a switch just swaps which Mode* is
// current. Holds no limb references itself — only Mode* — so it doesn't
// need to change as the number of limbs/modes grows; each concrete Mode
// gets whatever Leg&/LegController& it needs via its own constructor.
class ModeDispatcher {
public:
    ModeDispatcher(std::map<char, Mode*> key_to_mode, Mode* initial_mode);

    // One call per control tick: non-blocking keyboard poll, live-switch if
    // a mapped key differs from the current mode and the current mode
    // allows it (canExit()), then always run the (possibly just-switched)
    // current mode.
    void tick();

    Mode& current() { return *current_mode_; }

private:
    KeyboardInput keyboard_;
    std::map<char, Mode*> key_to_mode_;
    Mode* current_mode_;
};
