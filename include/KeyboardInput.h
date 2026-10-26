#pragma once
#include <termios.h>

// RAII wrapper putting stdin into raw, non-canonical, no-echo, non-blocking
// mode for the object's lifetime, restoring the original termios on
// destruction (so a crash mid-run doesn't leave the user's terminal
// broken). Used to drive live keyboard mode-switching (see ModeDispatcher)
// without blocking the ~500Hz control tick on a keypress.
//
// If stdin isn't a TTY (redirected/headless run), construction degrades
// gracefully: pollKey() always returns -1 rather than throwing:
class KeyboardInput {
public:
    KeyboardInput();
    ~KeyboardInput();

    KeyboardInput(const KeyboardInput&) = delete;
    KeyboardInput& operator=(const KeyboardInput&) = delete;

    // Returns the next available key (0-255), or -1 if none waiting. Never
    // blocks — safe to call every ~2ms tick.
    int pollKey();

private:
    bool is_tty_ = false;
    termios orig_termios_{};
};
