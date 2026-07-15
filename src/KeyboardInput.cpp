#include "KeyboardInput.h"
#include <unistd.h>

KeyboardInput::KeyboardInput() {
    is_tty_ = ::isatty(STDIN_FILENO);
    if (!is_tty_) return;

    tcgetattr(STDIN_FILENO, &orig_termios_);

    termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;  // non-blocking: read() returns immediately
    raw.c_cc[VTIME] = 0;  // with 0 bytes if none are available
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

KeyboardInput::~KeyboardInput() {
    if (is_tty_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
}

int KeyboardInput::pollKey() {
    if (!is_tty_) return -1;

    unsigned char c;
    ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return (n == 1) ? static_cast<int>(c) : -1;
}
