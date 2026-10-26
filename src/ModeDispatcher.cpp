#include "ModeDispatcher.h"

ModeDispatcher::ModeDispatcher(std::map<char, Mode*> key_to_mode, Mode* initial_mode)
    : key_to_mode_(std::move(key_to_mode)), current_mode_(initial_mode)
{
    // Mirrors Cheetah's ControlFSM::initialize(): the starting state gets
    // onEnter() called on it immediately, not just on later switches.
    current_mode_->onEnter();
}

void ModeDispatcher::tick() {
    int c = keyboard_.pollKey();
    if (c >= 0) {
        auto it = key_to_mode_.find(static_cast<char>(c));
        if (it != key_to_mode_.end() && it->second != current_mode_ && current_mode_->canExit()) {
            current_mode_->onExit();
            current_mode_ = it->second;
            current_mode_->onEnter();
        }
    }
    current_mode_->run();
}
