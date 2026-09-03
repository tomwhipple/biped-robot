#include "keymap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

namespace bimo {
namespace {

struct NameCode {
    const char* name;
    int code;
};

// Named (non-printable) keys only; letters and digits are their own ASCII.
const NameCode kNamed[] = {
    {"Space", kKeySpace},         {"Enter", kKeyEnter},
    {"Tab", kKeyTab},             {"Escape", kKeyEscape},
    {"Backspace", kKeyBackspace}, {"Up", kKeyUp},
    {"Down", kKeyDown},           {"Left", kKeyLeft},
    {"Right", kKeyRight},         {"PageUp", kKeyPageUp},
    {"PageDown", kKeyPageDown},   {"Home", kKeyHome},
    {"End", kKeyEnd},             {"Insert", kKeyInsert},
    {"Delete", kKeyDelete},       {"F1", kKeyF1},
    {"F2", kKeyF2},               {"F3", kKeyF3},
    {"F4", kKeyF4},               {"F5", kKeyF5},
    {"F6", kKeyF6},               {"F7", kKeyF7},
    {"F8", kKeyF8},               {"F9", kKeyF9},
    {"F10", kKeyF10},             {"F11", kKeyF11},
    {"F12", kKeyF12},
};

const struct {
    const char* name;
    const char* label;
} kActions[kActCount] = {
    {"forward", "forward"},
    {"back", "back"},
    {"strafe_left", "strafe left"},
    {"strafe_right", "strafe right"},
    {"turn_left", "turn left  (ccw)"},
    {"turn_right", "turn right (cw)"},
    {"arm", "arm / disarm"},
    {"stand", "stand"},
    {"estop", "e-stop"},
    {"quit", "quit (disarms)"},
    {"record", "record toggle"},
    {"reset_ext", "reset ext -> 14 B"},
    {"home", "reset servos -> stand"},
    {"control", "take control / watch only"},
};

void trim(char* s) {
    char* p = s;
    while (*p == ' ' || *p == '\t') ++p;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' ||
                     s[n - 1] == '\n')) {
        s[--n] = 0;
    }
}

}  // namespace

const char* actionName(int action) {
    if (action < 0 || action >= kActCount) return "?";
    return kActions[action].name;
}

const char* actionLabel(int action) {
    if (action < 0 || action >= kActCount) return "?";
    return kActions[action].label;
}

int actionFromName(const char* name) {
    for (int i = 0; i < kActCount; ++i) {
        if (!strcmp(name, kActions[i].name)) return i;
    }
    return -1;
}

const char* keyName(int key) {
    static char one[2] = {0, 0};
    if (key >= 'a' && key <= 'z') {
        one[0] = static_cast<char>(key);
        return one;
    }
    if (key >= '0' && key <= '9') {
        one[0] = static_cast<char>(key);
        return one;
    }
    for (size_t i = 0; i < sizeof kNamed / sizeof kNamed[0]; ++i) {
        if (kNamed[i].code == key) return kNamed[i].name;
    }
    return "";
}

int keyFromName(const char* name) {
    if (name == nullptr || name[0] == 0) return kKeyNone;
    if (name[1] == 0) {
        const char c = name[0];
        if (c >= 'a' && c <= 'z') return c;
        if (c >= 'A' && c <= 'Z') return c - 'A' + 'a';
        if (c >= '0' && c <= '9') return c;
        return kKeyNone;
    }
    for (size_t i = 0; i < sizeof kNamed / sizeof kNamed[0]; ++i) {
        if (!strcasecmp(name, kNamed[i].name)) return kNamed[i].code;
    }
    return kKeyNone;
}

void Keymap::reset() {
    // Arrows translate on the surface, PgUp/PgDn rotate: a window has real
    // key-release, so strafing is worth a pair of keys in a way it never was
    // on the terminal (link/tui.cpp keeps turn on left/right).
    key[kActForward] = kKeyUp;
    key[kActBack] = kKeyDown;
    key[kActStrafeLeft] = kKeyLeft;
    key[kActStrafeRight] = kKeyRight;
    key[kActTurnLeft] = kKeyPageUp;
    key[kActTurnRight] = kKeyPageDown;
    key[kActArm] = 'a';
    key[kActStand] = kKeySpace;
    key[kActEstop] = 'e';
    key[kActQuit] = 'q';
    key[kActRecord] = 'r';
    key[kActResetExt] = '0';
    key[kActHome] = 'h';
    // 't' for take. Deliberately not next to a motion key: this one hands the
    // robot between two consoles, and a fat finger on the way to an arrow
    // should not be able to steal it from whoever is flying it.
    key[kActControl] = 't';
}

int Keymap::actionFor(int k) const {
    if (k == kKeyNone) return -1;
    for (int i = 0; i < kActCount; ++i) {
        if (key[i] == k) return i;
    }
    return -1;
}

void Keymap::bind(int action, int k) {
    if (action < 0 || action >= kActCount) return;
    if (k != kKeyNone) {
        for (int i = 0; i < kActCount; ++i) {
            if (i != action && key[i] == k) key[i] = kKeyNone;
        }
    }
    key[action] = k;
}

bool Keymap::load(const char* path, char* err, size_t cap) {
    FILE* f = fopen(path, "r");
    if (f == nullptr) return true;     // no file: the defaults stand
    char line[256];
    int lineno = 0;
    while (fgets(line, sizeof line, f) != nullptr) {
        ++lineno;
        char* hash = strchr(line, '#');
        if (hash != nullptr) *hash = 0;
        trim(line);
        if (line[0] == 0) continue;
        char* eq = strchr(line, '=');
        if (eq == nullptr) {
            snprintf(err, cap, "%s:%d: expected `action = Key`", path, lineno);
            fclose(f);
            return false;
        }
        *eq = 0;
        char* rhs = eq + 1;
        trim(line);
        trim(rhs);
        const int action = actionFromName(line);
        if (action < 0) {
            snprintf(err, cap, "%s:%d: unknown action `%s`", path, lineno, line);
            fclose(f);
            return false;
        }
        const int k = keyFromName(rhs);
        if (k == kKeyNone && strcmp(rhs, "none") != 0) {
            snprintf(err, cap, "%s:%d: unknown key `%s`", path, lineno, rhs);
            fclose(f);
            return false;
        }
        bind(action, k);
    }
    fclose(f);
    return true;
}

bool Keymap::save(const char* path, char* err, size_t cap) const {
    // mkdir the parent; ENOENT on a fresh machine is not a reason to lose a
    // rebind the operator just made.
    const char* slash = strrchr(path, '/');
    if (slash != nullptr) {
        char dir[512];
        const size_t n = static_cast<size_t>(slash - path);
        if (n < sizeof dir) {
            memcpy(dir, path, n);
            dir[n] = 0;
            mkdir(dir, 0755);
        }
    }
    FILE* f = fopen(path, "w");
    if (f == nullptr) {
        snprintf(err, cap, "cannot write %s", path);
        return false;
    }
    fprintf(f, "# bimo_gui key bindings. One `action = Key` per line.\n");
    fprintf(f, "# Keys: a-z, 0-9, Space Enter Tab Escape Backspace Up Down\n");
    fprintf(f, "#       Left Right PageUp PageDown Home End Insert Delete F1-F12\n");
    for (int i = 0; i < kActCount; ++i) {
        const char* kn = keyName(key[i]);
        fprintf(f, "%-13s = %s\n", actionName(i), kn[0] ? kn : "none");
    }
    fclose(f);
    return true;
}

bool defaultKeymapPath(char* out, size_t cap) {
    const char* home = getenv("HOME");
    if (home == nullptr || home[0] == 0) return false;
    snprintf(out, cap, "%s/.bimo/keymap.conf", home);
    return true;
}

}  // namespace bimo
