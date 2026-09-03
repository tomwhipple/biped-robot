// Rebindable keys for the windowed console.
//
// The key codes here are this file's own, not the window toolkit's: the
// keymap is parsed, emitted and tested by `make test`, which must keep
// working on "a laptop with a C++ compiler" (firmware/host/Makefile's whole
// premise) with no GLFW and no ImGui installed. link/gui.cpp owns the one
// small translation from the toolkit's key enum to these.
//
// The file format is the house style -- one `action = Key` line, `#`
// comments, unknown lines refused loudly rather than dropped, because a
// silently ignored binding is a key that does nothing on a robot that is
// already moving.
#ifndef BIMO_LINK_KEYMAP_H_
#define BIMO_LINK_KEYMAP_H_

#include <stddef.h>

namespace bimo {

// Everything a key can be bound to. Motion actions map onto client.h's Axis
// values by construction (kActForward .. kActTurnRight, same order).
enum Action {
    kActForward = 0,
    kActBack,
    kActStrafeLeft,
    kActStrafeRight,
    kActTurnLeft,
    kActTurnRight,
    kActArm,
    kActStand,
    kActEstop,
    kActQuit,
    kActRecord,
    kActResetExt,
    kActHome,
    kActControl,
    kActCount,
};

// The first six actions ARE the six axes, in order (client.h Axis).
constexpr int kMotionActions = 6;

// Key codes: printable ASCII for letters and digits, so 'a' is 'a'; named
// keys start above ASCII. Values are ours alone -- see the file comment.
enum KeyCode {
    kKeyNone = 0,
    kKeyNamedFirst = 256,
    kKeySpace = kKeyNamedFirst,
    kKeyEnter,
    kKeyTab,
    kKeyEscape,
    kKeyBackspace,
    kKeyUp,
    kKeyDown,
    kKeyLeft,
    kKeyRight,
    kKeyPageUp,
    kKeyPageDown,
    kKeyHome,
    kKeyEnd,
    kKeyInsert,
    kKeyDelete,
    kKeyF1,
    kKeyF2,
    kKeyF3,
    kKeyF4,
    kKeyF5,
    kKeyF6,
    kKeyF7,
    kKeyF8,
    kKeyF9,
    kKeyF10,
    kKeyF11,
    kKeyF12,
    kKeyNamedLast = kKeyF12,
};

const char* actionName(int action);      // "forward", "arm", ...
const char* actionLabel(int action);     // "forward", "arm / disarm", ...
int actionFromName(const char* name);    // -1 if unknown

const char* keyName(int key);            // "a", "Space", "PageUp"; "" if none
int keyFromName(const char* name);       // kKeyNone if unknown

struct Keymap {
    int key[kActCount];

    Keymap() { reset(); }
    void reset();                        // the built-in defaults

    // -1 if nothing is bound to `key`.
    int actionFor(int key) const;
    // Binds, clearing any other action holding the same key -- one key can
    // only mean one thing, and a duplicate that silently shadows is worse
    // than one that visibly steals.
    void bind(int action, int key);

    // Returns false and fills `err` (cap bytes) on a bad line. A missing file
    // is NOT an error: it means "use the defaults".
    bool load(const char* path, char* err, size_t cap);
    bool save(const char* path, char* err, size_t cap) const;
};

// ~/.bimo/keymap.conf, expanded. Returns false if $HOME is unset.
bool defaultKeymapPath(char* out, size_t cap);

}  // namespace bimo

#endif  // BIMO_LINK_KEYMAP_H_
