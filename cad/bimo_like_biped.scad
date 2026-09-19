// Parametric biped skeleton (a biped), servo-anchored massing model.
// Why: lock proportions + the 8x STS3215 joint layout before designing printable parts.
// Axes: X = forward, Y = left/right, Z = up. Servo bodies ARE the limb segments.

$fn = 40;

// --- STS3215 servo (measured: 45.2 x 24.6 x 35.1 mm) ---
sl = 45.2;   // long axis  -> runs along the limb
sw = 24.6;   // thin axis
sh = 35.1;   // tall/gearbox axis -> fore-aft depth
horn_d   = 13;
horn_off = 9.5;   // output disc offset from the top end

// --- structure ---
thigh = 26;  // bracket length hip-servo -> knee-servo
shin  = 26;  // bracket length knee-servo -> ankle-servo
foot_l = 84; foot_w = 48; foot_t = 6;

// --- torso + head (SBC cavity) ---
td = 46; tw = 104; th = 72;   // depth, width, height; width clears 2 hips side-by-side
hip_sep = 56;                 // leg center-to-center
hd = 46; hw = 62; hh = 42;    // head box

// --- pose (deg) — override at render time with -D to show range of motion ---
p_roll = 0; p_hip = 0; p_knee = 0; p_ankle = 0;
gait = 0;   // fore/aft hip split: +gait one leg forward, other back (walking demo)

col_servo  = [0.22,0.23,0.27];
col_struct = [0.80,0.82,0.85];
col_head   = [0.20,0.45,0.70];
col_horn   = [0.85,0.72,0.15];

// Servo with output axis along +Y and body long axis along Z (a vertical limb link).
module servo_pitch() {
  color(col_servo) cube([sh, sw, sl], center=true);
  color(col_horn)
    translate([0, sw/2, sl/2 - horn_off]) rotate([-90,0,0]) cylinder(d=horn_d, h=3);
}

// Simple structural bracket between two servos.
module link(len) {
  color(col_struct) translate([0,0,-len/2]) cube([sh*0.72, sw+5, len], center=true);
}

module foot() {
  color(col_struct) translate([foot_l*0.18, 0, -foot_t/2])
    cube([foot_l, foot_w, foot_t], center=true);
}

// One leg. side = +1 (left) / -1 (right). Forward-kinematic chain so joints bend.
module leg(side) {
  translate([0, side*hip_sep/2, -th/2]) {
    rotate([p_roll*side, 0, 0]) {                 // hip roll (about X)
      translate([0,0,-sh/2]) rotate([0,0,90]) servo_pitch();   // roll servo under torso
      translate([0,0,-sh])
        rotate([0, p_hip + side*gait, 0]) {       // hip pitch (about Y) + gait split
          translate([0,0,-sl/2]) servo_pitch();   // thigh = servo body
          translate([0,0,-sl]) {
            link(thigh);
            translate([0,0,-thigh])
              rotate([0, p_knee, 0]) {            // knee
                translate([0,0,-sl/2]) servo_pitch();  // shin = servo body
                translate([0,0,-sl]) {
                  link(shin);
                  translate([0,0,-shin])
                    rotate([0, p_ankle, 0]) {     // ankle
                      translate([0,0,-sh/2]) rotate([0,0,90]) servo_pitch();
                      translate([0,0,-sh]) foot();
                    }
                }
              }
          }
        }
    }
  }
}

module torso() {
  color(col_struct) cube([td, tw, th], center=true);
  // head with hollow SBC cavity
  translate([0,0, th/2 + hh/2])
    color(col_head) difference() {
      cube([hd, hw, hh], center=true);
      translate([0,0,-3]) cube([hd-8, hw-8, hh-6], center=true);
    }
}

module biped() {
  torso();
  leg(1);
  leg(-1);
}

biped();
