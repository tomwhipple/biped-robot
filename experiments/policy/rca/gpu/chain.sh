#!/usr/bin/env bash
# Hand the GPU window over after teacher A's distill: stop the first wrapper the moment it
# would start its second teacher, then run the actuator-randomized teachers on this branch.
LOG=$HOME/code/robot-rca/experiments/policy/rca/gpu/rca_gpu.log
until grep -q "teacher robot_legs_walk14_s1_ggad start" "$LOG"; do sleep 20; done
pkill -9 -f "rca_gpu.sh robot_legs_walk14_s1_gg" ; sleep 1
pkill -f -- "--out robot_legs_walk14_s1_ggad" ; sleep 10
echo "$(date +%H:%M) chain: stopped teacher B before it trained; handing over to rca_gpu2" >> "$LOG"
"$HOME/code/robot-mjx/night/vision_pause.sh" resume >/dev/null 2>&1
# the actuator flags live on this branch: wait until the push has landed
until git -C "$HOME/code/robot-rca" -c credential.helper= -c 'credential.helper=!gh auth git-credential' \
      ls-remote https://github.com/tomwhipple/biped-robot.git policy-stand-rca | grep -q .; do sleep 30; done
BR=policy-stand-rca exec "$HOME/code/robot-rca/experiments/policy/rca/gpu/rca_gpu2.sh" \
  "robot_legs_walk14_s1_act|--steps 400000000 --num-evals 12 --act-delay-types sts3215=2:5,sts3250=0:1 --act-lag 2,8 --servo-kd-range 0.12,1.0 --servo-kp-range 0.6,1.2 --joint-frictionloss-range 0.1,0.4 --gyro-gain-range 0.7,1.6 --gyro-delay-max 1" \
  "robot_legs_walk14_s1_actar|--steps 300000000 --num-evals 10 --act-delay-types sts3215=2:5,sts3250=0:1 --act-lag 2,8 --servo-kd-range 0.12,1.0 --servo-kp-range 0.6,1.2 --joint-frictionloss-range 0.1,0.4 --gyro-gain-range 0.7,1.6 --gyro-delay-max 1 --w-action-rate 0.3"
