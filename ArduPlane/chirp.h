// 08/22/19
// Justin Matt
// This header file includes definitions necessary for the chirp developed in mode_fbwa.cpp

#ifndef CHIRP_H
#define CHIRP_H

// Support for chirp code
extern uint64_t t_in_fbwa;
extern uint64_t t0_fbwa;
extern uint64_t t_last;
extern uint64_t t_current;
extern float theta_sine;
extern float time_step_;
extern float wmin;
extern float wmax;
extern float Trec;
extern float K_sine;
extern float omega_rps;
extern float d_aileron;
extern uint64_t chirp;

#endif
