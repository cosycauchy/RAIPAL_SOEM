// sim/sim_hw.hpp — 실제 구동기 없이 endurance_test 의 로직을 시험하기 위한 가짜 구동기 (PC에서만 사용)
//
// 모터(회전자) + 기어 틈(백래시) + 10 kg 원판을 간단한 물리로 흉내 낸다. 모든 각도는 "출력축 기준".
// 실제 하드웨어의 응답을 보장하지 않는다. 사이클/측정/온도/이어하기 같은 "프로그램 흐름"을 확인하는 용도.
//
// 환경변수로 조절:  SIM_B0_DEG(초기 백래시, 0.10)  SIM_WEAR(출력 1바퀴당 백래시 증가[deg], 0)
//                   SIM_J(원판 관성 kg m^2, 0.12)  SIM_TEMP_GAIN(30)  SIM_TEMP_TAU(120)  SIM_TRACE(파일)
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <algorithm>

// ---- 실제 ethercat_actuator.hpp 와 같은 정의 ----
enum ModeOfOperation : uint8_t {
  MODE_NO_MODE = 0x00, MODE_PROFILE_POSITION = 0x01, MODE_PROFILE_VELOCITY = 0x03,
  MODE_PROFILE_TORQUE = 0x04, MODE_HOMING = 0x06, MODE_INTERPOLATION_POSITION = 0x07,
  MODE_CYCLIC_SYNCHRONOUS_POSITION = 0x08, MODE_CYCLIC_SYNCHRONOUS_VELOCITY = 0x09,
  MODE_CYCLIC_SYNCHRONOUS_TORQUE = 0x0A, MODE_PROFILE_COMMUTATION = 0x0F };
enum ControlWord : uint8_t {
  CW_ZERO = 0x00, CW_SHUTDOWN = 0x06, CW_SWITCH_ON = 0x07, CW_ENABLE_OPERATION = 0x0F,
  CW_DISABLE_OPERATION = 0x07, CW_FAULT_RESET = 0x80 };
struct ActuatorFeedback { int32_t pos{0}; int32_t vel{0}; int16_t tor{0}; uint8_t status{0}; uint8_t temp{0}; uint8_t error{0}; };
struct ActuatorCommand { uint8_t controlword{CW_FAULT_RESET}; uint8_t mode{0}; int32_t target_pos{0};
                         int32_t target_vel{0}; int16_t target_tor{0}; uint16_t kp{0}; uint16_t kd{0}; };

static double env_d(const char* k, double d) { const char* v = std::getenv(k); return v ? std::atof(v) : d; }

struct SimHw {
    // 구동기 사양 (출력축 기준)
    static constexpr double N = 28.35, CPR = 65536, PI = 3.14159265358979;
    static constexpr double NM_PER_UNIT = 0.53 * N / 1000.0;     // 토크 1 단위 = 출력 0.015 Nm
    double Jm = 0.024;                                           // 회전자 관성 (출력축 환산)
    double J  = env_d("SIM_J", 0.12);                            // 원판 관성
    double e  = 0.2;                                             // 충돌 반발 계수

    // 상태
    double th_m = 12345.0 * 2 * PI / (CPR * N), th_l = th_m;     // 시작 위치를 0이 아니게 (실제 pos 도 0이 아님)
    double w_m = 0, w_l = 0;
    double b_deg = env_d("SIM_B0_DEG", 0.10), wear = env_d("SIM_WEAR", 0.0), travel_rev = 0;
    double temp = 25, temp_gain = env_d("SIM_TEMP_GAIN", 30), temp_tau = env_d("SIM_TEMP_TAU", 120);
    double u2_lp = 0;
    ActuatorCommand cmd{}; ActuatorFeedback fb{};
    FILE* trace = nullptr; double t = 0, last_trace = 0;
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
    int fault_ticks = 0;

    SimHw() { fb.status = 0x40; fb.pos = pos(); if (const char* p = std::getenv("SIM_TRACE")) trace = std::fopen(p, "w"); }
    ~SimHw() { if (trace) std::fclose(trace); }
    int32_t pos() const { return (int32_t)std::llround(th_m * N * CPR / (2 * PI)); }

    double torque_cmd() const {                                  // 출력축 기준 모터 토크 [Nm]
        if (fb.status != 0x27) return 0;
        if (cmd.mode == MODE_CYCLIC_SYNCHRONOUS_TORQUE)
            return std::clamp<double>(cmd.target_tor, -4000, 4000) * NM_PER_UNIT;
        if (cmd.mode == MODE_CYCLIC_SYNCHRONOUS_POSITION) {
            double tgt = cmd.target_pos * 2 * PI / (CPR * N);
            return std::clamp(200.0 * (tgt - th_m) - 11.0 * w_m, -20.0, 20.0);
        }
        return 0;
    }

    void substep(double h) {
        double tau = torque_cmd();
        double Tm = tau - 0.01 * w_m - 0.1 * std::tanh(w_m / 0.02);
        double Tl =     - 0.002 * w_l - 0.05 * std::tanh(w_l / 0.02);
        double hg = 0.5 * b_deg * PI / 180.0, rel = th_m - th_l;
        double am = Tm / Jm, al = Tl / J;
        bool locked = (rel >= hg - 1e-9 && am > al) || (rel <= -hg + 1e-9 && am < al);
        if (locked) {
            double a = (Tm + Tl) / (Jm + J);
            w_m += a * h; w_l = w_m;
            th_m += w_m * h; th_l = th_m - (rel >= 0 ? hg : -hg);
        } else {
            w_m += am * h; w_l += al * h; th_m += w_m * h; th_l += w_l * h;
            rel = th_m - th_l;
            if (std::abs(rel) > hg) {
                double s = rel > 0 ? 1 : -1, vrel = w_m - w_l;
                th_l = th_m - s * hg;
                if (vrel * s > 0) {
                    double ee = std::abs(vrel) < 1e-3 ? 0 : e, p = Jm * w_m + J * w_l;
                    w_m = (p - J * ee * vrel) / (Jm + J); w_l = w_m + ee * vrel;
                }
            }
        }
        travel_rev += std::abs(w_l) * h / (2 * PI);
        b_deg = env_d("SIM_B0_DEG", 0.10) + wear * travel_rev;
        double u = tau / (0.53 * N);                             // 정격 토크 대비 비율
        u2_lp += (u * u - u2_lp) * std::min(1.0, h / 0.05);
        temp += h * ((25 + temp_gain * u2_lp) - temp) / temp_tau;
    }

    void step() {
        auto now = std::chrono::steady_clock::now();
        double dt = std::min(0.02, std::chrono::duration<double>(now - last).count());
        last = now;
        for (double s = 0; s < dt; s += 50e-6) substep(std::min(50e-6, dt - s));
        t += dt;
        // CiA-402 상태 머신 (제어워드에 따라 한 단계씩)
        uint8_t cw = cmd.controlword;
        if (cw == CW_FAULT_RESET && fb.status != 0x40 && fb.status != 0x21 && fb.status != 0x23 && fb.status != 0x27) fb.status = 0x40;
        else if (fb.status == 0x40 && cw == CW_SHUTDOWN) fb.status = 0x21;
        else if (fb.status == 0x21 && cw == CW_SWITCH_ON) fb.status = 0x23;
        else if (fb.status == 0x23 && cw == CW_ENABLE_OPERATION) fb.status = 0x27;
        else if (fb.status == 0x27 && cw == CW_SHUTDOWN) fb.status = 0x21;
        fb.pos = pos(); fb.tor = cmd.target_tor; fb.temp = (uint8_t)std::clamp(temp, 0.0, 255.0);
        if (trace && t - last_trace >= 0.005) {
            last_trace = t;
            std::fprintf(trace, "%.4f,%d,%.1f,%d,%d,%.4f,%.2f\n", t, fb.pos, w_m * N * 60 / (2 * PI) / N, cmd.target_tor,
                         cmd.mode, b_deg, temp);
        }
    }
};
inline SimHw& hw() { static SimHw h; return h; }
