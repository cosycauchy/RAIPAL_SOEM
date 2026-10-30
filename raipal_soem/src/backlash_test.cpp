// src/backlash_test.cpp
//
// 출력단을 기계적으로 잠근 상태에서 모터(입력)에 작은 토크를 0 -> +A -> 0 -> -A -> 0 으로
// 천천히 걸었다 풀기를 반복하고, 모터 엔코더 위치를 CSV로 기록한다.
// 출력이 잠겨 있으므로 모터가 움직인 양 = 백래시 + 기어 비틀림 변형이다.
// 백래시 값은 analyze_backlash.py 로 CSV에서 계산한다.
//
// 사용법:  sudo ./backlash_test <ifname> <backlash.yaml> <out.csv>
//
// 기반: main.cpp (demo) 의 EtherCAT / CiA-402 처리 방식을 그대로 따른다.

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

// ---------------------------------------------------------------- Ctrl+C 처리
static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

// ---------------------------------------------------------------- 설정
struct BacklashCfg {
    double  gear_ratio          = 28.35;
    int     counts_per_rev      = 0;     // 모터 엔코더 1회전당 카운트 (0이면 각도 환산 생략)
    int     torque_amplitude    = 50;    // 1000 = 정격 토크. 50 = 정격의 5%
    double  ramp_time_s         = 2.0;   // 0 -> A 까지 걸리는 시간
    int     repeats             = 5;     // 삼각파 반복 횟수
    double  settle_s            = 1.0;   // 시작 전 토크 0 유지 시간
    int     max_travel_counts   = 2000;  // 시작 위치에서 이만큼 벗어나면 즉시 정지 (안전)
    int     temp_limit_c        = 60;    // 이 온도 이상이면 즉시 정지 (선배 기준 70도 절대 금지)
    int     torque_hard_limit   = 200;   // 어떤 설정이든 이 값 넘게는 안 보냄 (정격 20%)
};

static BacklashCfg load_cfg(const char* path) {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node n = root["backlash_test"];
    if (!n) {
        std::fprintf(stderr, "Config error: 'backlash_test' 항목이 없음\n");
        std::exit(2);
    }
    BacklashCfg c;
    if (n["gear_ratio"])        c.gear_ratio        = n["gear_ratio"].as<double>();
    if (n["counts_per_rev"])    c.counts_per_rev    = n["counts_per_rev"].as<int>();
    if (n["torque_amplitude"])  c.torque_amplitude  = n["torque_amplitude"].as<int>();
    if (n["ramp_time_s"])       c.ramp_time_s       = n["ramp_time_s"].as<double>();
    if (n["repeats"])           c.repeats           = n["repeats"].as<int>();
    if (n["settle_s"])          c.settle_s          = n["settle_s"].as<double>();
    if (n["max_travel_counts"]) c.max_travel_counts = n["max_travel_counts"].as<int>();
    if (n["temp_limit_c"])      c.temp_limit_c      = n["temp_limit_c"].as<int>();
    if (n["torque_hard_limit"]) c.torque_hard_limit = n["torque_hard_limit"].as<int>();

    if (std::abs(c.torque_amplitude) > c.torque_hard_limit) {
        std::fprintf(stderr, "Config error: torque_amplitude(%d) > torque_hard_limit(%d)\n",
                     c.torque_amplitude, c.torque_hard_limit);
        std::exit(2);
    }
    if (c.ramp_time_s <= 0.0 || c.repeats <= 0) {
        std::fprintf(stderr, "Config error: ramp_time_s, repeats 는 양수여야 함\n");
        std::exit(2);
    }
    return c;
}

// 삼각파 토크 지령: 한 주기 = 0 -> +A -> 0 -> -A -> 0 (4구간, 각 ramp_time)
// t: 측정 시작 후 경과 시간. 끝나면 done=true
static int16_t torque_profile(double t, const BacklashCfg& c, int& rep, bool& done) {
    const double T = c.ramp_time_s;
    const double period = 4.0 * T;
    rep = static_cast<int>(t / period);
    if (rep >= c.repeats) { done = true; return 0; }
    done = false;
    const double tp = t - rep * period;
    double frac;  // -1 ~ +1
    if      (tp < T)       frac =  tp / T;               // 0 -> +1
    else if (tp < 2 * T)   frac =  1.0 - (tp - T) / T;   // +1 -> 0
    else if (tp < 3 * T)   frac = -(tp - 2 * T) / T;     // 0 -> -1
    else                   frac = -1.0 + (tp - 3 * T) / T; // -1 -> 0
    return static_cast<int16_t>(std::lround(frac * c.torque_amplitude));
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::puts("Usage: backlash_test <ifname> <backlash.yaml> <out.csv>\n"
                  "  e.g., sudo ./backlash_test enp3s0 ../src/backlash.yaml backlash_0cycle.csv");
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    const char* ifname   = argv[1];
    const BacklashCfg cfg = load_cfg(argv[2]);
    const char* csv_path = argv[3];

    FILE* csv = std::fopen(csv_path, "w");
    if (!csv) { std::perror("CSV 파일 열기 실패"); return 1; }
    // 분석 스크립트가 읽는 메타데이터 (# 으로 시작)
    std::fprintf(csv, "# gear_ratio=%.4f\n# counts_per_rev=%d\n# torque_amplitude=%d\n"
                      "# ramp_time_s=%.3f\n# repeats=%d\n",
                 cfg.gear_ratio, cfg.counts_per_rev, cfg.torque_amplitude,
                 cfg.ramp_time_s, cfg.repeats);
    std::fprintf(csv, "t_s,rep,cmd_tor,tor,pos,vel,temp,status,error\n");

    int exit_code = 0;
    try {
        EthercatMaster master(ifname);
        if (master.slaveCount() != 1) {
            std::fprintf(stderr, "이 테스트는 구동기 1개만 연결한 상태를 가정함 (발견: %d개)\n",
                         master.slaveCount());
            std::fclose(csv);
            return 2;
        }

        ActuatorPDOMap map{};
        EthercatActuator act(1, map);
        ActuatorCommand  cmd{};
        ActuatorFeedback fb{};

        cmd.controlword = CW_FAULT_RESET;
        cmd.mode        = MODE_CYCLIC_SYNCHRONOUS_TORQUE;  // 0x0A, 토크 모드
        cmd.target_tor  = 0;

        const auto period  = std::chrono::microseconds(5000);  // 5 ms, demo와 동일
        const double dt    = 0.005;

        enum Phase { BRINGUP, SETTLE, MEASURE, STOPPING } phase = BRINGUP;
        int     ticks_in_phase = 0;
        int     bringup_ticks  = 0;
        int     wkc_fail       = 0;
        int     stop_ticks     = 0;
        int32_t pos0           = 0;
        double  t_meas         = 0.0;
        std::string stop_reason = "완료";

        while (true) {
            // ------------------------------------------------ 지령 결정
            int  rep  = -1;
            bool done = false;
            int16_t tor_cmd = 0;
            if (phase == MEASURE) {
                tor_cmd = torque_profile(t_meas, cfg, rep, done);
                if (done) { phase = STOPPING; stop_ticks = 0; tor_cmd = 0; }
            }
            // 안전: 어떤 경우에도 하드 리밋을 넘지 않게
            tor_cmd = static_cast<int16_t>(std::clamp<int>(tor_cmd,
                                     -cfg.torque_hard_limit, cfg.torque_hard_limit));
            cmd.target_tor = (phase == MEASURE) ? tor_cmd : 0;

            act.writeCommand(cmd);
            int wkc = master.tickOnce();

            if (wkc >= master.expectedWKC()) {
                wkc_fail = 0;
                act.readFeedback(fb);
                act.advanceCiA402(fb, cmd);
                cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;

                // ---------------------------------------- 상태 전이
                if (phase == BRINGUP) {
                    if (fb.status == 0x27) {          // Operation Enabled
                        cmd.controlword = CW_ENABLE_OPERATION;
                        phase = SETTLE; ticks_in_phase = 0;
                        std::puts("\nOperation Enabled -> 토크 0으로 안정화 중");
                    } else if (++bringup_ticks > 2000) { // 10 s 안에 못 켜지면 포기
                        stop_reason = "구동기가 Operation Enabled 상태가 되지 않음";
                        exit_code = 3; break;
                    }
                } else if (phase == SETTLE) {
                    if (++ticks_in_phase * dt >= cfg.settle_s) {
                        pos0 = fb.pos;
                        phase = MEASURE; t_meas = 0.0;
                        std::printf("\n측정 시작 (pos0=%d)\n", pos0);
                    }
                }

                // ---------------------------------------- 안전 검사
                if (phase == MEASURE || phase == SETTLE) {
                    if (g_stop)                                   { stop_reason = "사용자 중단(Ctrl+C)"; exit_code = 4; }
                    else if (fb.status == 0x08)                   { stop_reason = "구동기 Fault"; exit_code = 5; }
                    else if (fb.temp >= cfg.temp_limit_c)         { stop_reason = "온도 제한 초과"; exit_code = 6; }
                    else if (phase == MEASURE &&
                             std::abs(fb.pos - pos0) > cfg.max_travel_counts) {
                        stop_reason = "이동량 제한 초과 - 출력이 제대로 잠겨 있는지 확인";
                        exit_code = 7;
                    }
                    if (exit_code != 0) { phase = STOPPING; stop_ticks = 0; }
                }

                // ---------------------------------------- 기록
                if (phase == MEASURE) {
                    std::fprintf(csv, "%.4f,%d,%d,%d,%d,%d,%d,%u,%u\n",
                                 t_meas, rep, tor_cmd, fb.tor, fb.pos, fb.vel,
                                 fb.temp, fb.status, fb.error);
                    t_meas += dt;
                }
                std::printf("phase=%d sw=0x%02X cmd=%4d tor=%4d dpos=%6d temp=%3d   \r",
                            phase, fb.status, cmd.target_tor, fb.tor,
                            fb.pos - pos0, fb.temp);
                std::fflush(stdout);
            } else if (++wkc_fail > 20) {               // 통신 100 ms 이상 끊김
                stop_reason = "EtherCAT 통신 끊김";
                exit_code = 8; break;
            }

            if (phase == BRINGUP && g_stop) { stop_reason = "사용자 중단(Ctrl+C)"; exit_code = 4; break; }

            // ---------------------------------------- 종료: 토크 0 유지 후 끄기
            if (phase == STOPPING) {
                cmd.target_tor = 0;
                if (++stop_ticks > 100) {               // 0.5 s 토크 0 유지
                    cmd.controlword = CW_SHUTDOWN;
                    act.writeCommand(cmd);
                    master.tickOnce();
                    break;
                }
            }
            std::this_thread::sleep_for(period);
        }

        std::printf("\n종료: %s\n", stop_reason.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        exit_code = 1;
    }

    std::fclose(csv);
    std::printf("CSV 저장: %s\n", csv_path);
    return exit_code;
}
