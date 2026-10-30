// src/backlash_test.cpp
//
// 10 kg 원판을 출력에 단 상태에서 백래시를 잰다. (선배 actuator_test 의 measure_backlash 방식)
//
// 원리:
//   모터에 작은 토크를 + / - 로 빠르게(수십 ms마다) 뒤집어 건다.
//   원판은 무거워서 그 짧은 시간 동안 거의 움직이지 않으므로, 모터만 기어 사이 틈(백래시)만큼 왔다 갔다 한다.
//   모터 엔코더(pos)가 왔다 갔다 한 폭 = 백래시 (모터 쪽 카운트).
//
// 순서 (측정 위치마다 반복):
//   1) 위치 모드(CSP)로 원판을 다음 측정 위치까지 "천천히" 돌린다 (목표를 조금씩 옮김)
//   2) 토크 모드(CST)로 바꾸고 ±토크 사각파를 measure_time_s 동안 건다
//   3) 매 루프 pos 를 CSV 에 기록
// 분석은 analyze_backlash.py 로 한다.
//
// 사용법:  sudo ./backlash_test <ifname> <backlash.yaml> <out.csv>

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

using clk = std::chrono::steady_clock;

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

// ---------------------------------------------------------------- 설정
struct Cfg {
    double gear_ratio        = 28.35;
    int    counts_per_rev    = 65536;   // 모터 엔코더 1회전 카운트 (선배 코드로 추정, 확인 필요)
    int    num_positions     = 4;       // 출력 한 바퀴를 몇 곳으로 나눠 잴지
    double move_speed_cps    = 50000;   // 위치 이동 속도 [카운트/초]. 50000 = 출력 약 10도/초
    int    position_tolerance= 500;     // 목표 위치 도착 판정 [카운트]
    double settle_s          = 1.0;     // 도착 후 기다리는 시간
    int    torque_mag        = 20;      // 진동 토크 크기. 1000 = 정격 → 20 = 2%
    int    flip_ms           = 15;      // 몇 ms마다 +/- 를 뒤집을지
    double measure_time_s    = 2.0;     // 한 위치에서 진동을 주는 시간
    int    period_us         = 500;     // 루프 주기 (선배 코드와 같은 0.5 ms)
    uint16_t kp = 0, kd = 0;            // 위치 모드 게인 (선배 코드는 0으로 사용)
    // ---- 안전 ----
    int    max_vibration_travel = 3000; // 진동 중 원판이 이만큼 넘게 밀리면 정지 [카운트]
    int    max_follow_error     = 20000;// 이동 중 목표와 실제 차이가 이만큼 넘으면 정지 [카운트]
    int    temp_limit_c         = 60;
    int    torque_hard_limit    = 100;  // 코드가 절대 넘기지 않는 진동 토크 (정격 10%)
};

static Cfg load_cfg(const char* path) {
    YAML::Node n = YAML::LoadFile(path)["backlash_test"];
    if (!n) { std::fprintf(stderr, "Config error: 'backlash_test' 항목이 없음\n"); std::exit(2); }
    Cfg c;
#define RD(key, type) if (n[#key]) c.key = n[#key].as<type>()
    RD(gear_ratio, double); RD(counts_per_rev, int); RD(num_positions, int);
    RD(move_speed_cps, double); RD(position_tolerance, int); RD(settle_s, double);
    RD(torque_mag, int); RD(flip_ms, int); RD(measure_time_s, double); RD(period_us, int);
    RD(kp, uint16_t); RD(kd, uint16_t);
    RD(max_vibration_travel, int); RD(max_follow_error, int);
    RD(temp_limit_c, int); RD(torque_hard_limit, int);
#undef RD
    if (std::abs(c.torque_mag) > c.torque_hard_limit) {
        std::fprintf(stderr, "Config error: torque_mag(%d) > torque_hard_limit(%d)\n",
                     c.torque_mag, c.torque_hard_limit);
        std::exit(2);
    }
    if (c.num_positions < 1 || c.flip_ms < 1 || c.period_us < 100 || c.move_speed_cps <= 0) {
        std::fprintf(stderr, "Config error: 값 범위 확인 (num_positions, flip_ms, period_us, move_speed_cps)\n");
        std::exit(2);
    }
    return c;
}

// ---------------------------------------------------------------- 공통 루프 도우미
struct Rig {
    EthercatMaster&   master;
    EthercatActuator& act;
    ActuatorCommand   cmd{};
    ActuatorFeedback  fb{};
    const Cfg&        cfg;
    int               wkc_fail = 0;
    std::string       err;            // 비어 있지 않으면 중단

    // 한 번 주고받기. 통신/고장/온도/중단 검사까지. false = 중단해야 함
    bool tick() {
        act.writeCommand(cmd);
        int wkc = master.tickOnce();
        if (wkc >= master.expectedWKC()) {
            wkc_fail = 0;
            uint8_t want_mode = cmd.mode;
            act.readFeedback(fb);
            act.advanceCiA402(fb, cmd);
            cmd.mode = want_mode;
            if (fb.status == 0x27) cmd.controlword = CW_ENABLE_OPERATION;
            if (fb.status == 0x08)           err = "구동기 Fault";
            if (fb.temp >= cfg.temp_limit_c) err = "온도 제한 초과";
        } else if (++wkc_fail > 200) {      // 약 0.1 s 이상 끊김
            err = "EtherCAT 통신 끊김";
        }
        if (g_stop) err = "사용자 중단(Ctrl+C)";
        std::this_thread::sleep_for(std::chrono::microseconds(cfg.period_us));
        return err.empty();
    }
    bool enabled() const { return fb.status == 0x27; }
};

// 위치 모드로 target 까지 목표를 조금씩 옮기며 이동 → 도착 후 settle
static bool move_to(Rig& r, int32_t target) {
    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
    r.cmd.kp = r.cfg.kp; r.cmd.kd = r.cfg.kd;
    double sp = r.fb.pos;                       // 현재 위치에서 출발 (점프 없음)
    const double step = r.cfg.move_speed_cps * r.cfg.period_us * 1e-6;
    auto settle_start = clk::time_point{};
    bool arrived = false;
    const auto deadline = clk::now() + std::chrono::seconds(
        10 + static_cast<int>(std::abs(target - sp) / r.cfg.move_speed_cps));

    while (true) {
        if (sp < target) sp = std::min<double>(sp + step, target);
        else             sp = std::max<double>(sp - step, target);
        r.cmd.target_pos = static_cast<int32_t>(std::lround(sp));
        if (!r.tick()) return false;

        if (std::abs(r.fb.pos - r.cmd.target_pos) > r.cfg.max_follow_error) {
            r.err = "이동 중 목표를 못 따라감 (원판이 걸렸거나 게인 문제)"; return false;
        }
        if (sp == target && std::abs(r.fb.pos - target) < r.cfg.position_tolerance) {
            if (!arrived) { arrived = true; settle_start = clk::now(); }
            if (clk::now() - settle_start >= std::chrono::duration<double>(r.cfg.settle_s)) return true;
        } else {
            arrived = false;
        }
        if (clk::now() > deadline) { r.err = "목표 위치 도착 시간 초과"; return false; }
    }
}

// 토크 모드로 ±torque_mag 사각파. 매 루프 CSV 기록
static bool vibrate(Rig& r, int pos_idx, FILE* csv, const clk::time_point t0) {
    const int16_t A = static_cast<int16_t>(std::clamp(r.cfg.torque_mag,
                                         -r.cfg.torque_hard_limit, r.cfg.torque_hard_limit));
    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    r.cmd.target_tor = A;
    const int32_t hold = r.fb.pos;
    const auto start = clk::now();
    auto last_flip = start;
    int half = 0;

    while (clk::now() - start < std::chrono::duration<double>(r.cfg.measure_time_s)) {
        if (!r.tick()) { r.cmd.target_tor = 0; return false; }
        auto now = clk::now();
        if (now - last_flip >= std::chrono::milliseconds(r.cfg.flip_ms)) {
            r.cmd.target_tor = static_cast<int16_t>(-r.cmd.target_tor);
            last_flip = now; ++half;
        }
        if (std::abs(r.fb.pos - hold) > r.cfg.max_vibration_travel) {
            r.cmd.target_tor = 0;
            r.err = "진동 중 원판이 너무 많이 밀림 - torque_mag 를 줄이거나 원판 고정 확인";
            return false;
        }
        double t = std::chrono::duration<double>(now - t0).count();
        std::fprintf(csv, "%.5f,%d,%d,%d,%d,%d,%d\n",
                     t, pos_idx, half, r.cmd.target_tor, r.fb.tor, r.fb.pos, r.fb.temp);
    }
    r.cmd.target_tor = 0;
    return true;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::puts("Usage: backlash_test <ifname> <backlash.yaml> <out.csv>\n"
                  "  e.g., sudo ./backlash_test enp3s0 ../src/backlash.yaml backlash_0cycle.csv");
        return 1;
    }
    std::signal(SIGINT, on_sigint);
    const Cfg cfg = load_cfg(argv[2]);
    const char* csv_path = argv[3];

    FILE* csv = std::fopen(csv_path, "w");
    if (!csv) { std::perror("CSV 파일 열기 실패"); return 1; }
    std::fprintf(csv, "# gear_ratio=%.4f\n# counts_per_rev=%d\n# torque_mag=%d\n# flip_ms=%d\n",
                 cfg.gear_ratio, cfg.counts_per_rev, cfg.torque_mag, cfg.flip_ms);
    std::fprintf(csv, "t_s,pos_idx,half,cmd_tor,tor,pos,temp\n");

    int code = 0;
    try {
        EthercatMaster master(argv[1]);
        if (master.slaveCount() != 1) {
            std::fprintf(stderr, "구동기 1개만 연결한 상태를 가정함 (발견: %d개)\n", master.slaveCount());
            std::fclose(csv); return 2;
        }
        ActuatorPDOMap map{};
        EthercatActuator act(1, map);
        Rig r{master, act, {}, {}, cfg};
        r.cmd.controlword = CW_FAULT_RESET;
        r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;   // 켜지는 동안은 토크 0
        r.cmd.target_tor = 0;

        // ---- 1. 켜기 (최대 10초)
        std::puts("구동기 켜는 중...");
        const auto on_deadline = clk::now() + std::chrono::seconds(10);
        while (!r.enabled()) {
            if (!r.tick()) break;
            if (clk::now() > on_deadline) { r.err = "구동기가 Operation Enabled 가 되지 않음"; break; }
        }

        // ---- 2. 측정 위치마다: 이동 → 진동
        const int32_t origin = r.fb.pos;
        const double out_rev = cfg.counts_per_rev * cfg.gear_ratio;   // 출력 1바퀴 = 모터 카운트
        const auto t0 = clk::now();
        for (int i = 0; r.err.empty() && i < cfg.num_positions; ++i) {
            int32_t target = origin + static_cast<int32_t>(std::lround(out_rev * i / cfg.num_positions));
            std::printf("[%d/%d] 위치 %d 로 이동 (출력 %.0f도)\n", i + 1, cfg.num_positions,
                        target, 360.0 * i / cfg.num_positions);
            if (!move_to(r, target)) break;
            std::printf("[%d/%d] 진동 측정 %.1f초\n", i + 1, cfg.num_positions, cfg.measure_time_s);
            if (!vibrate(r, i, csv, t0)) break;
        }

        // ---- 3. 토크 0 으로 0.5초 유지 후 끄기
        r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        r.cmd.target_tor = 0;
        std::string reason = r.err.empty() ? "완료" : r.err;
        if (!r.err.empty()) code = 3;
        r.err.clear(); g_stop = 0;
        for (int k = 0; k < 1000; ++k) r.tick();
        r.cmd.controlword = CW_SHUTDOWN;
        act.writeCommand(r.cmd); master.tickOnce();
        std::printf("종료: %s\n", reason.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        code = 1;
    }
    std::fclose(csv);
    std::printf("CSV 저장: %s\n", csv_path);
    return code;
}
