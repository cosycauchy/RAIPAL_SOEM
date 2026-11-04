// src/endurance_test.cpp
//
// 10 kg 원판을 단 상태에서 구동기를 정/역으로 돌리는 내구(신뢰성) 시험.
//
// 한 사이클 = (정격 토크) 정방향 1바퀴 + 역방향 1바퀴, (최대 토크) 정방향 1바퀴 + 역방향 1바퀴
//
// 원판만 달려 있으면 기어에 걸리는 토크는 "원판을 가속/감속시키는 토크"뿐이라서,
// 1바퀴 내내 정격 토크를 걸면 원판이 너무 빨라진다. 그래서 한 바퀴를 이렇게 돌린다.
//     가속(정격 또는 최대 토크) → 일정 속도로 관성 주행(토크 0) → 반대 토크로 감속 → 위치 모드로 정확히 정지
// 토크는 가속/감속 구간에만 걸리고, 방향이 바뀔 때마다 기어 틈에서 충격이 생긴다.
//
// 그 밖의 기능:
//   - 정해진 사이클마다 백래시 측정 (0, 100, 200, ... 1000, 이후 200마다). backlash_test 와 같은 방식/같은 CSV.
//   - 백래시가 limit_deg 이상이거나 total_cycles 에 도달하면 종료
//   - 온도 60°C 에서 일시정지, 50°C 로 식으면 재개 (70°C 는 절대 넘기지 않음)
//   - 사이클 수를 파일에 저장해서 다음에 이어서 실행 (1시간씩 끊어 돌릴 수 있음)
//
// 사용법:  sudo ./endurance_test <ifname> <endurance.yaml>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <csignal>
#include <ctime>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "ethercat_master.hpp"
#include "ethercat_actuator.hpp"

using clk = std::chrono::steady_clock;

static volatile std::sig_atomic_t g_stop = 0;
static void on_sigint(int) { g_stop = 1; }

// ---------------------------------------------------------------- 설정
struct Cfg {
    double gear_ratio     = 28.35;
    int    counts_per_rev = 65536;    // 모터 엔코더 1회전 카운트 (확인 필요)
    int    period_us      = 500;

    // ---- 사이클 (토크 단위: 1000 = 정격 토크)
    int    torque_rated   = 1000;     // 정격 토크 구간
    int    torque_peak    = 3250;     // 최대 토크 구간 (48.8 Nm / 15 Nm = 3.25)
    double torque_scale   = 1.0;      // 처음 시험 때 0.2 같은 값으로 줄여서 쓴다
    int    torque_hard_limit = 3400;  // 코드가 절대 넘기지 않는 토크
    double motor_rated_nm = 0.53;     // 화면 출력용
    double cruise_rpm_out = 80;       // 일정 속도 구간의 출력축 속도
    double abort_rpm_out  = 130;      // 이 속도를 넘으면 즉시 정지
    double cruise_kp      = 500;      // 일정 속도 구간: 속도 오차 100%당 걸 토크 [단위]
    double cruise_torque_max = 100;   // 일정 속도 구간에서 걸 수 있는 최대 토크 (마찰 보상용, 약 1.5 Nm)
    double accel_end_frac = 0.92;     // 일정 속도의 이 비율에 도달하면 가속 토크를 끈다 (지연 때문에 속도가 넘치는 것 방지)
    int    finish_tolerance = 3000;   // 한 바퀴 끝에서 이 정도(약 0.6도) 안에 들어오면 도착으로 본다
    double decel_margin   = 1.2;      // 감속을 목표보다 이만큼 일찍 시작 (1.2 = 20% 일찍)
    double stop_speed_frac= 0.05;     // 감속 후 이 속도(일정 속도의 3%) 아래면 위치 모드로 넘김
    double rev_timeout_s  = 15;       // 한 바퀴에 이 시간 넘으면 중단
    double settle_s       = 0.15;     // 한 바퀴 끝나고 멈춰 있는 시간

    // ---- 종료 조건 / 이어하기
    int    trace_cycles   = 1;        // 처음 N사이클은 모터 움직임을 trace_file 에 기록 (0 = 안 함)
    int    total_cycles   = 3000;
    int    run_cycles_limit = 0;      // 이번 실행에서 돌릴 최대 사이클 (0 = 제한 없음)
    double max_run_minutes  = 60;     // 이번 실행 최대 시간 (0 = 제한 없음)
    double limit_deg      = 0.2;      // 출력축 백래시가 이 이상이면 종료

    // ---- 백래시 측정 일정
    int    measure_first_every = 100;
    int    measure_first_until = 1000;
    int    measure_later_every = 200;

    // ---- 백래시 측정 방식 (backlash_test 와 같음)
    int    bl_num_positions  = 8;
    double bl_move_speed_cps = 100000;   // 위치 사이 이동 속도. 100000 = 출력 약 20도/초
    int    bl_settle_s_x10   = 10;       // 도착 후 대기 [0.1초 단위]
    int    bl_torque_mag     = 20;
    int    bl_flip_ms        = 15;
    double bl_measure_time_s = 2.0;
    int    bl_torque_hard_limit = 100;
    int    bl_max_vibration_travel = 3000;
    int    bl_skip_halves    = 4;
    double cool_before_measure_c = 0;    // >0 이면 측정 전에 이 온도까지 식힌다 (0 = 안 함)

    // ---- 위치 모드 / 안전
    int    position_tolerance = 500;
    int    max_follow_error   = 20000;
    double finish_speed_cps   = 100000;  // 한 바퀴 끝에서 남은 거리를 위치 모드로 맞출 때 속도
    int    finish_follow_error= 60000;   // 한 바퀴 끝에서 위치 모드로 넘길 때 허용하는 오차 (아직 조금 움직이는 중이라 더 넉넉히)
    uint16_t kp = 0, kd = 0;
    int    temp_pause_c  = 60;
    int    temp_resume_c = 50;
    int    temp_abort_c  = 68;
    double cool_max_min  = 90;

    // ---- 파일
    std::string data_dir     = ".";
    std::string state_file   = "endurance_state.txt";
    std::string cycle_log    = "endurance_log.csv";
    std::string backlash_log = "backlash_log.csv";
    std::string trace_file   = "rev_trace.csv";

    double out_rev_counts() const { return counts_per_rev * gear_ratio; }
    double cps_from_rpm(double rpm) const { return rpm * out_rev_counts() / 60.0; }
    double rpm_from_cps(double cps) const { return cps * 60.0 / out_rev_counts(); }
    double deg_from_counts(double c) const { return c / out_rev_counts() * 360.0; }
    double limit_counts() const { return limit_deg / 360.0 * out_rev_counts(); }
    std::string path(const std::string& name) const { return data_dir + "/" + name; }
};

static Cfg load_cfg(const char* path) {
    YAML::Node n = YAML::LoadFile(path)["endurance_test"];
    if (!n) { std::fprintf(stderr, "Config error: 'endurance_test' 항목이 없음\n"); std::exit(2); }
    Cfg c;
#define RD(key, type) if (n[#key]) c.key = n[#key].as<type>()
    RD(gear_ratio, double); RD(counts_per_rev, int); RD(period_us, int);
    RD(torque_rated, int); RD(torque_peak, int); RD(torque_scale, double);
    RD(torque_hard_limit, int); RD(motor_rated_nm, double);
    RD(cruise_rpm_out, double); RD(abort_rpm_out, double); RD(decel_margin, double);
    RD(stop_speed_frac, double); RD(accel_end_frac, double); RD(finish_tolerance, int); RD(cruise_kp, double); RD(cruise_torque_max, double); RD(rev_timeout_s, double); RD(settle_s, double);
    RD(total_cycles, int); RD(trace_cycles, int); RD(trace_file, std::string); RD(run_cycles_limit, int); RD(max_run_minutes, double);
    RD(limit_deg, double);
    RD(measure_first_every, int); RD(measure_first_until, int); RD(measure_later_every, int);
    RD(bl_num_positions, int); RD(bl_move_speed_cps, double); RD(bl_settle_s_x10, int);
    RD(bl_torque_mag, int); RD(bl_flip_ms, int); RD(bl_measure_time_s, double);
    RD(bl_torque_hard_limit, int); RD(bl_max_vibration_travel, int); RD(bl_skip_halves, int);
    RD(cool_before_measure_c, double);
    RD(position_tolerance, int); RD(max_follow_error, int); RD(finish_follow_error, int); RD(finish_speed_cps, double); RD(kp, uint16_t); RD(kd, uint16_t);
    RD(temp_pause_c, int); RD(temp_resume_c, int); RD(temp_abort_c, int); RD(cool_max_min, double);
    RD(data_dir, std::string); RD(state_file, std::string);
    RD(cycle_log, std::string); RD(backlash_log, std::string);
#undef RD
    auto bad = [](const char* msg) { std::fprintf(stderr, "Config error: %s\n", msg); std::exit(2); };
    if (c.torque_scale <= 0 || c.torque_scale > 1.0) bad("torque_scale 은 0 초과 1 이하");
    if (std::max(c.torque_rated, c.torque_peak) * c.torque_scale > c.torque_hard_limit)
        bad("torque_peak x torque_scale 이 torque_hard_limit 을 넘음");
    if (c.torque_rated <= 0 || c.torque_peak <= 0) bad("torque_rated/torque_peak 는 양수");
    if (c.cruise_rpm_out <= 0 || c.cruise_rpm_out >= c.abort_rpm_out) bad("cruise_rpm_out < abort_rpm_out 이어야 함");
    if (c.period_us < 100) bad("period_us >= 100");
    if (c.bl_num_positions < 1 || c.bl_flip_ms < 1 || c.bl_move_speed_cps <= 0) bad("bl_* 값 범위 확인");
    if (std::abs(c.bl_torque_mag) > c.bl_torque_hard_limit) bad("bl_torque_mag > bl_torque_hard_limit");
    if (c.measure_first_every < 1 || c.measure_later_every < 1) bad("measure_*_every >= 1");
    if (c.temp_resume_c >= c.temp_pause_c || c.temp_pause_c >= c.temp_abort_c) bad("temp_resume < temp_pause < temp_abort 이어야 함");
    if (c.temp_abort_c >= 70) bad("temp_abort_c 는 70 미만 (70°C 는 절대 금지)");
    if (c.decel_margin < 1.0) bad("decel_margin >= 1.0");
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
    std::string       err;
    int               temp_max = 0;          // 호출한 쪽이 0 으로 되돌려 쓰는 기록용
    FILE*             trace = nullptr;       // 움직임 기록 파일 (nullptr = 기록 안 함)
    int               phase_tag = 0;         // 기록에 같이 남기는 구간 번호 (바퀴*10 + 0 가속/1 주행/2 감속/3 위치 마무리)
    int               trace_n = 0;
    double            trace_t0 = 0;
    clk::time_point   next_tick = clk::now();

    // 속도 추정용 최근 위치 기록 (약 4 ms 구간)
    static constexpr int NS = 8;
    double  ts[NS]{}; int32_t ps[NS]{}; int ns = 0, head = 0;

    static double now_s() { return std::chrono::duration<double>(clk::now().time_since_epoch()).count(); }

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
            if (fb.status == 0x08)               err = "구동기 Fault";
            if (fb.temp >= cfg.temp_abort_c)     err = "온도 제한 초과 (중단)";
            temp_max = std::max<int>(temp_max, fb.temp);
            ts[head] = now_s(); ps[head] = fb.pos;
            head = (head + 1) % NS; ns = std::min(ns + 1, NS);
            if (trace && (++trace_n % 4 == 0))
                std::fprintf(trace, "%.4f,%d,%d,%d,%d,%d,%d,%.1f,%d\n", now_s() - trace_t0, phase_tag, fb.pos, cmd.target_pos,
                             cmd.target_tor, fb.tor, cmd.mode, cfg.rpm_from_cps(speed_cps()), fb.temp);
        } else if (++wkc_fail > 200) {
            err = "EtherCAT 통신 끊김";
        }
        if (g_stop) err = "사용자 중단(Ctrl+C)";
        // 일정한 주기로 돈다 (통신이 늦어 밀리면 따라잡으려고 서두르지 않고 지금부터 다시 센다)
        next_tick += std::chrono::microseconds(cfg.period_us);
        auto nw = clk::now();
        if (next_tick < nw) next_tick = nw;
        std::this_thread::sleep_until(next_tick);
        return err.empty();
    }
    bool enabled() const { return fb.status == 0x27; }

    // 모터 엔코더 속도 [카운트/초]. 최근 몇 ms 의 위치 변화로 계산 (단위 모르는 fb.vel 은 쓰지 않음)
    double speed_cps() const {
        if (ns < 2) return 0;
        int newest = (head - 1 + NS) % NS;
        int oldest = (ns < NS) ? 0 : head;
        double dt = ts[newest] - ts[oldest];
        return dt > 0 ? (double(ps[newest]) - double(ps[oldest])) / dt : 0;
    }
};

// 위치 모드로 target 까지 목표를 조금씩 옮기며 이동 → 도착 후 settle_s 동안 유지
static bool move_to(Rig& r, int32_t target, double speed_cps, double settle_s, int follow_limit = -1, int tol = -1) {
    if (follow_limit < 0) follow_limit = r.cfg.max_follow_error;
    if (tol < 0) tol = r.cfg.position_tolerance;
    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_POSITION;
    r.cmd.target_tor = 0;
    r.cmd.kp = r.cfg.kp; r.cmd.kd = r.cfg.kd;
    double sp = r.fb.pos;
    clk::time_point settle_start{};
    auto last_t = clk::now();
    bool arrived = false;
    const auto deadline = clk::now() + std::chrono::seconds(
        10 + static_cast<int>(1.5 * std::abs(target - sp) / speed_cps));

    while (true) {
        // 목표를 실제 흐른 시간만큼 옮긴다 (주기가 느려져도 속도가 그대로)
        auto now_t = clk::now();
        const double step = speed_cps * std::min(std::chrono::duration<double>(now_t - last_t).count(), 0.01);
        last_t = now_t;
        if (sp < target) sp = std::min<double>(sp + step, target);
        else             sp = std::max<double>(sp - step, target);
        r.cmd.target_pos = static_cast<int32_t>(std::lround(sp));
        if (!r.tick()) return false;

        if (std::abs(r.fb.pos - r.cmd.target_pos) > follow_limit) {
            char buf[200];
            std::snprintf(buf, sizeof buf, "이동 중 목표를 못 따라감 (오차 %d 카운트, 한계 %d, 속도 %.0f rpm) - 원판이 걸렸거나 게인 문제",
                          r.fb.pos - r.cmd.target_pos, follow_limit, r.cfg.rpm_from_cps(r.speed_cps()));
            r.err = buf; return false;
        }
        if (sp == target && std::abs(r.fb.pos - target) < tol) {
            if (!arrived) { arrived = true; settle_start = clk::now(); }
            if (clk::now() - settle_start >= std::chrono::duration<double>(settle_s)) return true;
        } else {
            arrived = false;
        }
        if (clk::now() > deadline) {
            char buf[200];
            std::snprintf(buf, sizeof buf, "목표 위치 도착 시간 초과 (남은 오차 %d 카운트, 허용 %d, 목표 지령은 %s)",
                          r.fb.pos - target, tol, sp == target ? "도달함" : "아직 도달 못함");
            r.err = buf; return false;
        }
    }
}

// ---------------------------------------------------------------- 온도 대기
// 토크 0 으로 두고 목표 온도 이하가 될 때까지 기다린다. 기다린 시간은 waited_s 에 더한다.
static bool wait_until_cool(Rig& r, int below_c, double& waited_s, const char* why) {
    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    r.cmd.target_tor = 0;
    const double t0 = Rig::now_s();
    double last_print = t0;
    std::printf("[온도] %s - 현재 %d°C, %d°C 이하가 될 때까지 대기\n", why, r.fb.temp, below_c);
    while (r.fb.temp > below_c) {
        if (!r.tick()) return false;
        double t = Rig::now_s();
        if (t - t0 > r.cfg.cool_max_min * 60) { r.err = "식는 데 너무 오래 걸림 (선풍기/환경 확인)"; return false; }
        if (t - last_print >= 30) {
            std::printf("[온도] 대기 중... %d°C (%.0f분 경과)\n", r.fb.temp, (t - t0) / 60);
            last_print = t;
        }
    }
    waited_s += Rig::now_s() - t0;
    std::printf("[온도] %d°C - 재개\n", r.fb.temp);
    return true;
}

static bool pause_if_hot(Rig& r, double& paused_s) {
    if (r.fb.temp < r.cfg.temp_pause_c) return true;
    return wait_until_cool(r, r.cfg.temp_resume_c, paused_s, "드라이버가 뜨거워 일시정지");
}

// ---------------------------------------------------------------- 한 바퀴
struct RevStats { double acc_deg = 0; double peak_cps = 0; double left_deg = 0; double dec_ms = 0; };

// 현재 위치에서 to 까지 한 바퀴: 가속(토크) → 일정 속도(약한 토크로 유지) → 감속(반대 토크) → 위치 모드로 정지
// decel_scale: 감속을 "정지거리의 몇 배" 남았을 때 시작할지. 바퀴를 돌 때마다 남은 각도를 보고 스스로 맞춘다.
static bool run_rev(Rig& r, int32_t to, int torque_units, RevStats& st, int tag_base, double& decel_scale) {
    const Cfg& c = r.cfg;
    const int32_t from = r.fb.pos;
    const int     dir  = (to > from) ? 1 : -1;
    const double  dist = std::abs(double(to) - double(from));
    const double  vc   = c.cps_from_rpm(c.cruise_rpm_out);
    const double  vmax = c.cps_from_rpm(c.abort_rpm_out);
    const int16_t T    = static_cast<int16_t>(std::lround(torque_units * c.torque_scale));

    enum { ACC, COAST, DEC } ph = ACC;
    int over_cnt = 0, stop_cnt = 0;           // 연속 판정 횟수 (타이밍/모터 틈 때문에 생기는 순간 잡음에 속지 않도록)
    double a_est = 0;                         // 가속 구간에서 구한 가속도 [카운트/초^2]
    double dec_v0 = 0, dec_done0 = 0, dec_remain0 = 0, d_est = 0;
    clk::time_point dec_t0{};
    bool dec_ok = true;                       // 감속이 정상적으로 끝났는지 (이상하면 decel_scale 을 고치지 않는다)
    double vh[24] = {}; int vh_i = 0, vh_n = 0;   // 최근 약 12 ms 속도 평균 (감속 직후 모터 쪽 속도가 순간 튀는 것을 평균으로 눌러줌)
    const auto t_begin = clk::now();

    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    r.cmd.target_tor = static_cast<int16_t>(dir * T);

    auto enter_dec = [&](double v, double done, double remain) {
        ph = DEC; r.cmd.target_tor = static_cast<int16_t>(-dir * T);
        dec_v0 = std::max(v, 1.0); dec_done0 = done; dec_remain0 = remain;
        d_est = dec_v0 * dec_v0 / (2.0 * a_est); dec_t0 = clk::now();
    };

    while (true) {
        r.phase_tag = tag_base + static_cast<int>(ph);
        if (!r.tick()) { r.cmd.target_tor = 0; return false; }
        const double v      = r.speed_cps() * dir;                 // 가는 방향이 +
        const double done   = (double(r.fb.pos) - from) * dir;     // 지금까지 간 거리
        const double remain = dist - done;
        vh[vh_i] = v; vh_i = (vh_i + 1) % 24; vh_n = std::min(vh_n + 1, 24);
        double v_avg = 0; for (int i = 0; i < vh_n; ++i) v_avg += vh[i]; v_avg /= vh_n;
        // 가속/주행 판단에는 최근 9개의 중앙값을 쓴다 (토크를 걸자마자 모터가 기어 틈을 지나며 속도가 한두 번 튀는 것을 무시)
        double v_med;
        { const int n = std::min(9, vh_n); double tmp[9];
          for (int k = 0; k < n; ++k) tmp[k] = vh[(vh_i - 1 - k + 48) % 24];
          std::nth_element(tmp, tmp + n / 2, tmp + n); v_med = tmp[n / 2]; }

        over_cnt = (v > vmax) ? over_cnt + 1 : 0;
        if (over_cnt >= 5) {
            char buf[160];
            std::snprintf(buf, sizeof buf, "속도 한도 초과 (%.0f rpm, 구간 %d) - 원판 관성이 예상보다 작거나 토크 단위 확인",
                          c.rpm_from_cps(v), static_cast<int>(ph));
            r.cmd.target_tor = 0; r.err = buf; return false;
        }
        if (clk::now() - t_begin > std::chrono::duration<double>(c.rev_timeout_s)) {
            char buf[200];
            std::snprintf(buf, sizeof buf, "한 바퀴가 너무 오래 걸림 (구간 %d, %.0f rpm, 진행 %.0f/%.0f도, 토크 %d) - 마찰/관성/토크 확인",
                          static_cast<int>(ph), c.rpm_from_cps(v), c.deg_from_counts(done), c.deg_from_counts(dist), r.cmd.target_tor);
            r.cmd.target_tor = 0; r.err = buf; return false;
        }
        st.peak_cps = std::max(st.peak_cps, v_avg);                // 표시용 최고 속도는 순간 잡음을 뺀 평균으로

        if (ph == ACC) {
            if (v_med >= c.accel_end_frac * vc) {                   // 일정 속도 근처 도달 → 가속 토크 끄기
                a_est = v_med * v_med / (2.0 * std::max(done, 1.0));
                st.acc_deg = c.deg_from_counts(done);
                r.cmd.target_tor = 0; ph = COAST;
            } else if (done > 2000 && v_med > 0.05 * vc) {          // 속도 못 올리고 반 넘게 옴 → 삼각 프로파일
                double a = v_med * v_med / (2.0 * done);
                if (remain <= decel_scale * v_med * v_med / (2.0 * a)) {
                    a_est = a; st.acc_deg = c.deg_from_counts(done);
                    enter_dec(v_med, done, remain);
                }
            }
        } else if (ph == COAST) {
            if (remain <= decel_scale * v_med * v_med / (2.0 * a_est)) {   // 정지거리만큼 남으면 감속 시작
                enter_dec(v_med, done, remain);
            } else {                                                // 마찰로 느려지지 않게 약한 토크로 속도 유지
                const double cap = std::min<double>(c.cruise_torque_max, T);
                const double tc  = std::clamp(c.cruise_kp * (vc - v_med) / vc, -cap, cap);
                r.cmd.target_tor = static_cast<int16_t>(std::lround(dir * tc));
            }
        } else {                                                    // DEC
            const double t_dec = std::chrono::duration<double>(clk::now() - dec_t0).count();
            const double t_exp = dec_v0 / a_est;                    // 감속에 걸릴 예상 시간
            // 거의 멈춤이 이어져도, 예상 감속 시간/거리의 일부는 지나야 진짜로 멈춘 것으로 본다
            const bool plausible = t_dec >= 0.45 * t_exp && (done - dec_done0) >= 0.25 * d_est;
            stop_cnt = (v_avg <= c.stop_speed_frac * vc) ? stop_cnt + 1 : 0;
            // 멈춘 것을 확인하고 끄면 늦어서 (최대 토크에서는 그 짧은 사이에 원판이 반대로 돈다) 예상 감속 시간의 85% 가 지나면 먼저 끈다.
            // 남은 속도는 마찰과 위치 모드가 마무리한다.
            if (t_dec >= 0.85 * t_exp) break;
            if (stop_cnt >= 8 && plausible) break;
            if (v_avg <= -0.3 * vc) { dec_ok = false; break; }      // 반대로 가기 시작함 → 토크 끄기
            if (t_dec > 3.0 * t_exp + 0.2) { dec_ok = false; break; }
        }
    }
    // 이번 바퀴에서 실제로 감속에 쓴 거리를 보고 다음 바퀴의 감속 시작 위치를 맞춘다 (목표 약간 못 미치게)
    const double done_stop = (double(r.fb.pos) - from) * dir;
    st.left_deg = c.deg_from_counts(dist - done_stop);
    st.dec_ms = std::chrono::duration<double>(clk::now() - dec_t0).count() * 1000.0;
    if (dec_ok && d_est > 0) {
        double d_act = dec_remain0 - (dist - done_stop);
        double want = std::clamp(1.08 * d_act / d_est, 0.5, 1.6);
        decel_scale = 0.5 * decel_scale + 0.5 * want;
    }
    r.cmd.target_tor = 0;
    r.phase_tag = tag_base + 3;
    return move_to(r, to, c.finish_speed_cps, c.settle_s, c.finish_follow_error, c.finish_tolerance);   // 남은 거리는 위치 모드로
}

// 한 사이클 = 정격(정,역) + 최대(정,역). 각 바퀴 사이에서 온도를 보고 필요하면 쉰다.
struct CycleStats { double acc_deg_rated = 0, acc_deg_peak = 0, peak_rpm = 0, paused_s = 0, left_deg = 0; };

static bool run_cycle(Rig& r, int32_t origin, CycleStats& cs, double* scales) {
    const int32_t far = origin + static_cast<int32_t>(std::lround(r.cfg.out_rev_counts()));
    const struct { int torque; double* acc; } phases[2] = {
        { r.cfg.torque_rated, &cs.acc_deg_rated }, { r.cfg.torque_peak, &cs.acc_deg_peak } };
    for (const auto& ph : phases) {
        for (int k = 0; k < 2; ++k) {
            if (!pause_if_hot(r, cs.paused_s)) return false;
            RevStats st;
            const int slot = static_cast<int>(&ph - phases) * 2 + k;
            if (!run_rev(r, k == 0 ? far : origin, ph.torque, st, slot * 10, scales[slot])) return false;
            cs.left_deg = std::max(cs.left_deg, std::abs(st.left_deg));
            if (k == 0) *ph.acc = st.acc_deg;
            cs.peak_rpm = std::max(cs.peak_rpm, r.cfg.rpm_from_cps(st.peak_cps));
        }
    }
    return true;
}

// ---------------------------------------------------------------- 백래시 측정 (backlash_test + analyze_backlash.py 와 같은 방식)
struct Sample { int half; int32_t pos; };

static bool vibrate(Rig& r, int pos_idx, FILE* csv, clk::time_point t0, std::vector<Sample>& out) {
    const Cfg& c = r.cfg;
    const int16_t A = static_cast<int16_t>(std::clamp(c.bl_torque_mag, -c.bl_torque_hard_limit, c.bl_torque_hard_limit));
    r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
    r.cmd.target_tor = A;
    const int32_t hold = r.fb.pos;
    const auto start = clk::now();
    auto last_flip = start;
    int half = 0;
    while (clk::now() - start < std::chrono::duration<double>(c.bl_measure_time_s)) {
        if (!r.tick()) { r.cmd.target_tor = 0; return false; }
        auto now = clk::now();
        if (now - last_flip >= std::chrono::milliseconds(c.bl_flip_ms)) {
            r.cmd.target_tor = static_cast<int16_t>(-r.cmd.target_tor);
            last_flip = now; ++half;
        }
        if (std::abs(r.fb.pos - hold) > c.bl_max_vibration_travel) {
            r.cmd.target_tor = 0;
            r.err = "진동 중 원판이 너무 많이 밀림 - bl_torque_mag 를 줄이거나 원판 고정 확인";
            return false;
        }
        double t = std::chrono::duration<double>(now - t0).count();
        std::fprintf(csv, "%.5f,%d,%d,%d,%d,%d,%d\n", t, pos_idx, half, r.cmd.target_tor, r.fb.tor, r.fb.pos, r.fb.temp);
        out.push_back({half, r.fb.pos});
    }
    r.cmd.target_tor = 0;
    return true;
}

static double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

struct Meas { double backlash_cnt = 0; double senior_max_cnt = 0; int temp_max = 0; std::string csv_name; };

// 위치별 진동 기록으로 백래시 계산 (analyze_backlash.py 와 같은 계산)
static bool compute_backlash(const std::vector<std::vector<Sample>>& per_pos, int skip, Meas& m) {
    std::vector<double> medians;
    double senior_max = 0;
    for (const auto& samples : per_pos) {
        std::map<int, std::vector<int32_t>> halves;
        for (const auto& s : samples) halves[s.half].push_back(s.pos);
        std::map<int, double> level;
        for (const auto& [h, ps] : halves) {
            if (h < skip || ps.size() < 6) continue;
            size_t tail = std::max<size_t>(2, ps.size() / 3);
            std::vector<double> t(ps.end() - tail, ps.end());
            level[h] = median(t);
            senior_max = std::max<double>(senior_max, *std::max_element(ps.begin(), ps.end()) - *std::min_element(ps.begin(), ps.end()));
        }
        std::vector<double> diffs;
        for (const auto& [h, lv] : level)
            if (level.count(h - 1)) diffs.push_back(std::abs(lv - level[h - 1]));
        if (!diffs.empty()) medians.push_back(median(diffs));
    }
    if (medians.empty()) return false;
    double sum = 0; for (double x : medians) sum += x;
    m.backlash_cnt = sum / medians.size();
    m.senior_max_cnt = senior_max;
    return true;
}

static std::string now_str() {
    std::time_t t = std::time(nullptr); char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M", std::localtime(&t));
    return buf;
}

static bool file_exists(const std::string& p) { FILE* f = std::fopen(p.c_str(), "r"); if (f) std::fclose(f); return f != nullptr; }

static bool measure_backlash(Rig& r, int32_t origin, int cycle, Meas& m) {
    const Cfg& c = r.cfg;
    m.csv_name = "backlash_" + std::to_string(cycle) + "cycle.csv";
    FILE* csv = std::fopen(c.path(m.csv_name).c_str(), "w");
    if (!csv) { r.err = "측정 CSV 파일을 열 수 없음: " + c.path(m.csv_name); return false; }
    std::fprintf(csv, "# gear_ratio=%.4f\n# counts_per_rev=%d\n# torque_mag=%d\n# flip_ms=%d\n",
                 c.gear_ratio, c.counts_per_rev, c.bl_torque_mag, c.bl_flip_ms);
    std::fprintf(csv, "t_s,pos_idx,half,cmd_tor,tor,pos,temp\n");

    std::vector<std::vector<Sample>> per_pos(c.bl_num_positions);
    r.temp_max = 0;
    const auto t0 = clk::now();
    bool ok = true;
    for (int i = 0; ok && i < c.bl_num_positions; ++i) {
        int32_t target = origin + static_cast<int32_t>(std::lround(c.out_rev_counts() * i / c.bl_num_positions));
        std::printf("  [측정 %d/%d] 출력 %.0f도로 이동 후 진동\n", i + 1, c.bl_num_positions, 360.0 * i / c.bl_num_positions);
        ok = move_to(r, target, c.bl_move_speed_cps, c.bl_settle_s_x10 / 10.0)
          && vibrate(r, i, csv, t0, per_pos[i]);
    }
    std::fclose(csv);
    if (!ok) return false;
    if (!move_to(r, origin, c.bl_move_speed_cps, c.settle_s, c.finish_follow_error, c.finish_tolerance)) return false;   // 시작 자리로 복귀 (정확히 맞출 필요 없음)
    m.temp_max = r.temp_max;
    if (!compute_backlash(per_pos, c.bl_skip_halves, m)) { r.err = "백래시를 계산할 구간이 없음 (bl_measure_time_s 확인)"; return false; }

    // backlash_log.csv 에 한 줄 추가 (analyze_backlash.py --cycle 과 같은 형식 → plot_trend.py 가 그대로 읽음)
    const std::string logp = c.path(c.backlash_log);
    bool is_new = !file_exists(logp);
    if (FILE* f = std::fopen(logp.c_str(), "a")) {
        if (is_new) std::fputs("datetime,cycle,backlash_cnt,backlash_deg,senior_max_cnt,temp_max_c,torque_mag,csv_file,note\n", f);
        std::fprintf(f, "%s,%d,%.0f,%.4f,%.0f,%d,%d,%s,endurance\n", now_str().c_str(), cycle,
                     m.backlash_cnt, c.deg_from_counts(m.backlash_cnt) , m.senior_max_cnt,
                     m.temp_max, c.bl_torque_mag, m.csv_name.c_str());
        std::fclose(f);
    }
    return true;
}

// ---------------------------------------------------------------- 이어하기용 상태 파일 / 사이클 로그
struct State { int cycles_done = 0; int last_measured = -1; };

static State load_state(const Cfg& c) {
    State s;
    if (FILE* f = std::fopen(c.path(c.state_file).c_str(), "r")) {
        char key[64]; int val;
        while (std::fscanf(f, " %63[^=]=%d", key, &val) == 2) {
            if (std::string(key) == "cycles_done") s.cycles_done = val;
            if (std::string(key) == "last_measured") s.last_measured = val;
        }
        std::fclose(f);
    }
    return s;
}

static void save_state(const Cfg& c, const State& s) {
    const std::string tmp = c.path(c.state_file) + ".tmp";
    if (FILE* f = std::fopen(tmp.c_str(), "w")) {
        std::fprintf(f, "cycles_done=%d\nlast_measured=%d\n", s.cycles_done, s.last_measured);
        std::fclose(f);
        std::rename(tmp.c_str(), c.path(c.state_file).c_str());
    }
}

static bool measure_due(const Cfg& c, int cycle) {
    return cycle <= c.measure_first_until ? cycle % c.measure_first_every == 0
                                          : cycle % c.measure_later_every == 0;
}

static void log_cycle(const Cfg& c, int cycle, double cycle_s, const CycleStats& cs, int temp) {
    const std::string p = c.path(c.cycle_log);
    bool is_new = !file_exists(p);
    if (FILE* f = std::fopen(p.c_str(), "a")) {
        if (is_new) std::fputs("datetime,cycle,cycle_s,paused_s,temp_c,acc_deg_rated,acc_deg_peak,peak_rpm\n", f);
        std::fprintf(f, "%s,%d,%.2f,%.0f,%d,%.1f,%.1f,%.1f\n", now_str().c_str(), cycle, cycle_s,
                     cs.paused_s, temp, cs.acc_deg_rated, cs.acc_deg_peak, cs.peak_rpm);
        std::fclose(f);
    }
}

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
    if (argc < 3) {
        std::puts("Usage: endurance_test <ifname> <endurance.yaml>\n"
                  "  e.g., sudo ./endurance_test enp3s0 ../src/endurance.yaml");
        return 1;
    }
    std::signal(SIGINT, on_sigint);
    const Cfg cfg = load_cfg(argv[2]);
    State st = load_state(cfg);

    int code = 0;
    std::string reason = "완료";
    try {
        EthercatMaster master(argv[1]);
        if (master.slaveCount() != 1) {
            std::fprintf(stderr, "구동기 1개만 연결한 상태를 가정함 (발견: %d개)\n", master.slaveCount());
            return 2;
        }
        ActuatorPDOMap map{};
        EthercatActuator act(1, map);
        Rig r{master, act, {}, {}, cfg, 0, {}, 0};
        r.cmd.controlword = CW_FAULT_RESET;
        r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        r.cmd.target_tor = 0;

        const double nm_per_unit = cfg.motor_rated_nm * cfg.gear_ratio / 1000.0;
        std::printf("=== 내구 시험 ===\n"
                    "정격 구간 %d (출력 약 %.1f Nm), 최대 구간 %d (출력 약 %.1f Nm), torque_scale %.2f\n"
                    "일정 속도 %.0f rpm (출력축), 속도 한도 %.0f rpm\n"
                    "백래시 종료 기준 %.2f도 = %.0f 카운트, 총 %d 사이클, 이번 실행 최대 %.0f분\n"
                    "저장된 진행: %d 사이클 완료\n",
                    cfg.torque_rated, cfg.torque_rated * cfg.torque_scale * nm_per_unit,
                    cfg.torque_peak, cfg.torque_peak * cfg.torque_scale * nm_per_unit, cfg.torque_scale,
                    cfg.cruise_rpm_out, cfg.abort_rpm_out, cfg.limit_deg, cfg.limit_counts(),
                    cfg.total_cycles, cfg.max_run_minutes, st.cycles_done);

        // ---- 1. 켜기 (최대 10초)
        std::puts("구동기 켜는 중...");
        const auto on_deadline = clk::now() + std::chrono::seconds(10);
        while (!r.enabled()) {
            if (!r.tick()) break;
            if (clk::now() > on_deadline) { r.err = "구동기가 Operation Enabled 가 되지 않음"; break; }
        }

        // ---- 2. 사이클 반복. 지금 원판 자리가 이번 실행의 0도(origin)
        const int32_t origin = r.fb.pos;
        const auto run_start = clk::now();
        FILE* tf = nullptr;
        if (cfg.trace_cycles > 0 && (tf = std::fopen(cfg.path(cfg.trace_file).c_str(), "w"))) {
            std::fputs("t_s,tag,pos,target_pos,cmd_tor,tor,mode,rpm,temp\n", tf);
            r.trace_t0 = Rig::now_s();
        }
        int ran = 0;
        int cycle = st.cycles_done;
        double scales[4] = {cfg.decel_margin, cfg.decel_margin, cfg.decel_margin, cfg.decel_margin};
        while (r.err.empty()) {
            // (a) 백래시 측정
            if (measure_due(cfg, cycle) && st.last_measured != cycle) {
                std::printf("\n[백래시] %d 사이클 측정 시작\n", cycle);
                double waited = 0;
                if (cfg.cool_before_measure_c > 0 && r.fb.temp > cfg.cool_before_measure_c &&
                    !wait_until_cool(r, static_cast<int>(cfg.cool_before_measure_c), waited, "측정 전 냉각")) break;
                Meas m;
                if (!measure_backlash(r, origin, cycle, m)) break;
                st.last_measured = cycle; save_state(cfg, st);
                std::printf("[백래시] %d 사이클: %.0f 카운트 = 출력 %.4f도 (종료 기준 %.2f도), 최고 %d°C\n",
                            cycle, m.backlash_cnt, cfg.deg_from_counts(m.backlash_cnt), cfg.limit_deg, m.temp_max);
                if (m.backlash_cnt >= cfg.limit_counts()) { reason = "백래시가 종료 기준에 도달"; break; }
            }
            // (b) 종료 조건
            if (cycle >= cfg.total_cycles) { reason = "목표 사이클 완료"; break; }
            if (cfg.run_cycles_limit > 0 && ran >= cfg.run_cycles_limit) { reason = "이번 실행 사이클 한도"; break; }
            if (cfg.max_run_minutes > 0 &&
                clk::now() - run_start > std::chrono::duration<double>(cfg.max_run_minutes * 60)) { reason = "이번 실행 시간 한도"; break; }

            // (c) 한 사이클
            const auto c0 = clk::now();
            CycleStats cs; r.temp_max = 0;
            r.trace = (ran < cfg.trace_cycles) ? tf : nullptr;
            if (!run_cycle(r, origin, cs, scales)) break;
            r.trace = nullptr;
            ++cycle; ++ran;
            st.cycles_done = cycle; save_state(cfg, st);
            double cycle_s = std::chrono::duration<double>(clk::now() - c0).count();
            log_cycle(cfg, cycle, cycle_s, cs, r.temp_max);
            if (ran <= 3 || cycle % 10 == 0)
                std::printf("사이클 %d/%d  %.1f초  %d°C  가속구간 정격 %.1f도 / 최대 %.1f도  최고 %.0f rpm  감속 후 남은 각도 최대 %.1f도%s\n",
                            cycle, cfg.total_cycles, cycle_s, r.temp_max, cs.acc_deg_rated, cs.acc_deg_peak,
                            cs.peak_rpm, cs.left_deg, cs.paused_s > 0 ? "  (온도 대기 있었음)" : "");
        }
        r.trace = nullptr;
        if (tf) std::fclose(tf);
        if (!r.err.empty()) { reason = r.err; code = 3; }

        // ---- 3. 토크 0 으로 0.5초 유지 후 끄기
        r.cmd.mode = MODE_CYCLIC_SYNCHRONOUS_TORQUE;
        r.cmd.target_tor = 0;
        r.err.clear(); g_stop = 0;
        for (int k = 0; k < 1000; ++k) r.tick();
        r.cmd.controlword = CW_SHUTDOWN;
        act.writeCommand(r.cmd); master.tickOnce();
        std::printf("\n종료: %s\n이번 실행 %d 사이클, 누적 %d 사이클 (진행은 %s 에 저장됨)\n",
                    reason.c_str(), ran, st.cycles_done, cfg.path(cfg.state_file).c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal: %s\n", e.what());
        code = 1;
    }
    return code;
}
