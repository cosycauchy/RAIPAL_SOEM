// sim/ethercat_actuator.hpp — 가짜 구동기 (실제 헤더와 같은 인터페이스)
#pragma once
#include <cstddef>
#include "sim_hw.hpp"
struct ActuatorPDOMap {};
class EthercatActuator {
public:
    EthercatActuator(uint16_t, const ActuatorPDOMap&) {}
    void readFeedback(ActuatorFeedback& fb) const { fb = hw().fb; }
    void writeCommand(const ActuatorCommand& c) { hw().cmd = c; }
    void advanceCiA402(const ActuatorFeedback& fb, ActuatorCommand& cmd) {
        switch (fb.status) {
            case 0x21: cmd.controlword = CW_SWITCH_ON; break;
            case 0x23: cmd.controlword = CW_ENABLE_OPERATION; break;
            case 0x27: cmd.controlword = CW_ENABLE_OPERATION; break;
            case 0x40: cmd.controlword = CW_SHUTDOWN; break;
            case 0x08: cmd.controlword = CW_FAULT_RESET; break;
            default: break;
        }
    }
};
