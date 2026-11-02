// sim/ethercat_master.hpp — 가짜 마스터. tickOnce() 때마다 가짜 구동기를 실제 경과 시간만큼 진행시킨다.
#pragma once
#include <string>
#include "sim_hw.hpp"
class EthercatMaster {
public:
    explicit EthercatMaster(const std::string&) { (void)hw(); }
    int tickOnce() { hw().step(); return 3; }
    int expectedWKC() const { return 3; }
    int slaveCount() const { return 1; }
};
