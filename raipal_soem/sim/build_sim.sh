#!/bin/bash
# 가짜 구동기로 endurance_test 를 빌드한다 (SOEM, 하드웨어 필요 없음)
#   사용: ./build_sim.sh   →  ./endurance_sim <아무 이름> <yaml>
cd "$(dirname "$0")"
g++ -std=c++20 -O2 -Wall -Wextra -I. ../src/endurance_test.cpp -o endurance_sim -lyaml-cpp
