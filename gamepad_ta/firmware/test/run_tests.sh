#!/bin/sh
cd "$(dirname "$0")" && g++ -std=c++11 -O2 -Wall -Wextra -o /tmp/test_adaptive test_adaptive.cpp && /tmp/test_adaptive
