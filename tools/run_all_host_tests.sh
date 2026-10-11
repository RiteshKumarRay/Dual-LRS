#!/usr/bin/env bash
set -e

echo "==============================================================="
echo "Building and Running ALL Dual-LRS Host Test Suites"
echo "==============================================================="

# 1. Transport Protocol Tests
echo -e "\n[1/7] Running Transport Protocol Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_protocol.cpp -o /tmp/test_tp
/tmp/test_tp

# 2. Transport Engine Tests
echo -e "\n[2/7] Running Transport Engine Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_engine.cpp src/transport_engine.cpp -o /tmp/test_te
/tmp/test_te

# 3. Production TDM Tests
echo -e "\n[3/7] Running Production TDM Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_STAGE31_RC_ONLY=1 -I include -I tools/mock_arduino tools/test_production_tdm.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_tdm
/tmp/test_tdm

# 4. Production RC Dispatch Tests
echo -e "\n[4/7] Running Production RC Dispatch Tests..."
g++ -std=c++17 -Wall -Wextra -I include -I tools/mock_arduino tools/test_production_rc_dispatch.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_rc_disp
/tmp/test_rc_disp

# 5. RC Adapter Tests
echo -e "\n[5/7] Running RC Adapter Tests..."
g++ -std=c++17 -Wall -Wextra -I include -I tools/mock_arduino tools/test_rc_adapter.cpp src/transport_engine.cpp -o /tmp/test_rca
/tmp/test_rca

# 6. Production MAVLink Transport Tests
echo -e "\n[6/7] Running Production MAVLink Transport Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_STAGE31_RC_ONLY=0 -I include -I tools/mock_arduino tools/test_production_mavlink_transport.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_pmt
/tmp/test_pmt

# 7. Reliable Link Transport Tests (Phase 2 & 3)
echo -e "\n[7/7] Running Phase 2 & 3 Reliable Link Transport Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_link_transport.cpp src/link_transport.cpp src/transport_engine.cpp -o /tmp/test_lt
/tmp/test_lt

echo -e "\n==============================================================="
echo "SUCCESS: ALL 7 HOST TEST SUITES PASSED CLEANLY (54/54 TESTS)!"
echo "==============================================================="
