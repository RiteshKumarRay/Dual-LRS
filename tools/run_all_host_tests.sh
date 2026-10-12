#!/usr/bin/env bash
set -e

echo "==============================================================="
echo "Building and Running ALL Dual-LRS Host Test Suites"
echo "==============================================================="

# 1. Transport Protocol Tests
echo -e "\n[1/9] Running Transport Protocol Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_protocol.cpp -o /tmp/test_tp
/tmp/test_tp

# 2. Transport Engine Tests
echo -e "\n[2/9] Running Transport Engine Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_transport_engine.cpp src/transport_engine.cpp -o /tmp/test_te
/tmp/test_te

# 3. Production TDM Tests
echo -e "\n[3/9] Running Production TDM Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_STAGE31_RC_ONLY=1 -I include -I tools/mock_arduino tools/test_production_tdm.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_tdm
/tmp/test_tdm

# 4. Production RC Dispatch Tests
echo -e "\n[4/9] Running Production RC Dispatch Tests..."
g++ -std=c++17 -Wall -Wextra -I include -I tools/mock_arduino tools/test_production_rc_dispatch.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_rc_disp
/tmp/test_rc_disp

# 5. RC Adapter Tests
echo -e "\n[5/9] Running RC Adapter Tests..."
g++ -std=c++17 -Wall -Wextra -I include -I tools/mock_arduino tools/test_rc_adapter.cpp src/transport_engine.cpp -o /tmp/test_rca
/tmp/test_rca

# 6. Production MAVLink Transport Tests
echo -e "\n[6/9] Running Production MAVLink Transport Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_STAGE31_RC_ONLY=0 -I include -I tools/mock_arduino tools/test_production_mavlink_transport.cpp src/tdm_engine.cpp src/transport_engine.cpp -o /tmp/test_pmt
/tmp/test_pmt

# 7. MAVLink Handler Telemetry Gating Tests
echo -e "\n[7/9] Running MAVLink Handler Telemetry Gating Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_ROLE_AIR=1 -I include -I tools/mock_arduino tools/test_mavlink_handler_telemetry.cpp src/mavlink_handler.cpp -o /tmp/test_mavlink_handler_telemetry
/tmp/test_mavlink_handler_telemetry

# 8. Reliable Link Transport Tests (Phase 2 & 3)
echo -e "\n[8/9] Running Phase 2 & 3 Reliable Link Transport Tests..."
g++ -std=c++17 -Wall -Wextra -I include tools/test_link_transport.cpp src/link_transport.cpp src/transport_engine.cpp -o /tmp/test_lt
/tmp/test_lt

# 9. Wi-Fi Telemetry & Client Manager Tests
echo -e "\n[9/9] Running Wi-Fi Telemetry & Client Manager Tests..."
g++ -std=c++17 -Wall -Wextra -DDUAL_LRS_ROLE_AIR=1 -I include -I tools/mock_arduino tools/test_wifi_telemetry.cpp src/mavlink_handler.cpp -o /tmp/test_wifi
/tmp/test_wifi

echo -e "\n==============================================================="
echo "SUCCESS: ALL 9 HOST TEST SUITES PASSED CLEANLY (79/79 TESTS)!"
echo "==============================================================="
