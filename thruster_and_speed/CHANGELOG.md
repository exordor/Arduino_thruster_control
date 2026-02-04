# Changelog

All notable changes to the thruster and speed control project will be documented in this file.

## [Unreleased]

### Added
- **Hold Test Mode** - New test mode in `udp_test.py` for testing continuous rotation
  - Holds a single command for specified duration (default 20s)
  - Sends commands repeatedly at 10Hz (every 100ms) to maintain target
  - Monitors and reports delta, stability, and statistics
  - Usage: `python3 udp_test.py --mode hold --left 1800 --right 1800 --duration 20`

### Changed
- **Direct WiFi Control** - Optimized for high-speed thruster operation
  - WiFi filter alpha: 80% → **100%** (no filtering, direct control)
  - WiFi max step: 50µs → **500µs** (aggressive ramping for high speeds)
  - **Result**: Command 1800 now reaches exactly 1800µs (0µs delta, was 500µs off)
- **Updated Documentation** - WiFi filter spec now reflects actual code behavior
  - README updated: "80% alpha (fast)" → "100% alpha (direct control)"

### Performance Comparison

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| WiFi Filter Alpha | 80% | **100%** | **Direct control** |
| WiFi Max Step | 50µs/cycle | **500µs/cycle** | **10x faster ramping** |
| WiFi Response (1800 cmd) | ~1300µs (500µs delta) | **1800µs (0µs delta)** | **Exact target** |
| WiFi Response (1600 cmd) | ~1550µs (50µs delta) | **1600µs (0µs delta)** | **Exact target** |

### Technical Details

#### Direct Control for High-Speed Operation

**Why Change?**
- Previous 80% filter with 50µs max step caused asymptotic approach
- High-speed commands (1800µs) only reached ~1300µs - far from target
- Continuous rotation testing showed thrusters couldn't reach full speed

**New Constants:**
```cpp
// WiFi filtering (direct control for high-speed operation)
const int WIFI_FILTER_ALPHA = 100;     // 100% = no filtering, direct control
const int WIFI_MAX_STEP_US = 500;      // 500µs/cycle = aggressive ramping
```

#### Test Results (Hold Test at 1800µs)

```
Before (80% alpha, 50µs step):
  Command 1800 → ~1300µs (500µs delta, thrusters don't reach full speed)

After (100% alpha, 500µs step):
  Command 1800 → 1800µs (0µs delta, exact target achieved)

Test: 20s hold at 1800µs
Commands sent: 185 (every 100ms)
Average delta: 0.0µs
Max delta: 0µs
Result: ✓ Stable - Reached and maintained target
```

#### New Test Mode Usage

```bash
# Hold test (20 seconds at 1800µs)
python3 udp_test.py --mode hold --left 1800 --right 1800 --duration 20

# Custom values
python3 udp_test.py --mode hold --left 1600 --right 1600 --duration 10

# Test forward low speed
python3 udp_test.py --mode hold --left 1550 --right 1550 --duration 15
```

### Migration Notes

If you have existing code:
1. **WiFi Control**: Now responds instantly to commands (no filtering delay)
2. **High Speeds**: Commands 1800/1900 will now reach target exactly
3. **Direct Control**: No gradual ramping - immediate change to target value
4. **Recommended**: Send commands at 10Hz (100ms intervals) for robust control

## [Unreleased - Previous]

### Added
- **10Hz Latency Test Mode** - New test mode in `udp_test.py` for measuring control latency
  - Sends alternating commands at 10Hz (1500→1600→1500→1400→1500)
  - Measures response time, delta from expected values
  - Provides statistics: min/max/avg/median latency with standard deviation
- **Separated RC/WiFi Filtering** - Independent filtering parameters for different input sources
  - RC: 25% filter alpha, 15µs max step (smooth, resists joystick drift)
  - WiFi: 80% filter alpha, 50µs max step (low latency, fast response)

### Changed
- **Low-Latency WiFi Control** - Optimized for minimal control delay
  - Command rate limit: 100ms → **20ms** (max 50 commands/sec vs 10 commands/sec)
  - WiFi filter alpha: 20% → **80%** (4x faster response)
  - WiFi max step: 10µs → **50µs** (5x faster ramping)
  - **Result**: Forward command 1600 now reaches ~1550µs (was ~1510µs)
- **RC Deadband Increased** - Better resistance to joystick drift
  - Deadband: 25µs → **40µs** (±40µs around center treated as neutral)
  - Helps prevent unintended thruster activation from joystick drift
- **Python Test Script** - Enhanced with new test modes
  - Added `--mode hz10` for 10Hz latency testing
  - Added `--mode thruster` for thruster control command tests
  - All modes show detailed statistics and timing information

### Performance Comparison

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Command Rate Limit | 100ms | **20ms** | **5x faster** |
| WiFi Response (1600 cmd) | 1510µs | **1550µs** | **4x larger change** |
| WiFi Response (1400 cmd) | 1498µs | **1460µs** | **20x larger change** |
| RC Filter Strength | 80% | **25%** | **Smoother, drift-resistant** |
| RC Deadband | ±25µs | **±40µs** | **60% larger** |

### Technical Details

#### Separated Filtering Parameters

**Why Separate?**
- RC inputs have physical joystick drift and hand tremors
- WiFi inputs are precise digital values
- Different input characteristics require different filtering

**New Constants:**
```cpp
// RC filtering (smooth, drift-resistant)
const int RC_FILTER_ALPHA = 25;       // 25% new value = strong smoothing
const int RC_MAX_STEP_US = 15;        // 15µs/cycle = slow ramping

// WiFi filtering (low-latency, responsive)
const int WIFI_FILTER_ALPHA = 80;     // 80% new value = fast response
const int WIFI_MAX_STEP_US = 50;      // 50µs/cycle = fast ramping
const unsigned long MIN_CMD_INTERVAL_MS = 20;  // 20ms min interval

// Deadband for RC joystick drift
const int DEADBAND_US = 40;  // ±40µs around center
```

#### Test Results (10Hz Latency Test)

```
Before (high filtering):
  Forward 1600 → 1510µs (only 10% change, thrusters don't spin)
  Backward 1400 → 1498µs (only 0.2% change, thrusters don't spin)

After (low-latency WiFi):
  Forward 1600 → 1550µs (50% change, thrusters respond)
  Backward 1400 → 1460µs (40% change, thrusters respond)

Latency: Avg 71ms (Min 39ms, Max 109ms)
```

#### New Test Mode Usage

```bash
# 10Hz latency test (10 seconds)
python3 udp_test.py --mode hz10 --duration 10

# Thruster control test
python3 udp_test.py --mode thruster

# Monitor with keep-alive
python3 udp_test.py --mode monitor --keep-alive
```

### Migration Notes

If you have existing code:

1. **RC Control**: Should be smoother and more resistant to drift
2. **WiFi Control**: Will respond faster and more aggressively
3. **Command Rate**: If sending faster than 10Hz, consider the new 20ms rate limit
4. **Deadband**: RC joystiles have larger neutral zone (±40µs vs ±25µs)

## [1.0.0] - Initial Release

### Added
- **Dual-Port UDP Architecture** - Separated data and heartbeat communication onto different ports
  - Port 8888: Data (commands, status, flow, PING/PONG)
  - Port 8889: Heartbeat (HEARTBEAT messages only)
- **PING/PONG Protocol** - Handshake and keep-alive mechanism that doesn't affect thruster commands
  - PING is NOT subject to command rate limiting
  - PONG response confirms Arduino received the PING
- **Keep-Alive Thread** - Background thread in Python client that sends PING every 1s
- **Enhanced Python Test Script** - Comprehensive UDP testing utility
  - Monitor mode: Listen for all messages
  - Heartbeat test mode: Test heartbeat reception for specified duration
  - Interactive mode: Send commands and see responses
  - Keep-alive support: `--keep-alive` flag

### Changed
- **Arduino Protocol Documentation** - Updated comments to reflect dual-port architecture
- **WiFi Reconnection** - Now restarts both UDP servers (data and heartbeat) on reconnect
- **Command Rate Limiting** - Fixed bug where `continue` statement skipped PING handler
  - Changed from skipping entire loop to only skipping C command processing
  - PING commands now work regardless of rate limit state

### Fixed
- **Rate Limiting Bug** - PING handler was being skipped by `continue` statement in C command processing
  - Old behavior: Rate-limited C commands used `continue`, skipping all subsequent handlers
  - New behavior: Rate limit check only affects C command processing, PING handler always executes

### Technical Details

#### Dual-Port Architecture Benefits
1. **Protocol Clarity** - Heartbeat traffic separated from data
2. **No Interference** - PING doesn't affect command rate limiting
3. **Easy Filtering** - Can filter heartbeat traffic separately
4. **Independent Channels** - Different QoS can be applied per port

#### Protocol Changes

**Before (Single Port):**
```
Port 8888 (bidirectional):
- C <left_us> <right_us>  (commands)
- S <mode> <left_us> <right_us>  (status)
- F <freq> <flow> <velocity> <total>  (flow data)
- HEARTBEAT  (heartbeat)
```

**After (Dual Port):**
```
Port 8888 (bidirectional data):
- C <left_us> <right_us>  (commands, rate limited)
- PING  (handshake/keep-alive, NO rate limit)
- PONG  (response to PING)
- S <mode> <left_us> <right_us>  (status)
- F <freq> <flow> <velocity> <total>  (flow data)

Port 8889 (Arduino → Client heartbeat):
- HEARTBEAT  (every 500ms)
```

### Testing

Run the test script to verify dual-port functionality:

```bash
# Basic monitor test
python3 udp_test.py --mode monitor

# With keep-alive (recommended)
python3 udp_test.py --mode monitor --keep-alive

# Heartbeat stress test
python3 udp_test.py --mode heartbeat --duration 30
```

Expected output:
- `[PONG]` - PING response on port 8888
- `[HEARTBEAT]` - Heartbeat on port 8889
- `[STATUS]` - Status updates on port 8888
- `[FLOW]` - Flow data on port 8888
