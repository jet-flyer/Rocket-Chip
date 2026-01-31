# Agent Whiteboard

**Purpose:** Communication across context windows and between agents.

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes

**Last reviewed by Nathan**: *[DATE]*

---

## Open Flags

### 2026-01-31 - Claude Code CLI
**File**: Multiple files (see list below)
**Observation**: ISM330DHCX code and references remain in codebase after switching to ICM-20948 as primary IMU
**Why it seems odd**: ICM-20948 has native ArduPilot (Invensensev2) support; ISM330DHCX code may be dead weight
**Possible explanations**: ISM330DHCX kept as alternative/backup sensor option (FeatherWing #4569 is on-hand)
**Severity**: Medium
**Recommendation**: Review and decide: keep as option, or remove to reduce maintenance burden

**Files with ISM330DHCX code/references (require review):**
- `src/hal/IMU_ISM330DHCX.h` - Full driver header
- `src/hal/IMU_ISM330DHCX.cpp` - Full driver implementation
- `src/hal/IMU_ICM20948.h:16` - Includes ISM330DHCX for Vector3f types (coupling)
- `src/hal/Mag_LIS3MDL.h:19` - Include guard incorrectly references ISM330DHCX
- `tests/smoke_tests/st_sensors_test.cpp` - Test for ISM330DHCX
- `tests/smoke_tests/imu_qwiic_test.c` - Test for ISM330DHCX
- `tests/smoke_tests/hal_validation.cpp` - Probes ISM330DHCX address
- `lib/ap_compat/RocketChip/hwdef.h:61` - Comments about ISM330DHCX
- `lib/ap_compat/AP_HAL_RP2350/SPIDevice.cpp:59,69` - ISM330DHCX SPI mode comments
- `CMakeLists.txt:133,165,868` - ISM330DHCX build targets

**Note**: `lib/st_drivers/` submodule contains ST platform drivers (ISM330DHCX, LIS3MDL) - decision needed on whether to keep.

---

<!-- Template for new flags:

### [DATE] - [AGENT NAME]
**File**: `path/to/file`  
**Lines**: (if applicable)  
**Observation**: [What you noticed]  
**Why it seems odd**: [Your reasoning]  
**Possible explanations**: [What might justify this]  
**Severity**: Low / Medium / High  
**Recommendation**: [Ask Nathan / Defer / Needs discussion]

---

-->

---

## Resolved

*No resolved flags yet.*

<!-- Move resolved flags here with resolution notes:

### [DATE] - [AGENT] (Resolved [DATE])
**File**: `path/to/file`  
**Observation**: [Original observation]  
**Resolution**: [Decision and reasoning]

---

-->
