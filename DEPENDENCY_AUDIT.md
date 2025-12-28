# Rocket-Chip Dependency Audit Report
**Date:** 2025-12-28
**Auditor:** Claude Code Analysis
**Repository:** Rocket-Chip

---

## Executive Summary

This audit analyzes the Rocket-Chip project's dependencies for outdated packages, security vulnerabilities, and unnecessary bloat. The project is an Arduino-based rocket telemetry system targeting the Adafruit RP2350 HSTX Feather board.

**Key Findings:**
- ✅ Active development code (Dev/Claude) has a clean, minimal dependency footprint
- ⚠️ DEPRECATED folder contains 136KB of legacy code with redundant dependencies
- ⚠️ Several unused libraries present in DEPRECATED code only
- ✅ No critical security vulnerabilities identified in core dependencies
- 💡 Opportunities for cleanup and optimization identified

---

## Current Dependencies

### Active Dependencies (Dev/Claude/)

These are the libraries actively used in the current development code:

| Library | Purpose | Status | Notes |
|---------|---------|--------|-------|
| **Adafruit_ICM20948** | 9-axis IMU sensor | ✅ Required | Primary IMU for accelerometer, gyroscope, magnetometer |
| **Adafruit_ICM20X** | ICM20X sensor family | ✅ Required | Base library for ICM20948 |
| **Adafruit_DPS310** | Barometric pressure | ✅ Required | Primary altitude/pressure sensor |
| **Adafruit_Sensor** | Unified sensor API | ✅ Required | Base library for all Adafruit sensors |
| **Adafruit_AHRS** | Sensor fusion | ✅ Required | Madgwick filter for attitude estimation |
| **Adafruit_NeoPixel** | LED control | ✅ Required | Status indicator control |
| **LittleFS** | File system | ✅ Required | Internal flash storage for data logging |
| **MAVLink** | Telemetry protocol | ✅ Required | Industry-standard telemetry format |
| **Wire** | I2C communication | ✅ Required | Core I2C library |
| **SPI** | SPI communication | ✅ Required | Core SPI library |
| **Arduino.h** | Arduino core | ✅ Required | Base Arduino framework |

**Total Active Dependencies:** 11 libraries (all necessary)

### Legacy Dependencies (DEPRECATED/ only)

These libraries are only referenced in deprecated code:

| Library | Status | Recommendation |
|---------|--------|----------------|
| **Adafruit_BMP280** | ⚠️ Unused in active code | Remove or document as alternative |
| **Adafruit_BMP085_U** | ⚠️ Unused in active code | Remove (obsolete sensor) |
| **Adafruit_LSM303_U** | ⚠️ Unused in active code | Remove (replaced by ICM20948) |
| **Adafruit_Simple_AHRS** | ⚠️ Unused in active code | Remove (using Adafruit_AHRS instead) |
| **SD** | ⚠️ Unused in active code | Remove (using LittleFS instead) |

---

## Security Vulnerability Assessment

### Critical Findings: None ✅

### Library Security Status:

1. **Adafruit Libraries**
   - **Risk Level:** Low
   - **Vendor:** Adafruit (reputable, actively maintained)
   - **Update Recommendation:** Keep current via Arduino Library Manager
   - **Notes:** Adafruit libraries are well-maintained and regularly updated

2. **MAVLink**
   - **Risk Level:** Low
   - **Version:** MAVLink 2.0 (noted in code comments)
   - **Notes:** Industry-standard protocol, widely used in aerospace
   - **Recommendation:** Verify using latest stable MAVLink 2.0 implementation

3. **LittleFS**
   - **Risk Level:** Low
   - **Notes:** Part of RP2040/RP2350 core, maintained by Raspberry Pi Foundation

### General Security Recommendations:

1. **Keep Libraries Updated:** Use Arduino Library Manager to check for updates monthly
2. **Pin Critical Versions:** Consider documenting specific library versions that are tested and known to work
3. **Validate Sensor Data:** Continue implementing bounds checking on sensor readings (already present in code)
4. **Input Validation:** Serial command handler has good input validation (present in code)

---

## Bloat and Redundancy Analysis

### Repository Size Breakdown

```
Total Repository:    717 KB
├── Dev/            252 KB (35%)  ← Active development
├── DEPRECATED/     136 KB (19%)  ← Legacy code
├── docs/            26 KB (4%)
└── Other files     303 KB (42%)
```

### Identified Bloat

#### 1. DEPRECATED Folder (136 KB, 10 files)
**Impact:** Medium
**Priority:** High

The DEPRECATED folder contains legacy implementations that are no longer maintained:
- `Datalog/`, `Datalogger/`, `OLD-Datalogger/` - Multiple outdated logging implementations
- `ahrs_10dof-RocketChip/` - Old 10-DoF sensor code
- `bmp280test/` - Sensor testing code
- `PSRAMTest/` - PSRAM testing code
- `Gemeni/`, `Grok/` - Old AI-assisted implementations
- `ORIGINAL_RH_RF69.h` (52 KB) - Large legacy radio header file

**Recommendation Options:**
1. **Archive to Git Branch:** Move DEPRECATED to a `legacy` branch to preserve history
2. **Delete Entirely:** Remove if no longer needed (history preserved in Git)
3. **Keep Documentation Only:** Extract any useful documentation/comments before removal

**Estimated Space Savings:** 136 KB (19% reduction)

#### 2. Redundant Library References

**Files with multiple includes for similar purposes:**
- Multiple pressure sensor libraries (BMP280, BMP085, DPS310) when only DPS310 is used
- Both `Adafruit_AHRS` and `Adafruit_Simple_AHRS` referenced (only using Adafruit_AHRS)

**Recommendation:** Clean up unused includes in DEPRECATED code or remove entirely

#### 3. Duplicate Code Patterns

**Observation:** Multiple implementations of similar functionality across DEPRECATED folders
- Three separate datalogger implementations
- Multiple sensor initialization patterns

**Recommendation:** If DEPRECATED code is retained, consolidate to single reference implementation

---

## Outdated Package Analysis

### Version Check Recommendations

Since this is an Arduino project, library versions are typically managed through the Arduino Library Manager. To check for updates:

```bash
# If using Arduino CLI
arduino-cli lib list
arduino-cli lib upgrade

# Or use Arduino IDE: Tools > Manage Libraries
```

### Specific Library Update Checks:

| Library | Current Status | Action Required |
|---------|---------------|-----------------|
| Adafruit_ICM20948 | Unknown | Check Arduino Library Manager |
| Adafruit_DPS310 | Unknown | Check Arduino Library Manager |
| Adafruit_Sensor | Unknown | Check Arduino Library Manager |
| Adafruit_AHRS | Unknown | Check Arduino Library Manager |
| Adafruit_NeoPixel | Unknown | Check Arduino Library Manager |
| MAVLink | 2.0 (from comments) | Verify latest 2.0.x version |

**Note:** Arduino libraries don't typically include version info in source files. Use Arduino Library Manager to check for updates.

### Recommended Update Process:

1. Before updating libraries, create a git tag: `git tag -a pre-lib-update-YYYY-MM-DD -m "Before library updates"`
2. Update libraries via Arduino Library Manager
3. Test compilation for all active sketches
4. Test on hardware if possible
5. Document any API changes required

---

## Configuration Analysis

### Hardware Info YAML (5.5 KB)

**Status:** ✅ Well-structured
**Purpose:** Documents hardware configuration, I2C addresses, pin mappings
**Recommendation:** Keep as-is, very useful reference

### Config.h Analysis

The `Dev/Claude/Claude_TX_RP2350/config.h` file is well-organized with:
- Clear sensor configuration
- Adjustable sampling rates
- Calibration parameters
- Flight detection thresholds

**Recommendations:**
- ✅ Good separation of concerns
- ✅ Clear documentation
- 💡 Consider adding library version requirements as comments

---

## Recommendations Summary

### Priority 1 - High Impact, Low Risk

1. **Archive or Remove DEPRECATED Folder**
   - **Benefit:** 19% repository size reduction, clearer project structure
   - **Risk:** Low (all history in Git)
   - **Action:**
     ```bash
     # Option A: Move to branch
     git checkout -b archive/deprecated
     git add DEPRECATED/
     git commit -m "Archive deprecated code"
     git checkout main
     git rm -r DEPRECATED/
     git commit -m "Remove deprecated code (archived in archive/deprecated branch)"

     # Option B: Delete (if confident)
     git rm -r DEPRECATED/
     git commit -m "Remove deprecated code (history preserved in Git)"
     ```

2. **Document Library Versions**
   - **Benefit:** Reproducible builds, easier troubleshooting
   - **Action:** Create `DEPENDENCIES.md` file listing tested library versions
   - **Format:**
     ```markdown
     # Tested Library Versions
     - Adafruit ICM20948: v1.x.x
     - Adafruit DPS310: v1.x.x
     (etc.)
     ```

### Priority 2 - Medium Impact

3. **Clean Up Unused Includes in Active Code**
   - Review includes in Dev/Claude/ files
   - Remove any unused headers
   - Benefit: Faster compilation, clearer dependencies

4. **Add PlatformIO Configuration (Optional)**
   - Create `platformio.ini` for better dependency management
   - Benefit: Version pinning, reproducible builds, CI/CD support
   - Example:
     ```ini
     [env:adafruit_feather_rp2350]
     platform = raspberrypi
     board = adafruit_feather_rp2350
     framework = arduino
     lib_deps =
         adafruit/Adafruit ICM20948@^1.0.0
         adafruit/Adafruit DPS310@^1.0.0
         # etc.
     ```

### Priority 3 - Low Impact, Future Improvements

5. **Security Hardening**
   - Implement bounds checking on all sensor inputs (mostly done)
   - Add checksum validation for MAVLink messages
   - Consider signed firmware updates for production

6. **Monitoring Setup**
   - Set up monthly library update checks
   - Subscribe to Adafruit library release notifications
   - Monitor MAVLink project for security advisories

---

## Unnecessary Dependencies

### Confirmed Unnecessary (in active code):

- ❌ **Adafruit_BMP280** - Alternative pressure sensor not used
- ❌ **Adafruit_BMP085_U** - Obsolete pressure sensor library
- ❌ **Adafruit_LSM303_U** - Alternative IMU not used (using ICM20948)
- ❌ **Adafruit_Simple_AHRS** - Replaced by Adafruit_AHRS
- ❌ **SD library** - Not used (using LittleFS)

### Action Required:

These libraries are only referenced in DEPRECATED code. If DEPRECATED folder is removed, these become obsolete.

---

## Testing Recommendations

Before making any changes:

1. **Baseline Test**
   - Compile current code and note binary size
   - Test on hardware if available
   - Document current functionality

2. **After Cleanup**
   - Verify all active sketches compile
   - Check binary size (should be unchanged or smaller)
   - Test on hardware

3. **After Library Updates**
   - Full regression test
   - Verify sensor calibration still works
   - Check data logging functionality
   - Test MAVLink telemetry output

---

## Implementation Plan

### Phase 1: Immediate Actions (Low Risk)
1. Create backup branch with current state
2. Archive DEPRECATED folder to separate branch
3. Remove DEPRECATED folder from main branch
4. Update README to note where to find archived code

### Phase 2: Documentation (No Risk)
1. Create DEPENDENCIES.md with current library versions
2. Add library version requirements to config.h as comments
3. Document tested hardware configurations

### Phase 3: Library Maintenance (Medium Risk - Test Thoroughly)
1. Check for library updates via Arduino Library Manager
2. Update libraries one at a time
3. Test after each update
4. Document any required code changes

### Phase 4: Optional Improvements
1. Consider PlatformIO for better dependency management
2. Set up automated build testing
3. Implement additional security hardening

---

## Conclusion

The Rocket-Chip project has a **healthy dependency structure** with minimal bloat in active development code. The main opportunity for improvement is removing or archiving the DEPRECATED folder, which would:
- Reduce repository size by 19%
- Clarify which code is actively maintained
- Simplify dependency management
- Reduce confusion for new contributors

The active dependencies are all appropriate for the project's requirements, and no critical security vulnerabilities were identified. Regular library updates through Arduino Library Manager are recommended to stay current with bug fixes and improvements.

**Overall Assessment:** ✅ Good dependency hygiene, with clear improvement path available

---

## Appendix: Commands for Verification

```bash
# Check for unused includes in active code
grep -rh "#include" Dev/Claude/ --include="*.ino" --include="*.h" | sort | uniq

# Check library usage
grep -r "Adafruit_BMP" Dev/Claude/

# Check repository size
du -sh DEPRECATED/ Dev/ docs/

# Count files
find DEPRECATED/ -type f | wc -l
find Dev/ -type f | wc -l
```

---

**Report Generated:** 2025-12-28
**Next Review Recommended:** 2026-03-28 (Quarterly)
