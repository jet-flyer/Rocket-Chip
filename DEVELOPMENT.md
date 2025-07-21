# Development Workflow

## Version Management Strategy

### Current Structure Analysis
The project currently has multiple versioning approaches:
- **Date-based**: `2025/`, `6-5/`, `6-7/`, etc.
- **AI-assisted**: `Claude/`, `Grok/`, `Gemeni_RX_3-24-25/`
- **Feature-based**: `Modular/`, `PSRAMTest/`, etc.

### Recommended Consolidation Plan

#### Phase 1: Organize by Functionality
1. **Deprecated Versions** (`v1.0-deprecated/`)
   - `ahrs_10dof-RocketChip/`
   - `bmp280test/`
   - `Datalog/`
   - `Datalogger/`
   - `OLD-Datalogger/`

2. **AI-Assisted Versions** (organized by AI model)
   - `claude/` - Claude AI-assisted implementations
   - `grok/` - Grok AI-assisted implementations
   - `gemini/` - Gemini AI-assisted implementations

3. **Modular Versions** (`v2.0-modular/`)
   - `RocketChip/` - Basic modular implementation
   - `RocketChip_AP/` - ArduPilot integration
   - `RocketChip_Multi-board/` - Multi-board support

4. **Current Development** (`v3.0-current/`)
   - Latest stable implementation
   - Consolidated best features from AI-assisted versions

#### Phase 2: Branch Strategy
```
main                    # Stable releases
├── develop            # Integration branch
├── feature/psram      # PSRAM development
├── feature/modular    # Modular architecture
├── feature/ardupilot  # ArduPilot integration
└── experimental       # Experimental features
```

### Migration Steps

1. **Create new structure**
   ```bash
   mkdir -p versions/{v1.0-deprecated,v2.0-modular,v3.0-current}
   mkdir -p versions/ai-assisted/{claude,grok,gemini}
   mkdir -p development/{experimental,consolidated}
   mkdir -p docs/{hardware,software,tutorials}
   ```

2. **Move existing files**
   ```bash
   # Deprecated versions
   mv ahrs_10dof-RocketChip/ versions/v1.0-deprecated/
   mv bmp280test/ versions/v1.0-deprecated/
   mv Datalog/ versions/v1.0-deprecated/
   mv Datalogger/ versions/v1.0-deprecated/
   mv OLD-Datalogger/ versions/v1.0-deprecated/
   
   # AI-assisted versions (organized by model)
   mv 2025/Claude/ versions/ai-assisted/claude/
   mv 2025/Grok/ versions/ai-assisted/grok/
   mv 2025/Gemeni_RX_3-24-25/ versions/ai-assisted/gemini/
   
   # Modular versions
   mv 2025/Claude/Opus4/Modular/RocketChip/ versions/v2.0-modular/
   mv 2025/Claude/Opus4/Modular/RocketChip_AP/ versions/v2.0-modular/
   mv 2025/Claude/Opus4/Modular/RocketChip_Multi-board/ versions/v2.0-modular/
   
   # PSRAM and other features integrated into main versions
   mv PSRAMTest/ versions/v2.0-modular/psram-test/
   ```

3. **Update documentation**
   - Update README.md with new structure
   - Create version-specific documentation
   - Document migration process

### Git Branch Strategy

#### Main Branches
- `main`: Production-ready code
- `develop`: Integration branch for features

#### Feature Branches
- `feature/modular`: Modular architecture
- `feature/ardupilot`: ArduPilot integration
- `feature/sensors`: New sensor support
- `feature/psram`: PSRAM optimizations

#### Development Branches
- `dev/claude`: Claude AI-assisted development
- `dev/grok`: Grok AI-assisted development
- `dev/gemini`: Gemini AI-assisted development

### Version Naming Convention

#### Semantic Versioning
- `v1.0.0`: Major.Minor.Patch
- `v1.1.0`: New features, backward compatible
- `v2.0.0`: Breaking changes

#### Development Versions
- `v1.0.0-alpha.1`: Alpha releases
- `v1.0.0-beta.1`: Beta releases
- `v1.0.0-rc.1`: Release candidates

### Consolidation Guidelines

1. **Keep the best implementation** from each AI-assisted version
2. **Document differences** between versions
3. **Create migration guides** for users
4. **Maintain backward compatibility** where possible
5. **Tag releases** for easy reference

### Recommended Actions

1. **Before GitHub upload**:
   - Consolidate similar versions
   - Remove duplicate code
   - Standardize naming conventions
   - Create clear documentation

2. **After GitHub upload**:
   - Use branches for active development
   - Tag stable releases
   - Maintain clean commit history
   - Regular cleanup of old branches 