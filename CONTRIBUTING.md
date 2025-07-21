# Contributing to Rocket Chip

Thank you for your interest in contributing to the Rocket Chip project! This document provides guidelines for contributing effectively.

## 🤝 How to Contribute

### Types of Contributions
- **Bug Reports**: Report issues you encounter
- **Feature Requests**: Suggest new features
- **Code Contributions**: Submit pull requests
- **Documentation**: Improve docs and examples
- **Testing**: Test on different hardware configurations

## 🚀 Getting Started

### Prerequisites
- Arduino IDE 1.8.x or 2.x
- Basic knowledge of Arduino/C++
- Hardware for testing (if applicable)

### Development Setup
1. Fork the repository
2. Clone your fork locally
3. Create a feature branch
4. Make your changes
5. Test thoroughly
6. Submit a pull request

## 📝 Code Style Guidelines

### Arduino/C++ Style
```cpp
// Use descriptive variable names
float launchAccelerationThreshold = 2.0f;

// Use camelCase for variables and functions
void readSensorData() {
    // Implementation
}

// Use UPPER_CASE for constants
#define MAX_BUFFER_SIZE 4096

// Add comments for complex logic
// Calculate total acceleration magnitude
float totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
```

### File Organization
- Keep functions under 50 lines when possible
- Group related functions together
- Use clear file and folder names
- Include header guards in .h files

## 🔧 Development Workflow

### Branch Naming
- `feature/description`: New features
- `bugfix/description`: Bug fixes
- `docs/description`: Documentation updates
- `test/description`: Testing improvements

### Commit Messages
Use conventional commit format:
```
type(scope): description

feat(sensors): add support for BMP388 pressure sensor
fix(psram): resolve memory allocation issue
docs(readme): update installation instructions
test(launch): add automated launch detection tests
```

### Pull Request Process
1. **Create descriptive PR title**
2. **Add detailed description** of changes
3. **Include testing information**
4. **Reference related issues**
5. **Request review** from maintainers

## 🧪 Testing Guidelines

### Hardware Testing
- Test on target hardware (RP2350 Feather)
- Verify sensor readings are accurate
- Test launch detection functionality
- Check data logging integrity

### Code Testing
- Verify compilation on different Arduino cores
- Test with different sensor configurations
- Validate memory usage and performance
- Check for memory leaks

### Documentation Testing
- Verify all code examples compile
- Test installation instructions
- Check link validity in documentation

## 📋 Issue Reporting

### Bug Reports
Include the following information:
```
**Hardware**: [Board and sensors used]
**Arduino Core**: [Version]
**Libraries**: [List of libraries and versions]
**Expected Behavior**: [What should happen]
**Actual Behavior**: [What actually happens]
**Steps to Reproduce**: [Detailed steps]
**Code Example**: [Minimal code to reproduce]
```

### Feature Requests
```
**Feature Description**: [What you want to add]
**Use Case**: [Why this feature is needed]
**Proposed Implementation**: [How it could work]
**Alternative Solutions**: [Other approaches considered]
```

## 🏗️ Project Structure

### Adding New Versions
1. Create version folder: `versions/vX.Y-description/`
2. Include main sketch and config files
3. Add documentation in `docs/`
4. Update version compatibility matrix
5. Test thoroughly before submission

### Adding New Sensors
1. Create sensor library wrapper
2. Add configuration options
3. Update calibration procedures
4. Include example code
5. Document wiring diagrams

## 📚 Documentation Standards

### Code Documentation
- Use clear, concise comments
- Document function parameters and return values
- Include usage examples
- Explain complex algorithms

### User Documentation
- Write for beginners and experts
- Include step-by-step instructions
- Provide troubleshooting guides
- Use clear, descriptive language

## 🔒 Security Considerations

### Sensitive Information
- Never commit API keys or passwords
- Use environment variables for secrets
- Document required configuration
- Provide example config files

### Code Security
- Validate all inputs
- Handle errors gracefully
- Use secure communication protocols
- Follow Arduino security best practices

## 🎯 Contribution Areas

### High Priority
- Bug fixes and stability improvements
- Performance optimizations
- Documentation improvements
- Testing coverage

### Medium Priority
- New sensor support
- Additional output formats
- Configuration improvements
- Example projects

### Low Priority
- Experimental features
- Alternative implementations
- Cosmetic improvements
- Additional AI integrations

## 📞 Getting Help

### Questions and Discussion
- Open an issue for questions
- Use GitHub Discussions for general topics
- Check existing documentation first
- Be specific about your problem

### Code Review
- Be open to feedback
- Respond to review comments promptly
- Make requested changes
- Ask for clarification when needed

## 🙏 Recognition

Contributors will be recognized in:
- Project README
- Release notes
- Documentation
- GitHub contributors page

---

Thank you for contributing to Rocket Chip! Your help makes this project better for everyone. 