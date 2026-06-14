# KnockLock: Rhythmic 2FA Home Security System
<img width="664" height="790" alt="image" src="https://github.com/user-attachments/assets/a94e9419-d501-4765-81a5-a26ee862901c" />
<img width="1484" height="1102" alt="image" src="https://github.com/user-attachments/assets/4ab391ea-9127-4ed4-8224-7c6e8806551f" />

A sophisticated two-factor authentication home security system that combines **rhythmic knock recognition** and **combination keypad entry** for enhanced access control.

## Overview

KnockLock is an embedded systems project that implements a dual-authentication mechanism for home security. The system recognizes specific knock patterns on the door and requires a secondary keypad code entry, providing both convenience and security through two independent authentication factors.

### Key Features

- 🔐 **Two-Factor Authentication**: Combines knock pattern recognition with keypad code entry
- 🎵 **Rhythmic Knock Detection**: Recognizes specific knock patterns and timing sequences
- ⌨️ **Digital Keypad**: Secondary authentication via numeric combination
- ⚡ **Embedded System**: Optimized for microcontroller implementation
- 🚪 **Smart Door Control**: Automated locking/unlocking mechanism

## Technology Stack

### Languages
- **C** (41.5%) - Core system logic and main application
- **Assembly** (37.2%) - Low-level hardware operations and performance-critical sections
- **Pawn** (17.1%) - Scripting layer for knock pattern logic
- **Visual Basic 6.0** (3.4%) - Configuration and testing utilities
- **HTML** (0.2%) - Documentation and web interface components
- **Other** (0.8%) - Build scripts and configuration files

## Project Structure

```
├── src/                  # Source code
│   ├── knock_detection/  # Knock pattern recognition algorithms
│   ├── keypad/          # Keypad input handling
│   ├── authentication/  # 2FA logic
│   └── hardware/        # Hardware interface and drivers
├── docs/                # Project documentation
├── tests/               # Test suite
└── config/              # Configuration files
```

## Getting Started

### Prerequisites
- Microcontroller development environment (e.g., Arduino IDE, Keil, or equivalent)
- C/Assembly compiler toolchain
- Serial communication capability for testing

### Building the Project

1. Clone the repository:
   ```bash
   git clone https://github.com/ivan-yung/EEC136AB-spring2026-India-KnockLock.git
   cd EEC136AB-spring2026-India-KnockLock
   ```

2. Configure your hardware platform in the build settings

3. Compile and upload to your microcontroller:
   ```bash
   make build
   make upload
   ```

## How It Works

### Authentication Flow

1. **Knock Detection Phase**
   - System monitors door sensor for vibration/sound patterns
   - Analyzes timing and rhythm of knocks
   - Verifies pattern matches registered authentication knock sequence

2. **Keypad Entry Phase**
   - Upon successful knock recognition, keypad is activated
   - User enters their numeric code
   - System validates code against stored credentials

3. **Access Grant**
   - Successful authentication activates door lock mechanism
   - System logs access event with timestamp
   - Ready for next authentication cycle

### Security Considerations

- Knock patterns are stored with timing variance tolerance
- Keypad codes support variable-length combinations
- Failed authentication attempts can trigger alerts
- System includes anti-tamper detection

## Hardware Requirements

- Microcontroller (e.g., ARM Cortex-M, PIC, AVR)
- Door sensor/vibration detector
- Digital keypad matrix
- Door lock actuator
- Power management circuit
- Optional: wireless communication module for logging

## Configuration

Customize authentication parameters by editing the configuration files:
- Knock pattern: Timing sequences and acceptable variance
- Keypad code: Length and numeric combination
- Security settings: Attempt limits, lockout duration

## Testing

Run the test suite to verify functionality:
```bash
make test
```

Tests include:
- Knock pattern recognition accuracy
- Keypad input handling
- Authentication state machine
- Hardware interface verification

## Project Context

This project was developed as part of **EEC136AB** (Spring 2026) at UC Davis by **India Team**, featuring embedded systems design principles and practical IoT security implementation.

## Contributing

This is a course project. For modifications or improvements, please create an issue or pull request with clear documentation of changes.

## License

[Specify your license here - e.g., MIT, Apache 2.0, etc.]

## Contact

For questions or technical details, please refer to the course documentation or contact the development team.

---

**Status**: Active Development | **Last Updated**: Spring 2026
