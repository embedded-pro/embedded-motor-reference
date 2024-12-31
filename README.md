# embedded-motor-reference

A comprehensive embedded motor control reference implementation demonstrating Field-Oriented Control (FOC) and PID control for DC, BLDC, and PMSM motors with detailed examples and real-world applications.

## Overview

This project serves as a practical reference for implementing motor control algorithms in embedded systems. It provides working examples of common motor control techniques and demonstrates best practices for different motor types and control strategies.

## Features

### Motor Support
- **DC Motors**: Basic PWM and current control
- **BLDC Motors**: Commutation and FOC implementation
- **PMSM Motors**: Complete FOC with position feedback
- **Generic Control**: PID implementations and tuning examples

### Control Algorithms
- Field-Oriented Control (FOC)
- Space Vector Modulation (SVM)
- PID control with anti-windup
- Current and velocity control loops
- Position control implementation

### Hardware Interfacing
- ADC sampling for current sensing
- Encoder and Hall sensor feedback
- PWM generation and dead-time control
- Temperature and fault monitoring
- Motor driver integration examples

## Getting Started

### Prerequisites
- Compatible development board (see [Supported Hardware](#supported-hardware))
- Development environment setup for embedded systems
- Basic understanding of motor control concepts

### Quick Start
1. Clone the repository
```bash
git clone https://github.com/yourusername/embedded-motor-reference.git
```

2. Set up your development environment following the instructions in [docs/setup.md](docs/setup.md)

3. Build the example project
```bash
cd embedded-motor-reference
make
```

4. Flash to your development board
```bash
make flash
```

## Project Structure
```
├── docs/               # Documentation
├── examples/           # Example applications
│   ├── dc/            # DC motor examples
│   ├── bldc/          # BLDC motor examples
│   ├── pmsm/          # PMSM motor examples
│   └── pid/           # PID tuning examples
├── src/               # Source files
├── include/           # Header files
└── tests/             # Test suites
```

## Documentation

Detailed documentation is available in the [docs](docs/) directory:
- [Motor Control Basics](docs/motor-control.md)
- [FOC Implementation](docs/foc.md)
- [PID Tuning Guide](docs/pid-tuning.md)
- [Hardware Setup](docs/hardware.md)

## Contributing

We welcome contributions! Please read our [Contributing Guidelines](CONTRIBUTING.md) before submitting pull requests.

### Development Setup
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.