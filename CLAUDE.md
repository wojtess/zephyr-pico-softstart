# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **RP2040 (Raspberry Pi Pico)** embedded project built using the **Zephyr RTOS** framework. The project implements a PWM-based LED control system as a foundation for future softstart current control development.

### Project Roadmap

1. **Phase 1 (Current)**: LED ON/OFF control via binary protocol
2. **Phase 2 (Next)**: PWM duty cycle control
3. **Phase 3 (Future)**: Softstart with precision current regulation (1% accuracy)

### Academic Project Requirements

| Requirement | Status | Notes |
|-------------|--------|-------|
| 1. Modularyzacja kodu + Doxygen docs | ✅ | `src/main.c` documented, run `make docs` |
| 2. Git + śledzenie zmian | ✅ | Clean commit history, modular development |
| 3. Precyzyjna regulacja prądu (1%) | 🔄 | Phase 3 (after PWM implementation) |
| 4. Desktop GUI application | 🔄 | Dear PyGui in development (Krok 3/6) |
| 5. Skrypty symulacyjne + logowanie | ⏳ | Planned for end of project |
| 6. Protokół szeregowy z CRC | ✅ | `protocol.py` + firmware implementation |
| 7. RTOS (Zephyr) | ✅ | Fully integrated |

---

## Build System

This project uses a hybrid approach:
- **Makefile** - Primary build orchestration (run `make build`, `make flash`, etc.)
- **CMake** - Zephyr's native build system (invoked indirectly via `west`)
- **West** - Zephyr's meta-tool for SDK/workspace management

## Common Commands

```bash
# One-time setup (downloads Zephyr SDK and dependencies)
make deps

# Build the project
make build

# Flash to device (Pico must be in BOOTSEL mode - hold BOOTSEL button while plugging in USB)
make flash

# Clean build artifacts
make clean

# Generate Doxygen documentation
make docs
```

### Python Scripts (GUI & Testing)

```bash
# Install Python dependencies (uses project venv)
./venv/bin/pip install -r scripts/requirements.txt

# Run GUI application (Dear PyGui)
./venv/bin/python scripts/gui.py

# Run CLI test script
./venv/bin/python scripts/test_pwm.py
```

---

## Hardware/Board Target

- **Board**: `rpi_pico` (Raspberry Pi Pico - RP2040)
- **LED**: GPIO25 (built-in LED)
- **Communication**: USB CDC ACM (Virtual Serial Port)
- **Baud Rate**: 115200

---

## Architecture

### Binary Protocol (UART)
The application uses a 3-byte binary frame format:
- **Frame**: `[CMD][VALUE][CRC8]`
- **CMD 0x01**: SET LED (value 0=OFF, 1+=ON)
- **Response**: `0xFF` (ACK) or `0xFE` (NACK + error code)
- **CRC-8**: Polynomial 0x07, Initial 0x00

### Software Components

**Main Application** (`src/main.c`):
- **State Machine**: 3-state protocol parser (WAIT_CMD → WAIT_VALUE → WAIT_CRC)
- **Ring Buffer**: 32-byte ISR-safe UART receive buffer
- **Interrupt-Driven**: UART RX interrupt with main loop processing
- **GPIO Control**: Atomic LED state management via Zephyr GPIO API
- **Doxygen Docs**: Fully documented, run `make docs`

**Device Tree Overlay** (`boards/rpi_pico.overlay`):
- Configures USB CDC ACM UART device (cdc_acm_uart0)

**Configuration** (`prj.conf`):
- USB device stack configuration
- UART/console settings
- GPIO driver enable

### Python Scripts

**`scripts/protocol.py`** - Shared protocol module:
- Protocol constants (CMD_SET_LED, RESP_ACK, etc.)
- CRC-8 calculation function
- Frame building and parsing functions
- Used by both GUI and test scripts

**`scripts/gui.py`** - Dear PyGui desktop application (WIP - Krok 3/6):
- ✅ Serial port detection (Linux/macOS/Windows)
- ✅ Connect/Disconnect functionality
- ✅ Status message area with visual feedback
- ⏳ LED toggle button (Krok 4)
- ⏳ Protocol implementation in GUI (Krok 4)
- ⏳ Visual status indicators (Krok 6)

**`scripts/test_pwm.py`** - Host-side test utility:
- Auto-detects USB CDC ACM port
- Implements CRC-8 algorithm
- Sends LED control commands and verifies ACK/NACK responses

---

## Development Notes

### Project Structure
```
project1/
├── src/
│   └── main.c              # Firmware with Doxygen docs
├── scripts/
│   ├── protocol.py         # Shared protocol logic
│   ├── gui.py              # Dear PyGui application
│   ├── test_pwm.py         # CLI test script
│   └── requirements.txt    # Python dependencies
├── boards/
│   └── rpi_pico.overlay    # Device tree overlay
├── prj.conf                # Zephyr project config
├── Doxyfile                # Doxygen configuration
├── Makefile                # Build orchestration
├── CMakeLists.txt          # CMake for Zephyr
├── CLAUDE.md               # This file
└── README.md               # Basic build instructions
```

### Important Notes

- The project uses a Python virtual environment (`venv/`) for West and dependencies
- Build outputs are in `build/zephyr/` (key file: `zephyr.uf2` for flashing)
- Zephyr workspace is in `.zephyr/` (managed by West)
- **Doxygen docs**: Run `make docs` to generate HTML documentation in `docs/html/`
- When modifying protocol, update both `src/main.c` and `scripts/protocol.py` to keep CRC and frame format in sync
- Always wait for USB DTR signal in firmware before proceeding - this ensures host is ready

### Git Commit History Style

Commits use Polish language, simple past tense:
- `dodano gui.py i protocol.py`
- `dodano wykrywanie portow szeregowych i wybor dropdown`
- `poprawiono ux i obsluge bledow wykrywania portow`
- `dodano polaczenie i rozlaczenie serial`

### Development Workflow

1. Make incremental changes (one feature at a time)
2. Test thoroughly before committing
3. Write clear Polish commit messages
4. Update Doxygen comments when modifying firmware
5. Keep protocol.py in sync with firmware

---

## Future Work (Softstart Phase)

### Planned Features for Softstart Implementation:

1. **PWM Control**:
   - Add PWM peripheral configuration
   - Implement duty cycle commands (0-100%)
   - Add frequency control option

2. **Current Sensing**:
   - ADC integration for current measurement
   - Calibration routines
   - Overcurrent protection

3. **Softstart Algorithm**:
   - Gradual current ramp-up
   - PID control for 1% accuracy
   - Thermal monitoring

4. **Logging & Analysis**:
   - Signal logging (current, voltage, temperature)
   - Python analysis scripts
   - Real-time plotting in GUI
