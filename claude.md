# Smartknob-RS Development Guide

This document contains essential information for AI-assisted development on this codebase, with special focus on LVGL integration.

## Project Overview

This is a Rust firmware for a haptic smartknob device targeting ESP32-S3 with:
- Haptic feedback via BLDC motor (Field Oriented Control)
- LVGL-based UI on a round 240x240 GC9A01 display
- Embassy async runtime (bare-metal, no RTOS)
- Dual-core architecture (Core 0: UI/tasks, Core 1: FOC loop)

---

## Project Structure

```
smartknob-rs/
├── smartknob/              # Main firmware crate (ESP32-S3 specific)
│   ├── src/
│   │   ├── bin/
│   │   │   ├── main.rs              # Original hardware variant
│   │   │   ├── seedlabs_devkit.rs   # Seedlabs devkit variant
│   │   │   └── old_hw.rs            # Legacy hardware
│   │   ├── display.rs       # LVGL + display driver setup
│   │   ├── cli.rs           # USB serial command interface
│   │   ├── motor_control.rs # FOC task integration
│   │   ├── knob_tilt.rs     # LDC sensor for tilt/press detection
│   │   ├── signals.rs       # Global signal exports
│   │   └── shutdown.rs      # Power management
│   ├── lv_conf.h            # LVGL configuration (CRITICAL)
│   ├── partition_table.csv  # Flash partition layout
│   └── Cargo.toml
│
├── smartknob-core/          # Hardware-agnostic core (no ESP32 specifics)
│   └── src/
│       ├── haptic_core.rs   # Main haptic system coordinator
│       ├── haptic_core/
│       │   ├── encoder/     # Encoder abstractions
│       │   │   └── mt6701.rs  # MT6701 magnetic encoder
│       │   ├── motor_driver.rs
│       │   └── haptic_hardware.rs
│       ├── flash.rs         # Key-value flash storage abstraction
│       └── system_settings/ # Logging, calibration storage
│
├── smartknob-esp32/         # ESP32-specific implementations
│   └── src/
│       ├── flash.rs         # ESP flash driver
│       └── motor_driver/
│           └── mcpwm.rs     # MCPWM 6-PWM driver for TMC6300
│
├── haptic_lib/              # Pure haptic curve/pattern library (no_std)
│   └── src/
│       ├── curve.rs         # HapticCurve, CurveBuilder, AbsoluteCurve
│       ├── easings.rs       # Easing functions (Cubic, Sinusoidal, etc.)
│       ├── patterns.rs      # HapticPattern, Command sequences
│       └── player.rs        # HapticPlayer for playback
│
├── ldc1x1x/                 # TI LDC1614 inductance sensor driver
│   └── src/
│       ├── lib.rs           # Async I2C driver
│       └── data.rs          # Register definitions
│
├── haptic_demo/             # Desktop visualization tool
│   └── src/main.rs          # Generates HTML charts of haptic curves
│
├── Cargo.toml               # Workspace definition
├── set_env.sh               # ESP toolchain environment setup
└── README.md
```

---

## Build & Flash Instructions

### Initial Setup

```bash
# Install ESP toolchain manager
cargo install espup

# Install toolchains (do this with VSCode closed!)
espup install
```

### Building

```bash
# Source the ESP environment (REQUIRED before every build session)
source set_env.sh

# Build from the smartknob directory
cd smartknob

# Build for seedlabs devkit (primary target)
cargo build --release --bin seedlabs_devkit

# Build for original hardware
cargo build --release --bin smartknob-rs
```

### Flashing

```bash
cd smartknob
cargo run --release --bin seedlabs_devkit
```

### Using the Firmware CLI

1. Connect via USB serial (115200 baud)
2. Press Enter to activate the CLI (logs stop while CLI is active)
3. Type `help` for available commands
4. Type `exit` to return to logging mode

---

## Memory Architecture

### Heap Allocation
- **Internal SRAM:** 72KB heap (`esp_alloc::heap_allocator!(size: 72 * 1024)`)
- **PSRAM:** Used for framebuffer allocation (too large for SRAM)
- Framebuffer is 240×240×2 = 115,200 bytes (RGB565)

### Flash Partitions (partition_table.csv)
| Name     | Type | Size  | Purpose                    |
|----------|------|-------|----------------------------|
| nvs      | data | 24KB  | Non-volatile storage       |
| phy_init | data | 4KB   | PHY calibration            |
| factory  | app  | 3MB   | Application                |
| storage  | data | 3MB   | User data (FAT, ekv store) |

---

## Dual-Core Architecture

### Core 0 (Main Core)
- Embassy executor running multiple tasks
- Tasks: display, UI, render, LED ring, CLI, flash, LDC sensor, brightness
- Cooperative multitasking via async/await

### Core 1 (Dedicated FOC)
- Single task: `update_foc()`
- Runs the FOC loop at maximum speed (~20kHz target)
- Communicates with Core 0 via atomic variables and signals

### Inter-Core Communication
```rust
// Atomic encoder position (written by Core 1, read anywhere)
smartknob_core::haptic_core::get_encoder_position() -> f32

// Motor command signal (written by CLI/UI, read by FOC task)
MOTOR_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, MotorCommand>

// Display brightness
DISPLAY_BRIGHTNESS_SIGNAL: Signal<CriticalSectionRawMutex, u8>

// Knob events (tilt, press)
KNOB_EVENTS_CHANNEL: PubSubChannel<CriticalSectionRawMutex, KnobTiltEvent, 10, 4, 1>
```

---

## LVGL Integration Details

### Crate Stack
```
lv_bevy_ecs v0.6.5     ← High-level Rust API + Bevy ECS
    └── lightvgl-sys v9.4.3  ← Raw C bindings to LVGL 9.4
            └── LVGL 9.4 (compiled from source via build.rs)
```

### lv_conf.h Key Settings

Location: `smartknob/lv_conf.h`

| Setting | Value | Notes |
|---------|-------|-------|
| `LV_COLOR_DEPTH` | 16 | RGB565 format |
| `LV_USE_STDLIB_MALLOC` | `LV_STDLIB_CLIB` | Uses libc malloc |
| `LV_DEF_REFR_PERIOD` | 33ms | ~30 FPS target |
| `LV_DPI_DEF` | 130 | Dots per inch |
| `LV_USE_OS` | `LV_OS_NONE` | No OS threading |
| `LV_USE_DRAW_SW` | 1 | Software rendering |
| `LV_DRAW_SW_SUPPORT_RGB565` | 1 | Only RGB565 enabled |
| `LV_USE_LOG` | 1 | LVGL logging enabled |
| `LV_LOG_LEVEL` | `LV_LOG_LEVEL_WARN` | |
| `LV_USE_SYSMON` | 1 | Performance monitoring |
| `LV_USE_PERF_MONITOR` | 1 | FPS/CPU display |
| `LV_USE_MEM_MONITOR` | 1 | Memory display |

### Currently Enabled Widgets
| Widget | Enabled | Notes |
|--------|---------|-------|
| `LV_USE_ARC` | 1 | Arc/gauge widget |
| `LV_USE_LABEL` | 1 | Text labels |
| `LV_USE_THEME_DEFAULT` | 1 | Default theme |
| All others | 0 | Disabled to save space |

### Enabling Additional Widgets

To add more widgets, edit `smartknob/lv_conf.h`:

```c
// Example: Enable button widget
#define LV_USE_BUTTON        1

// Example: Enable slider (requires bar)
#define LV_USE_BAR           1
#define LV_USE_SLIDER        1

// Example: Enable image support
#define LV_USE_IMAGE         1

// Example: Enable flexbox layout
#define LV_USE_FLEX          1
```

After changing `lv_conf.h`, you must **rebuild from scratch**:
```bash
cargo clean
cargo build --release --bin seedlabs_devkit
```

### Enabling Additional Fonts

```c
// In lv_conf.h - enable more Montserrat sizes
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1

// Set different default font
#define LV_FONT_DEFAULT &lv_font_montserrat_16
```

### Display Buffer Configuration

In `display.rs`:
```rust
const WIDTH: usize = 240;
const HEIGHT: usize = 240;
const LINE_HEIGHT: usize = HEIGHT / 20;  // 12 lines per buffer

// Partial buffer: renders 12 lines at a time
let buffer = DrawBuffer::<{ WIDTH * LINE_HEIGHT }, Rgb565>::create(
    WIDTH as u32, 
    LINE_HEIGHT as u32
);
```

Increasing `LINE_HEIGHT` uses more RAM but may improve performance.

---

## LVGL API Usage Patterns

### Basic Widget Creation (display.rs)

```rust
use lv_bevy_ecs::{
    display::{Display, DrawBuffer},
    events::Event,
    functions::*,
    support::Align,
    widgets::{Arc, Label, LvglWorld},
};

// Initialize LVGL
lv_init();
lv_bevy_ecs::logging::connect();

// Create widgets
let mut arc = Arc::create_widget();
lv_obj_set_size(&mut arc, 150, 150);
lv_arc_set_rotation(&mut arc, 135);
lv_arc_set_bg_angles(&mut arc, 0, 270);
lv_arc_set_value(&mut arc, 10);
lv_obj_set_align(&mut arc, Align::Center.into());

let mut label = Label::create_widget();
lv_label_set_long_mode(&mut label, LabelLongMode::Clip.into());
lv_label_set_text_static(&mut label, c"Hello");
lv_obj_set_align(&mut label, Align::TopMid.into());

// Add event callback
lv_obj_add_event_cb(&mut arc, Event::ValueChanged, |mut event| {
    let Some(mut obj) = lv_event_get_target_obj(&mut event) else {
        return;
    };
    let value = lv_arc_get_value(&mut obj);
    let text = CString::new(value.to_string()).unwrap();
    lv_label_set_text(&mut label, text.as_c_str());
});

// Add to world (Bevy ECS tracking)
let mut world = LvglWorld::default();
world.spawn(label);
world.spawn(arc);
```

### Display/Render Loop Architecture

```rust
// Task 1: display_task - Sends framebuffer to hardware
#[embassy_executor::task]
async fn display_task(...) {
    // Initialize SPI, display driver
    // Register LVGL display callback
    display.register(buffer, |refresh| {
        // Copy rendered pixels to framebuffer
        fbuf.fill_contiguous(&area, data);
        FRAMEBUFFER_READY.signal(());
    });
    
    loop {
        FRAMEBUFFER_READY.wait().await;
        tft_display.show_raw_data(...).await;
    }
}

// Task 2: render_task - Drives LVGL rendering
#[embassy_executor::task]
async fn render_task(...) {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        lv_tick_inc(diff.into());  // Update LVGL tick
        lv_timer_handler();        // Process timers, render
        ticker.next().await;
    }
}

// Task 3: ui_task - Application UI logic
#[embassy_executor::task]
async fn ui_task() {
    LVGL_READY_SIGNAL.wait().await;
    // Create widgets, handle app logic
}
```

### Common LVGL Functions Available

```rust
// Object manipulation
lv_obj_set_size(&mut obj, width, height)
lv_obj_set_pos(&mut obj, x, y)
lv_obj_set_align(&mut obj, Align::Center.into())
lv_obj_add_flag(&mut obj, ObjFlag::Hidden)
lv_obj_remove_flag(&mut obj, ObjFlag::Hidden)

// Arc widget
lv_arc_set_value(&mut arc, value)
lv_arc_get_value(&mut arc) -> i32
lv_arc_set_range(&mut arc, min, max)
lv_arc_set_rotation(&mut arc, degrees)
lv_arc_set_bg_angles(&mut arc, start, end)

// Label widget
lv_label_set_text(&mut label, text: &CStr)
lv_label_set_text_static(&mut label, text: &CStr)
lv_label_set_long_mode(&mut label, mode)

// Events
lv_obj_add_event_cb(&mut obj, Event::ValueChanged, |event| { ... })
lv_event_get_target_obj(&mut event) -> Option<Obj>

// Timer/tick
lv_tick_inc(duration)
lv_timer_handler()
```

### Style Example

```rust
use lv_bevy_ecs::support::*;

// Create a style
let mut style = lv_style_t::default();
lv_style_init(&mut style);
lv_style_set_bg_color(&mut style, lv_color_make(255, 0, 0));
lv_style_set_border_width(&mut style, 2);

// Apply to widget
lv_obj_add_style(&mut obj, &style, StylePart::Main.into());
```

---

## Haptic Curve System

### Creating Haptic Curves

```rust
use haptic_lib::{CurveBuilder, Easing, EasingType};

// Build a detent-like curve
let curve = CurveBuilder::<6>::new()   // max 6 components
    .add_eased(0.3, 1.0, 0.0, Easing::Cubic(EasingType::Out))  // ramp down
    .add_eased(0.5, 0.0, -1.0, Easing::Cubic(EasingType::In)) // center valley
    .add_eased(0.5, 1.0, 0.0, Easing::Cubic(EasingType::Out))  // ramp up
    .add_eased(0.3, 0.0, -1.0, Easing::Cubic(EasingType::In)) // back to center
    .build()
    .unwrap()
    .make_absolute(I16F16::ZERO);  // anchor at position 0

// Apply curve
haptic_core.set_curve(&curve, 1.0).await;  // scale = 1.0
```

### Component Types
- `add_const(width, value)` - Constant torque over range
- `add_linear(width, start, end)` - Linear interpolation
- `add_eased(width, start, end, easing)` - Eased transition

### Easing Functions
- `Easing::Linear`
- `Easing::Quadratic(In/Out/InOut)`
- `Easing::Cubic(In/Out/InOut)`
- `Easing::Quartic(In/Out/InOut)`
- `Easing::Sinusoidal(In/Out/InOut)`
- `Easing::Exponential(In/Out/InOut)`

### Visualizing Curves (Desktop)

```bash
cd haptic_demo
cargo run
# Opens chart.html with interactive curve visualization
```

---

## CLI Commands Reference

| Command | Description |
|---------|-------------|
| `help` | Show all commands |
| `exit` | Exit CLI, resume logging |
| `shutdown` | Initiate system shutdown |
| `log-enable <channel>` | Enable log channel |
| `log-disable <channel>` | Disable log channel |
| `log-list` | List all log channels |
| `flash-format` | Format flash storage |
| `align` | Start motor alignment procedure |
| `encoder-validate` | Validate encoder linearity |
| `tune-up <value>` | Increase FOC zero offset |
| `tune-down <value>` | Decrease FOC zero offset |
| `tune-store` | Save tuned alignment |
| `beep <freq> <dur_ms> <vol%> <offset>` | Play tone via motor |
| `brightness <percent>` | Set display brightness |

### Log Channels
- `Encoder` - Encoder position logging
- `FOCLoop` - FOC loop statistics
- `Render` - UI render timing
- `DisplayTransfer` - SPI transfer timing
- `Push` - Knob press/tilt events

---

## Hardware Pin Mapping

### seedlabs_devkit (primary)
| Function | GPIO |
|----------|------|
| LED Ring (WS2812B) | GPIO12 |
| Motor UH | GPIO8 |
| Motor UL | GPIO16 |
| Motor VH | GPIO18 |
| Motor VL | GPIO7 |
| Motor WH | GPIO17 |
| Motor WL | GPIO15 |
| Encoder CLK | GPIO13 |
| Encoder MISO | GPIO14 |
| Encoder CS | GPIO11 |
| LCD SCK | GPIO4 |
| LCD MOSI | GPIO3 |
| LCD DC | GPIO2 |
| LCD CS | GPIO9 |
| LCD BL | GPIO5 |
| LCD RST | GPIO10 |

### main.rs (original hardware)
| Function | GPIO |
|----------|------|
| LED Ring | GPIO39 |
| LDC INT | GPIO40 |
| I2C SCL | GPIO42 |
| I2C SDA | GPIO41 |
| Motor UH | GPIO48 |
| Motor VH | GPIO21 |
| Motor WH | GPIO18 |
| LCD SCK | GPIO13 |
| LCD MOSI | GPIO14 |
| LCD DC | GPIO16 |
| LCD CS | GPIO15 |
| LCD BL | GPIO9 |
| LCD RST | GPIO8 |

---

## Common Development Tasks

### Adding a New LVGL Widget Type

1. Enable in `lv_conf.h`:
   ```c
   #define LV_USE_BUTTON 1
   ```

2. Use in Rust (if binding exists in lv_bevy_ecs):
   ```rust
   use lv_bevy_ecs::widgets::Button;
   let mut btn = Button::create_widget();
   ```

3. If binding doesn't exist, use raw FFI via lightvgl-sys

### Adding Encoder Input to UI

```rust
use smartknob_core::haptic_core::get_encoder_position;

loop {
    let pos = get_encoder_position();
    lv_arc_set_value(&mut arc, (pos * 10.0) as i32);
    Timer::after_millis(16).await;  // ~60 FPS update
}
```

### Responding to Knob Events

```rust
use crate::signals::KNOB_EVENTS_CHANNEL;

let mut receiver = KNOB_EVENTS_CHANNEL.subscriber().unwrap();
loop {
    match receiver.next_message_pure().await {
        KnobTiltEvent::PressStart => { /* handle press */ },
        KnobTiltEvent::TiltStart(dir) => { /* handle tilt */ },
        _ => {}
    }
}
```

---

## Debugging Tips

### Memory Issues
- Check `esp_alloc::HEAP.stats()` for heap usage
- PSRAM is used for large allocations (framebuffer)
- Stack overflow often manifests as random crashes

### Display Issues
- Ensure DMA buffers match expected sizes
- Check `LV_COLOR_DEPTH` matches framebuffer format
- RGB565 color order can be BGR or RGB (`ColorOrder::Bgr`)

### Build Issues
- Clean build required after `lv_conf.h` changes
- Missing symbols → check linker scripts
- `set_env.sh` must be sourced in each terminal session

### Performance
- FOC loop should stay under 50µs per iteration
- Render times over 33ms cause frame drops
- Use log channels selectively (logging has overhead)

---

## Key Dependencies & Versions

| Crate | Version | Purpose |
|-------|---------|---------|
| esp-hal | 1.0.0 | ESP32-S3 HAL |
| esp-rtos | 0.2.0 | Embassy integration |
| embassy-executor | 0.9.1 | Async executor |
| lv_bevy_ecs | 0.6.5 | LVGL Rust bindings |
| lightvgl-sys | 9.4.3 | Raw LVGL bindings |
| foc | 0.3.0 | FOC algorithms |
| ekv | (git) | Key-value flash store |
| bevy_ecs | 0.17.0 | ECS framework |

---

## Future Development Notes

### Expanding LVGL UI
- Consider enabling `LV_USE_FLEX` and `LV_USE_GRID` for layouts
- More fonts increase binary size significantly
- Image support requires decoder and increases memory usage

### UI Architecture Ideas
- Separate UI crate for desktop simulation (mentioned in CONCEPT.md)
- Map encoder → scroll wheel, knob press → Enter key
- Tilt directions → arrow keys

### Thread Safety
- LVGL is NOT thread-safe; call `lv_*` functions only from one task
- Use Embassy signals/channels for cross-task communication
- Atomic types for simple cross-core data sharing

