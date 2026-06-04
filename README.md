# This is heavily work in progress!

This is very much still in development and not ready to be used or even functional in any capacity!
The only reason to look at this repo is if you are interested in helping out with developing it

# For users on Github

This project was moved to [Codeberg](https://codeberg.org/florianL21/smartknob-rs).
The issues on github have been locked and PRs were deactivated. The code here will simply mirror the one on codeberg now.
If you want to contribute/create issues pelase do so at the codeberg repository.

# Current status of features

- [x] Create and play back haptic curves
- [x] Render something on the display using slint
- [x] All hardware drivers are present and functional
- [x] Communication with the PC
- [ ] Haptic feedback like klicks
- [ ] Any sort of actually useful UI

# Quick overview of the project

This firmware makes use of many rust crates for various functionality but the main ones used are:

- `embassy` for the no_std, async, bare metal runtime
- `esp-hal` for the ESP32-S3 HAL abstraction
- `slint-ui` as a UI library and for rendering the user interface on the display
- `mipidsi` for driving the display
- `foc` for the FOC control algorithms for driving the motor
- `ekv` for storing configuration values in flash

Some of these crates may have been forked by me to make async versions of them to be able to use them more comfortably with embassy.

## Firmware layout

The main firmware is located under the `embedded/smartknob` folder.
Currently the `src/bin/main.rs` file is used to start up and spawn all needed tasks.
At this point it is worth mentioning that most tasks run on core0, except
for the motor control task which is the only task running on core1 to ensure
the most amount of CPU time possible for running the FOC calculations.

This repo currently supports 2 hardware variants that I custom made.
It also has basic support for the seeedlabs devkit hardware.
Adding support for new hardware should be pretty easy at this point as a lot of things
have been abstracted away.
The core logic layers are already fully hardware agnostic. These can be found under the `generic` folder.
Layers more specific to ESP32S3 chips can be found in the `embedded/smartknob-esp32` folder. These are specific to the chip, but do not make any assumptions about how hardware is connected together.
The final firmware and "wiring" of a particular hardware is done int he files in `embedded/smartknob/src/bin`.
One file is for one specific hardware implementation.

Furthermore this repo now supports LVGL as well as slint for the UI libraries.
There are still no real UI components implemented in either of them.
The implementor of a hardware may choose which UI stack they want to use.
Slint is configured as the default, If LVGL is preferred one can simply disable slint
support by adding `--no-default-features` to the CLI, and then add a `--features=lvgl` to enable LVGL support.
If compile time is of no concern both can also simply be left enabled at the same time.

Do note however that in order for the LVGL compilation to work you should source the `set_env.sh` script
in the root of this repo.

# How to get up and running

Hint: Do this while not having VSCode open as it may hog the files needed to modify the toolchains

Install espup:

```sh
cargo install espup
```

Install toolchains:

```sh
espup install
```

## How to compile

Environment setup for ESP toolchain

```sh
. set_env.sh
```

Always compile the full firmware from within the smartknob directory.
You can choose different hardware variants by changing the name given to "--bin"
Always compile for release

```sh
cd embedded/smartknob
cargo build --release --bin seedlabs_devkit
```

And to flash to the target run

```sh
cargo run --release --bin seedlabs_devkit
```

## Using the firmware

On the first startup of the system it needs to be calibrated.
This can be done by connecting the smartknob to the PC via USB and then run some commands from the smartknob-cli crate.
To get up and running quickly you can run the following:

```sh
cd generic/smartknob-cli
cargo run -- motor calibrate
```

When the firmware currently boots it shows nothing of use on the screen.
However it does already come preloaded with a demo of a haptic curve which runs from the start.

The most interesting feature of the firmware currently is that the
configuration of the haptic system cahn be changed live from the PC while the smartknob is running.
A few commands that can be of interest:

```sh
# This opens a webbrowser to visualize the haptic curve in the given json file
cargo run -- curve test_curve.json visualize chart.html -o
# The file can be modified and then pushed to the knob by running
cargo run -- curve test_curve.json push
# To view live logs from the smartknob you can open a monitor by running
cargo run -- watch
# While watch is running further commands can be issued from a second terminal session
```

If you are interested in more commands check the --help of the CLI
