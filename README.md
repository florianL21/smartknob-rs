# This is not a functional smartknob firmware!
This is very much still in development and not ready to be used or even functional in any capacity!
The only reason to look at this repo is if you are interested in helping out with developing it


# Quick overview of the project

This firmware makes use of many rust crates for various functionality but the main ones used are:

* `embassy` for the no_std, async, bare metal runtime
* `esp-hal` for the ESP32-S3 HAL abstraction
* `slint-ui` as a UI library and for rendering the user interface on the display
* `mipidsi` for driving the display
* `foc` for the FOC control algorithms for driving the motor
* `ekv` for storing configuration values in flash
* `embedded-cli` for providing a debugging interface for playing around during development

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

The firmware does not do much as of right now.
But when it boots up it presents some log output thought he serial monitor.

By pressing the enter key you will be dropped into a "shell". Here you can type `help` to see all available commands
While in the "shell" no log output will be produced.

Once the `exit` command is issued the shell will exit and log message printing is enabled again.
