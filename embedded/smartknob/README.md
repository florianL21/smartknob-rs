# Smartknob core library

The core library implements the core functionality of a smartknob system.
The library should be kept as generic as possible but to limit scope and complexity it is acceptable to
define reasonable limitations.

# Current deliberate limitations

- It is assumed that embassy-rs is used as the runtime
- It is assumed that the system uses esp32 hardware
    - Even with this assumption esp32 specific code should be kept ot a minimum and should be contained to a few files
- It is assumed that the system has at least 2 cores
- It is assumes that one of the cores is 100% dedicated to running the FOC loop
