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
. $HOME/export-esp.sh
```

Always compile for release

```sh
cargo build --release
```