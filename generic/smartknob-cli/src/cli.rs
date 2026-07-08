use clap::{Args, Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

/// CLI utility for smartknobs
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
pub struct Cli {
    /// Command to run
    #[command(subcommand)]
    pub command: Command,
}

#[derive(ValueEnum, Debug, Clone)]
pub enum LogState {
    Enable,
    Disable,
}

#[derive(Subcommand, Debug)]
pub enum Command {
    /// Connect to the smartknob and print out information from it, until cancelled
    Watch {
        /// Print out logs from the smartknob; active by default
        #[arg(short, long, default_value_t = true)]
        logs: bool,
        /// Print out all events coming from the smartknob
        #[arg(short, long, default_value_t = false)]
        events: bool,
    },
    /// Work with haptic curves
    Curve(CurveArgs),
    /// Work with interactive haptic programs written in the rhai (https://rhai.rs/book/) scripting language
    Program(ProgramArgs),
    /// Enable or disable a log channel
    Log {
        /// The channel which to enable/disable
        #[arg(value_enum)]
        channel: smartknob_core::comm::LogChannel,

        /// whether to enable or disable the log channel
        enabled: LogState,
    },
    /// Actions related to the motor system
    Motor(MotorArgs),
    /// Initiate a system shutdown
    Shutdown,
    /// Format the whole flash
    FlashErase,
    /// Set the backlight brightness of the display
    Brightness {
        /// Brightness in percent 0-100
        percent: u8,
    },
}

#[derive(Debug, Args)]
pub struct MotorArgs {
    #[command(subcommand)]
    pub action: MotorActions,
}

#[derive(Debug, Subcommand)]
pub enum MotorActions {
    /// Calibrate the smartknob system
    Calibrate,
    /// Validate that the encoder and motor are properly calibrated
    Validate,
    /// offset the current motor calibration by the given value
    MotorTune { value: f32 },
    /// Save the modified current electrical angle offset to flash
    MotorTuneStore,
    /// Make a beep sound
    Beep {
        /// Frequency of the beep
        freq: f32,
        /// duration in ms
        duration: u64,
        /// volume in % between 0 and 100
        volume: f32,
        /// Note offset frequency
        note_offset: u32,
    },
}

#[derive(Debug, Args)]
pub struct CurveArgs {
    /// File which contains a haptic curve definition
    pub curve_file: PathBuf,

    #[command(subcommand)]
    pub action: CurveActions,
}

#[derive(Debug, Subcommand)]
pub enum CurveActions {
    /// Pushes a haptic curve to a connected smartknob device
    Push,
    /// Create an empty haptic curve scaffold file in the given file location. The given file is not allowed to exist yet.
    /// A schema file is created next to it and will always be called haptic_curve_schema.json. This file will be overwritten if it already exists
    Init,
    /// Visualizes a haptic curve in an html file with an interactively explore-able graph
    Visualize {
        /// Angle to start rendering the graph from
        #[arg(short, long)]
        start_angle: Option<f32>,
        /// If the rendered output should be opened once it is written
        #[arg(short, long, default_value_t = false)]
        open: bool,
        /// File to render the visualized graph into
        graph_output_file: PathBuf,
    },
}

#[derive(Debug, Args)]
pub struct ProgramArgs {
    /// File which contains a haptic script
    pub script_file: PathBuf,

    #[command(subcommand)]
    pub action: ProgramActions,
}

#[derive(Debug, Subcommand)]
pub enum ProgramActions {
    /// Pushes a program to a connected smartknob device
    Push,
}
