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
    /// Work with haptic curves
    Curve(CurveArgs),
    /// Enable or disable a log channel
    Log {
        /// The channel which to enable/disable
        #[arg(value_enum)]
        channel: idc::LogChannel,

        /// whether to enable or disable the log channel
        enabled: LogState,
    },
}

#[derive(Debug, Args)]
pub struct CurveArgs {
    /// File which contains a haptic curve definition
    curve_file: PathBuf,

    #[command(subcommand)]
    action: CurveActions,
}

#[derive(Debug, Subcommand)]
enum CurveActions {
    /// Pushes a haptic curve to a connected smartknob device
    Push,
    /// Visualizes a haptic curve in an html file with an interactiveley explorable graph
    Visualize { graph_output_file: PathBuf },
}
