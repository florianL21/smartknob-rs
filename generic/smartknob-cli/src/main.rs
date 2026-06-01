mod cli;
mod curves;
mod smartknob;

use anyhow::{Context, Result, anyhow, bail};
use charming::HtmlRenderer;
use clap::Parser;
use log::{error, info, warn};
use schemars::schema_for;
use serde::{Deserialize, Serialize};
use smartknob_core::comm::{self, LogChannel};
use smartknob_core::haptics::HapticConfiguration;
use smartknob_core::haptics::base::{CurveBuilder, CurveSegment};
use std::path::PathBuf;
use std::time::Duration;
use std::{fs::File, io::BufReader, io::BufWriter};
use tokio::spawn;
use tokio::task::JoinSet;
use tokio::time::sleep;

use crate::cli::MotorArgs;
use crate::smartknob::comm::{ConnectionError, event_printer, log_printer};
use crate::{
    cli::{Cli, Command, CurveActions, CurveArgs, LogState},
    curves::create_graph,
    smartknob::comm::Communication,
};

#[derive(Serialize, Deserialize, schemars::JsonSchema)]
pub struct HapticConfigurationWithSchema {
    config: HapticConfiguration,
    #[serde(rename = "$schema")]
    schema: String,
}

impl HapticConfigurationWithSchema {
    fn new(config: HapticConfiguration) -> Self {
        Self {
            config,
            schema: "haptic_config_schema.json".into(),
        }
    }
}

async fn handle_curve_command(curve_file: PathBuf, action: CurveActions) -> Result<()> {
    match action {
        CurveActions::Push => {
            let file = File::open(curve_file)?;
            let reader = BufReader::new(file);
            let config: HapticConfigurationWithSchema = serde_json::from_reader(reader)?;
            send_single_message_expect_ack(
                comm::Command::PushHapticConfig(config.config),
                Duration::from_secs(1),
            )
            .await?;
        }
        CurveActions::Init => {
            if curve_file.exists() {
                bail!(
                    "The curve file at {curve_file:?} already exists. When calling init the file must not yet exist"
                );
            }
            let folder = curve_file.parent().context(
                "Could not get partent directory of given output file path ({curve_file_path})",
            )?;
            let schema_file_path = folder.with_file_name("haptic_config_schema.json");
            let file = File::create(schema_file_path).context("Schema file")?;
            let writer = BufWriter::new(file);
            let schema = schema_for!(HapticConfigurationWithSchema);
            serde_json::to_writer_pretty(writer, &schema)?;

            let mut curve_builder = CurveBuilder::new();
            let seg1 = curve_builder.new_segment(
                CurveSegment::new()
                    .add_bezier3(0.5, [0.0, -0.1, -1.0])
                    .add_bezier3(0.5, [1.0, 0.1, 0.0]),
            );
            let endstop_left =
                curve_builder.new_segment(CurveSegment::new().add_linear(0.8, 1.0, 0.0));
            let endstop_right =
                curve_builder.new_segment(CurveSegment::new().add_linear(0.8, 0.0, -1.0));
            let test_curve = curve_builder
                .push(endstop_left)
                .push_repeated(seg1, 3)
                .push(endstop_right)
                .finish(-0.3);
            let file = File::create(curve_file)?;
            let writer = BufWriter::new(file);

            serde_json::to_writer_pretty(
                writer,
                &HapticConfigurationWithSchema::new(HapticConfiguration::Curve(test_curve)),
            )?;
        }
        CurveActions::Visualize {
            start_angle,
            graph_output_file,
            open: open_file,
        } => {
            let file = File::open(curve_file)?;
            let reader = BufReader::new(file);
            let config: HapticConfigurationWithSchema = serde_json::from_reader(reader)?;
            if let HapticConfiguration::Curve(curve) = config.config {
                let chart = create_graph(start_angle.unwrap_or_default(), curve, 0.01);

                let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
                renderer.save(&chart, &graph_output_file)?;
                if open_file {
                    open::that(graph_output_file)?;
                }
            } else {
                bail!("Haptic config is not a curve type and can thus not be visualized");
            }
        }
    }
    Ok(())
}

async fn send_single_message(msg: comm::Command, timeout: Duration) -> Result<comm::Response> {
    let mut comm = Communication::init().await?;
    match comm.take_log().await {
        Ok(stream) => {
            spawn(log_printer(stream));
        }
        Err(ConnectionError::AlreadyConnected) => {
            warn!(
                "Log port is already opened somewhere else. No logs from the smarknob will be included in this commands log output"
            );
        }
        Err(e) => {
            error!("Failed to initialize log port: {e}");
        }
    }
    let mut comms = comm.take_comms().await?;
    // Sleep a bit of time before returning to give the logger task time to get logs from the smartknob before terminating the program
    let resp = comms.send_command(&msg, timeout).await?;
    sleep(Duration::from_millis(100)).await;
    Ok(resp)
}

async fn send_single_message_expect_ack(msg: comm::Command, timeout: Duration) -> Result<()> {
    let reply = send_single_message(msg, timeout).await;
    if let Ok(comm::Response::Ack) = reply {
        Ok(())
    } else {
        Err(anyhow!(
            "Did not get expected reply of Ack. Instead Got: {reply:?}"
        ))
    }
}

async fn handle_log_command(channel: LogChannel, enabled: LogState) -> Result<()> {
    let msg = match enabled {
        LogState::Enable => comm::Command::Log {
            channel,
            enabled: true,
        },
        LogState::Disable => comm::Command::Log {
            channel,
            enabled: false,
        },
    };
    send_single_message_expect_ack(msg, Duration::from_secs(1)).await
}
async fn handle_motor_commends(args: MotorArgs) -> Result<()> {
    match args.action {
        cli::MotorActions::Calibrate => {
            send_single_message_expect_ack(comm::Command::MotorCalibrate, Duration::from_secs(30))
                .await
        }
        cli::MotorActions::Beep {
            freq,
            duration,
            volume,
            note_offset,
        } => {
            send_single_message_expect_ack(
                comm::Command::Beep {
                    freq,
                    duration,
                    volume,
                    note_offset,
                },
                Duration::from_secs(5),
            )
            .await
        }
        cli::MotorActions::MotorTune { value } => {
            send_single_message_expect_ack(comm::Command::MotorTune(value), Duration::from_secs(1))
                .await
        }
        cli::MotorActions::MotorTuneStore => {
            send_single_message_expect_ack(comm::Command::MotorTuneStore, Duration::from_secs(3))
                .await
        }
        cli::MotorActions::Validate => {
            send_single_message_expect_ack(comm::Command::EncoderValidate, Duration::from_secs(30))
                .await
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();
    let cli = Cli::parse();

    match cli.command {
        Command::Log { channel, enabled } => handle_log_command(channel, enabled).await?,
        Command::Curve(CurveArgs { curve_file, action }) => {
            handle_curve_command(curve_file, action).await?
        }
        Command::Watch { events, logs } => {
            let mut comm = Communication::init().await?;
            let mut set = JoinSet::new();
            if logs {
                match comm.take_log().await {
                    Ok(stream) => {
                        info!("Connected successfully, start streaming log messages now");
                        set.spawn(log_printer(stream));
                    }
                    Err(e) => {
                        bail!("Failed to connect to log port: {e}");
                    }
                }
            }
            if events {
                match comm.take_comms().await {
                    Ok(mut channels) => {
                        let events = channels.take_event_receiver()?;
                        set.spawn(event_printer(events));
                    }
                    Err(e) => {
                        bail!("Failed to connect to event port: {e}");
                    }
                }
            }
            let _ = set.join_all().await;
        }
        Command::Motor(args) => {
            handle_motor_commends(args).await?;
        }
        Command::Shutdown => {
            send_single_message_expect_ack(comm::Command::Shutdown, Duration::from_secs(1)).await?;
        }
        Command::Brightness { percent } => {
            send_single_message_expect_ack(
                comm::Command::Brightness { percent },
                Duration::from_secs(1),
            )
            .await?;
        }
        Command::FlashErase => {
            send_single_message_expect_ack(comm::Command::FlashErase, Duration::from_secs(1))
                .await?;
        }
    }
    Ok(())
}
