mod cli;
mod curves;
mod smartknob;

use anyhow::{Context, Result, bail};
use charming::HtmlRenderer;
use clap::Parser;
use futures::SinkExt;
use log::info;
use smartknob_core::haptics::base::{CurveBuilder, CurveSegment};
use std::{fs::File, io::BufReader, io::BufWriter};

use crate::{
    cli::{Cli, Command, CurveActions, CurveArgs, LogState},
    curves::create_graph,
    smartknob::comm::init_comms,
};

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();
    let cli = Cli::parse();

    match cli.command {
        Command::Log { channel, enabled } => {
            let msg = match enabled {
                LogState::Enable => idc::Command::LogEnable(channel),
                LogState::Disable => idc::Command::LogDisable(channel),
            };
            let mut comms = init_comms().await?;
            comms.requests.send(&msg).await?;
            let resp = comms
                .responses
                .recv()
                .await
                .context("Could not get response")?;
            info!("Response: {resp:?}");
        }
        Command::Curve(CurveArgs { curve_file, action }) => match action {
            CurveActions::Push => {
                todo!();
            }
            CurveActions::Init => {
                if curve_file.exists() {
                    bail!(
                        "The curve file at {curve_file:?} already exists. When calling init the file must not yet exist"
                    );
                }
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
                serde_json::to_writer_pretty(writer, &test_curve)?;
            }
            CurveActions::Visualize {
                start_angle,
                graph_output_file,
                open: open_file,
            } => {
                let file = File::open(curve_file)?;
                let reader = BufReader::new(file);
                let curve = serde_json::from_reader(reader)?;

                let chart = create_graph(start_angle.unwrap_or_default(), curve, 0.01);

                let mut renderer = HtmlRenderer::new("Force graph", 1000, 800);
                renderer.save(&chart, &graph_output_file)?;
                if open_file {
                    open::that(graph_output_file)?;
                }
            }
        },
    }

    Ok(())
}
