mod cli;
mod curves;
mod smartknob;

use anyhow::{Context, Result};
use clap::Parser;
use futures::SinkExt;
use log::info;

use crate::{
    cli::{Cli, Command, LogState},
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
        Command::Curve(_) => {
            todo!();
        }
    }

    Ok(())
}
