#![cfg_attr(not(feature = "host"), no_std)]
//! This crate provides a few ready-made hardware agnostic modules for building a smartknob system.
//! Most modules in here somehow interface with each other, but are made in a way where they
//! can be used in isolation if their inputs and outputs (usually in the form of signals) are serviced by something.
//! Please check the documentation of the submodules of this create to learn more about each individual system.

pub mod comm;
pub mod haptics;
pub mod knob_tilt;
pub mod system_settings;

#[cfg(feature = "embed")]
pub mod display;
#[cfg(feature = "embed")]
pub mod flash;
#[cfg(feature = "embed")]
pub mod led_ring;
#[cfg(feature = "embed")]
pub mod shutdown;
#[cfg(feature = "embed")]
pub mod uplink;
