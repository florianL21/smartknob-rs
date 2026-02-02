#![no_std]
//! This crate provides a few ready-made hardware agnostic modules for building a smartknob system.
//! Most modules in here somehow interface with each other, but are made in a way where they
//! can be used in isolation if their inputs and outputs (usually in the form of signals) are serviced by something.
//! Please check the documentation of the submodules of this create to learn more about each individual system.

pub mod cli;
pub mod display;
pub mod flash;
pub mod haptic_core;
pub mod knob_tilt;
pub mod led_ring;
pub mod shutdown;
pub mod system_settings;
