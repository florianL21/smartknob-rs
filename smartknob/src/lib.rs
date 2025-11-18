#![no_std]
#![feature(type_alias_impl_trait)]

extern crate alloc;

pub mod cli;
pub mod config;
pub mod display;
pub mod flash;
pub mod knob_tilt;
pub mod motor_control;
pub mod shutdown;
