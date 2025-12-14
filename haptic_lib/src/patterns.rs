use fixed::types::I16F16;
use heapless::Vec;

pub struct HapticPattern<const N: usize> {
    commands: Vec<I16F16, N>,
    repeat: u16,
    multiply: u16,
}

impl<const N: usize> HapticPattern<N> {
    pub fn new(commands: Vec<I16F16, N>, repeat: u16, multiply: u16) -> Self {
        Self {
            commands,
            repeat,
            multiply,
        }
    }
}
