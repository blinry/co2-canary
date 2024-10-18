use core::slice::Windows;
use heapless::HistoryBuffer;

/*pub struct Measurement {
    value: u16,
    time: u64, // TODO: use a proper time type
}*/

const HISTORY_SIZE: usize = 200;

pub struct History {
    values: HistoryBuffer<u16, HISTORY_SIZE>,
}

impl History {
    pub const fn new() -> Self {
        History {
            values: HistoryBuffer::new(),
        }
    }

    pub fn add_measurement(&mut self, measurement: u16) {
        self.values.write(measurement);
    }

    pub fn data_for_display(&self) -> (usize, impl Iterator<Item = &u16>) {
        (self.values.len(), self.values.iter())
    }

    pub fn at(&self, index: usize) -> u16 {
        self.values[index]
    }

    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn max_value(&self) -> Option<u16> {
        self.values.iter().max().copied()
    }

    pub fn windows_2(&self) -> Windows<u16> {
        self.values.windows(2)
    }
}
