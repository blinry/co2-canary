use heapless::Vec;

/*pub struct Measurement {
    value: u16,
    time: u64, // TODO: use a proper time type
}*/

const HISTORY_SIZE: usize = 200;

pub struct History {
    values: Vec<u16, HISTORY_SIZE>,
}

impl History {
    pub const fn new() -> Self {
        History { values: Vec::new() }
    }

    pub fn add_measurement(&mut self, measurement: u16) {
        if self.values.is_full() {
            self.values.remove(0);
        }
        self.values.push(measurement).unwrap();
    }

    pub fn data_for_display(&self) -> &[u16] {
        self.values.as_slice()
    }
}
