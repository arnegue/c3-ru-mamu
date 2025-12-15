use esp_idf_hal::task::queue::Queue;
use esp_idf_sys::TickType_t;

use crate::message_parser::message_parser::MessageParser;
use crate::telemetry_data::TelemetryData;

use heapless::Vec;

pub trait Task {
    fn run(&mut self);
}

// Parser task, responsible for parsing data received via queue
pub struct ParserTask<'a, RawMessageType: Copy, const MAX_Message_length: usize> {
    pub parser: &'a mut dyn MessageParser<RawMessageType, MAX_Message_length>,
    pub inbound: Queue<RawMessageType>,
    pub outbound: Queue<TelemetryData>,
    current_chunks: Vec<RawMessageType, MAX_Message_length>,
}

impl<RawMessageType: Copy, const MAX_Message_length: usize> Task
    for ParserTask<'_, RawMessageType, MAX_Message_length>
{
    fn run(&mut self) {
        loop {
            if let Some((received_chunk, _)) = self.inbound.recv_front(TickType_t::MAX) {
                // New bytes received
                if self.current_chunks.push(received_chunk).is_err() {
                    log::error!("Could not push received chunk to current-chunks. Clearing them");
                    self.current_chunks.clear();
                    continue;
                }

                match self.parser.parse(&mut self.current_chunks) {
                    Ok(_) => todo!(),
                    Err(_) => todo!(),
                }

                // if let Some(data) = self.parser.parse(raw) {
                //     self.outbound.send_back(data, 100).unwrap();
                // }
            }
        }
    }
}

impl<'a, RawMessageType: Copy, const MAX_Message_length: usize>
    ParserTask<'a, RawMessageType, MAX_Message_length>
{
    /// Constructor for ParserTask
    pub fn new(
        parser: &'a mut dyn MessageParser<RawMessageType, MAX_Message_length>,
        inbound: Queue<RawMessageType>,
        outbound: Queue<TelemetryData>,
    ) -> Self {
        let current_chunks: Vec<RawMessageType, MAX_Message_length> = Default::default();

        Self {
            parser,
            inbound,
            outbound,
            current_chunks,
        }
    }
}
