use heapless::Vec;

use crate::{
    message_parser::message_parser::{MessageParser, ParseError},
    telemetry_data::TelemetryData,
};

// ################# NMEA0183 #################
type NMEAType = u8;
const MAXIMUM_NMEA_LENGTH: usize = 82;
type RawNMEAMessage = Vec<NMEAType, MAXIMUM_NMEA_LENGTH>;

pub struct NMEAMessageParser {}

impl MessageParser<NMEAType, MAXIMUM_NMEA_LENGTH> for NMEAMessageParser {
    fn parse(&mut self, raw: &mut RawNMEAMessage) -> Result<TelemetryData, ParseError> {
        // Look for a start marker (e.g. '$')
        while let Some(&b) = raw.get(0) {
            if b == b'$' || b == b'!' {
                break; // found start
            } else {
                raw.remove(0); // discard garbage
            }
        }
        Err(ParseError::IncompleteData)
    }

    fn write(&self, message: TelemetryData) -> RawNMEAMessage {
        todo!()
    }
}
