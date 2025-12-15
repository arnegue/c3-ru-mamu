use crate::telemetry_data::TelemetryData;
use heapless::Vec;

#[derive(Debug)]
pub enum ParseError {
    IncompleteData,     // Not enough data was received to finish parsing
    UnsupportedMessage, // Message is not supported (yet)
    InvalidFormat,      // e.g. special characters are missing
}

// Trait responsible for returning MessageType in correct format (e.g. NMEA messages start with $ (or !) and end with \n)
pub trait MessageParser<
    RawMessageType,                  // Underlying format like u8, u16 or similar
    const MAX_Message_length: usize, // Maximum size of a RawMessageType
>
{
    fn parse(
        &mut self,
        raw: &mut Vec<RawMessageType, MAX_Message_length>,
    ) -> Result<TelemetryData, ParseError>;
    fn write(&self, message: TelemetryData) -> Vec<RawMessageType, MAX_Message_length>; // serialize back to bytes
}
