use crate::data_provider::DataProvider;
use heapless::Vec;

// Trait responsible for returning TypeMessages in correct length (e.g. NMEA messages start with $ (or !) and end with \n)
pub trait MessageProvider<DataProviderType, MessageType, const MAX_Message_length: usize> {
    fn get_data_provider(&self) -> &dyn DataProvider<DataProviderType>;

    fn get_message(&self) -> Vec<MessageType, MAX_Message_length>;
}

type NMEAType = u8;
const MAXIMUM_NMEA_LENGTH: usize = 82;
type NMEAMessage = Vec<NMEAType, MAXIMUM_NMEA_LENGTH>;

struct NMEAMessageProvider<'a, NMEAType> {
    data_provider: &'a (dyn DataProvider<NMEAType> + 'a),
}

impl<'a> MessageProvider<NMEAType, NMEAType, MAXIMUM_NMEA_LENGTH>
    for NMEAMessageProvider<'a, NMEAType>
{
    fn get_data_provider(&self) -> &dyn DataProvider<NMEAType> {
        self.data_provider
    }

    fn get_message(&self) -> Vec<NMEAType, MAXIMUM_NMEA_LENGTH> {
        todo!()
    }
}

type SeatalkDataType = u8; // Data is actually in u8, but command bit is the 9th bit
type SeatalkMessageType = u16; // Only to detect command bit
const MAXIMUM_SEATALK_LENGTH: usize = 25;
type SeatalkMessage = Vec<NMEAType, MAXIMUM_NMEA_LENGTH>;

struct SeatalkMessageProvider<'a, SeatalkType> {
    data_provider: &'a (dyn DataProvider<SeatalkType> + 'a),
}

impl<'a> MessageProvider<SeatalkMessageType, SeatalkDataType, MAXIMUM_SEATALK_LENGTH>
    for SeatalkMessageProvider<'a, SeatalkMessageType>
{
    fn get_data_provider(&self) -> &dyn DataProvider<SeatalkMessageType> {
        self.data_provider
    }

    fn get_message(&self) -> Vec<SeatalkDataType, MAXIMUM_SEATALK_LENGTH> {
        todo!()
    }
}
