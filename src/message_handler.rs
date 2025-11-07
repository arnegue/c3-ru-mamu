

const SPLITTER_BUFFER_MESSAGE_SIZE: usize = 5; // Factor for buffer lenghts

// Splitter splits messages in buffer to specific messages parsable by its handlers
trait MessageSplitter<MsgInputType, MsgOutputType, const MAX_MSG_LENGTH: usize>
{
    fn get_next_message(&self, buffer: [MsgInputType; SPLITTER_BUFFER_MESSAGE_SIZE]) -> [MsgOutputType; MAX_MSG_LENGTH];
}

// Provider that provides handler a parsable message
pub trait MessageProfider<MsgInputType, MsgOutputType, const MAX_MSG_LENGTH: usize> {
    fn set_splitter(&self, splitter: dyn MessageSplitter<MsgInputType, MsgOutputType, MAX_MSG_LENGTH>);
    fn get_message(&self) -> [u8; MAX_MSG_LENGTH];
}


const MAX_SEATALK_LENGTH: usize = 25; // Maximum length of a seatalk message


// Splitter for Seatalk Messages
struct SeatalkSplitter {
    message_buffer: [u8; SPLITTER_BUFFER_MESSAGE_SIZE]
}

impl MessageSplitter<u16, u8, MAX_SEATALK_LENGTH> for SeatalkSplitter {
    fn get_next_message(&self, buffer: [u16; SPLITTER_BUFFER_MESSAGE_SIZE]) -> [u8; MAX_SEATALK_LENGTH] {
        // Iterate buffer
        // Search for command bit
        // cut out size in buffer
        todo!()
    }
}

pub struct SeatalkMessageProvider {
    splitter: SeatalkSplitter
}

impl MessageProfider<u16, u8, MAX_SEATALK_LENGTH> for SeatalkMessageProvider {
    fn get_message(&self) -> [u8; MAX_SEATALK_LENGTH] {
        todo!()
    }
    
    fn set_splitter(&self, splitter: dyn MessageSplitter<u16, u8, MAX_SEATALK_LENGTH>) {
        self.splitter = splitter;
    }
}