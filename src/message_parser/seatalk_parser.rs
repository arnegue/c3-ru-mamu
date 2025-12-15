use heapless::Vec;

use crate::{
    message_parser::message_parser::{MessageParser, ParseError},
    telemetry_data::TelemetryData,
};

// ################# Seatalk1 #################
type SeatalkType = u16; // Actually it's mostly u8, only u16 to detect 9th bit (command byte)
const MAXIMUM_SEATALK_LENGTH: usize = 25;
type RawSeatalkMessage = Vec<SeatalkType, MAXIMUM_SEATALK_LENGTH>;

enum SeatalkStateMachineStates {
    None,      // Start, No Command Byte was yet received
    CmdByte,   // Command Byte was received
    DataBytes, // DataBytes were received. When finished, Parsing can happen
}

// StateMachine responsible for that a correct, parsable message was received (command byte, length byte and correct ammount of data bytes were received)
struct SeatalkParserStateMachine {
    current_state: SeatalkStateMachineStates,
    current_message: RawSeatalkMessage,
}

impl SeatalkParserStateMachine {
    pub fn new() -> Self {
        SeatalkParserStateMachine {
            current_state: SeatalkStateMachineStates::None,
            current_message: Default::default(),
        }
    }

    pub fn update(&mut self, next_word: u16) {
        match self.current_state {
            SeatalkStateMachineStates::None => self.none(next_word),
            SeatalkStateMachineStates::CmdByte => self.cmd_byte(next_word),
            SeatalkStateMachineStates::DataBytes => self.data_bytes(next_word),
        }
    }

    // Checks if given word is a command byte
    fn is_command_byte(next_word: u16) -> bool {
        return (next_word >> 8) > 0; // If MSB is bigger than 0, it's a command byte
    }

    // State None
    fn none(&mut self, next_word: u16) {
        if Self::is_command_byte(next_word) {
            self.current_state = SeatalkStateMachineStates::CmdByte;
            self.cmd_byte(next_word);
        }
        // Discard byte that's no command byte
    }

    // State Command Byte
    fn cmd_byte(&mut self, next_word: u16) {
        // Case: Multiple consecutive command bytes
        if Self::is_command_byte(next_word) {
            self.current_message.clear();
            self.current_message.push(next_word);
            self.current_state = SeatalkStateMachineStates::CmdByte;
        } else {
            // Else it's a length byte
            self.current_state = SeatalkStateMachineStates::DataBytes;
            self.data_bytes(next_word);
        }
    }

    // State Length Byte
    fn data_bytes(&mut self, next_word: u16) {
        // Case: Multiple consecutive command bytes
        if Self::is_command_byte(next_word) {
            self.current_state = SeatalkStateMachineStates::CmdByte;
            self.cmd_byte(next_word);
            return;
        }

        self.current_message.push(next_word);

        // Shortest message over all is 3 bytes (cmd byte, length byte, data byte)
        let expected_length = ((self.current_message[1] & 0b1111) + 3) as usize;
        if self.current_message.len() == expected_length {
            self.current_state = SeatalkStateMachineStates::None;
            // Finished
        }
        // else:
        // - Not enough bytes received, stay in state
        // - Too many bytes received can't happen, because one byte after another gets pushed into
    }
}

pub struct SeatalkMessageParser {
    state_machine: SeatalkParserStateMachine,
}

impl MessageParser<SeatalkType, MAXIMUM_SEATALK_LENGTH> for SeatalkMessageParser {
    fn parse(&mut self, raw: &mut RawSeatalkMessage) -> Result<TelemetryData, ParseError> {
        for word in raw {
            self.state_machine.update(*word);
        }

        Ok(TelemetryData {
            speed: Some(5.0),
            depth: None,
            course: None,
        })
    }

    fn write(&self, message: TelemetryData) -> RawSeatalkMessage {
        todo!()
    }
}

// TODO test with parsing:
// E.g. "Select Fathoms", but first byte (with command is garbage)
//         Garbage                     Actual Datagram
// (0x01 0x00)             (0x01 0x65 | 0x00  0x00 | 0x00  0x02)
// (0x01 0x00 | 0x00 0x02) (0x01 0x65 | 0x00  0x00 | 0x00  0x02)
//            ( 0x00 0x02) (0x01 0x65 | 0x00  0x00 | 0x00  0x02)
