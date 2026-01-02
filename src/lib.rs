//! A simplified driver for the MAX31865 RTD to Digital converter (Raspberry Pi focus)
//!
//! # References
//! - Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
//! - Wiring diagrams:  https://www.playingwithfusion.com/docs/1203
//!

// TODO: Update and improve README (see other branches), esp sample code
// TODO: Improve and test fault handling, add to README test case
// TODO: Enhance RtdError to differentiate between Pin and Spi (transfer) errors.
// TODO: get down to a single Error type: Use RtdError directly in private code,
// TODO: Enable no_std => ![cfg_attr(not(test), no_std)]
// TODO: Create an ice bath manual test program - watch temperatures go down and up
//
//  Requirements for ice bath test
//      1. Explain to the user what's going to happen
//      2. Verify temperature is in the range above 40, under 110 F
//      3. Prompt user to put the probe in the ice bath
//      4. Display temperatures every second in a loop,
//         which stops after 5 minutes, or when the temperature reaches
//         35 degrees F or so.
//      5. Instruct user to remove probe from the bath
//      6. Display temperatures every second in a loop,
//         which stops after 5 minutes, or when the temperature reaches
//         60 degrees F or so.// FIXME: figure out what to do about this...
//
// TODO: Stub off hardware access by creating abstract implementations of Trait(s) and
//       create minimal Mock unit tests to validate basic abstract operations.
//       This implementation shall be available only under a "mock" feature.
//
//  Requirements for Traits
//      1. All traits must use RTDError as their error class
//      2. Must include an SPI abstraction and a Pin abstraction
//      3. Pin abstraction must implement raise and lower APIs, and include raise and lower APIs,
//         and implement at least OutputPins, with the ability create them with pullups or pulldowns
//         Creation must check for range of pin number.
//      4. SPI abstraction must implement transfer
//          (pub fn new(cs_pin: u8, leads: RTDLeads, filter: FilterHz) -> Result<Self, RtdError>)
//      5. SPI constructor/new must take current SPI parameters
//      6. SPI abstraction must implement transfer API
//      7. Traits shall have zero effect on top level (public) API
//      8. Traits shall not change interactions with real hardware.
//         Do not "improve" the real hardware interactions. That code is well-proven.
//         This should be an "of course" kind of thing.
//
// TODO: Create mock implementation of SPI and Pin abstractions
//
//      1. switching between mock and real APIs shall be controlled by a feature called "mock".
//         Without the mock feature, the hardware implementation of the traits shall be used
//         and with it enabled, the mock version shall be used.
//      2. The mock implementation of SPI transfers shall assume transfer is to known
//         MAX31865 registers and verify correct interaction with the mocked hardware
//         by callers.
//      4. Minimal mock tests to verify basic mocked calls don't fail shall be included.
//         See also next to-do item. These tests are to validate the mock implementation.
//      5. "Real" hardware shall not be available when "mock" feature is selected.
//      5. Mock hardware shall not change interactions with real hardware at all.
//         This should be an "of-course" kind of thing
//
// TODO: Create "mock" hardware tests which exercise and test the APIs with mock feature.
//       The purpose of this is to test our normal interactions with the hardware and
//       also exercise error legs that are impossible to create automatically in real hardware.
//
//      1. Mocked hardware tests shall exercise underlying hardware and create error
//         situations which are difficult to create in real hardware
//      2. Mock test only API calls shall be created which inject faults and control
//         hardware for the benefit of mock tests. Ability to inject faults and control
//         contents of hardware registers will be added as needed for tests.
//
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity, SpiBus};
extern crate alloc;

#[cfg(feature = "doc")]
pub mod examples;

pub mod temp_conversion;
pub mod rtd_reader;
// Re-export RTDReader at root for flat imports (agreed API consistency)
pub use rtd_reader::RTDReader;
// Private module for low-level driver (opaque to users)
mod private;

// Public enums and helpers (crate-level)
#[derive(Debug, Clone, Copy)]
/// RTD lead configurations supported by the MAX31865.
pub enum RTDLeads {
    Two = 2,
    Three = 3,
    Four = 4,
}

#[derive(Debug, Clone, Copy)]
/// Noise filter settings based on mains frequency.
pub enum FilterHz {
    /// 50 Hz filter (updates ~16 Hz).
    Fifty = 1,
    /// 60 Hz filter (updates ~19 Hz).
    Sixty = 0,
}

#[derive(Debug)]
/// An enumeration of all the different faults the API can report back.
pub enum RtdError {
    InvalidChipSelect,	// The chip select lead given is out of range
    Init(String),	// Initialization failed
    Read(String),	// Reading or writing the SPI bus failed
    Fault(u8),		// An error was reported by the MAX31865
}

#[derive(Debug, Clone, Copy)]
/// All the errors the MAX31865 can report to us.
pub enum MaxFault {
    RtdInMinusUndervoltage,    // Bit 0: RTDIN- undervoltage
    RtdInPlusOvervoltage,      // Bit 1: RTDIN+ overvoltage
    RtdInMinusOvervoltage,     // Bit 2: RTDIN- overvoltage
    RtdInPlusOpen,             // Bit 3: RTDIN+ open circuit
    RtdInMinusOpen,            // Bit 4: RTDIN- open circuit
    RtdUnderOrOvertemp,        // Bit 5: RTD under/over temperature
    RtdOverOrUnderBiasVoltage, // Bit 6: RTD over/under bias voltage
    AutoConversionFault,       // Bit 7: Auto-conversion fault
}

impl MaxFault {
    /// Returns the bitmask (u8) for this MAX31865 fault type.
    pub fn bit(self) -> u8 {
        match self {
            MaxFault::RtdInMinusUndervoltage => 0b00000001,
            MaxFault::RtdInPlusOvervoltage => 0b00000010,
            MaxFault::RtdInMinusOvervoltage => 0b00000100,
            MaxFault::RtdInPlusOpen => 0b00001000,
            MaxFault::RtdInMinusOpen => 0b00010000,
            MaxFault::RtdUnderOrOvertemp => 0b00100000,
            MaxFault::RtdOverOrUnderBiasVoltage => 0b01000000,
            MaxFault::AutoConversionFault => 0b10000000,
        }
    }

    /// Returns a human-readable description for this MAX31865 fault.
    pub fn description(self) -> &'static str {
        match self {
            MaxFault::RtdInMinusUndervoltage => "RTD IN- Undervoltage",
            MaxFault::RtdInPlusOvervoltage => "RTD IN+ Overvoltage",
            MaxFault::RtdInMinusOvervoltage => "RTD IN- Overvoltage",
            MaxFault::RtdInPlusOpen => "RTD IN+ Open Circuit",
            MaxFault::RtdInMinusOpen => "RTD IN- Open Circuit",
            MaxFault::RtdUnderOrOvertemp => "RTD Under/Over Temperature",
            MaxFault::RtdOverOrUnderBiasVoltage => "RTD Over/Under Bias Voltage",
            MaxFault::AutoConversionFault => "Auto-Conversion Fault",
        }
    }
}

/// Public helper to decode a full fault status byte into a list of active faults (for users).
/// Returns a Vec of descriptions for set bits; empty if no faults.
pub fn decode_fault_status(status: u8) -> Vec<&'static str> {
    let mut faults = Vec::new();
    let all_faults = [
        (MaxFault::RtdInMinusUndervoltage, 0b00000001),
        (MaxFault::RtdInPlusOvervoltage, 0b00000010),
        (MaxFault::RtdInMinusOvervoltage, 0b00000100),
        (MaxFault::RtdInPlusOpen, 0b00001000),
        (MaxFault::RtdInMinusOpen, 0b00010000),
        (MaxFault::RtdUnderOrOvertemp, 0b00100000),
        (MaxFault::RtdOverOrUnderBiasVoltage, 0b01000000),
        (MaxFault::AutoConversionFault, 0b10000000),
    ];
    for (fault, bit) in all_faults {
        if status & bit != 0 {
            faults.push(fault.description());
        }
    }
    faults
}

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};
