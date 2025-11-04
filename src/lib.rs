//! Opaque RTD driver for MAX31865 on Raspberry Pi.
//! Hides all hardware details: user passes CS pin num + leads + filter Hz, gets temps in hundredths °C.
//! We use traits to hide the hardware away in support of mocking
//!
//! # Features
//! - `mock`: Enables mock hardware for testing (stubs rppal interfaces; exercises same driver logic).
//! - `no_fp`: Disables API calls that use floating point.

use std::any::Any;
use std::error::Error as StdError;
use std::fmt;
#[cfg(not(feature = "mock"))]
use rppal::spi::{Bus, Mode, SlaveSelect, Error as SpiError};
#[cfg(not(feature = "mock"))]
use rppal::gpio::Gpio;
#[cfg(feature = "mock")]
use std::sync::Arc;
#[cfg(feature = "mock")]
use std::cell::RefCell;

// Public opaque API - unchanged
pub struct RtdReader {
    inner: RtdInner,
    }

#[derive(Debug, Clone, Copy)]
pub enum RTDLeads {
    Two = 2,
    Three = 3,
    Four = 4,
    }

#[derive(Debug, Clone, Copy)]
pub enum FilterHz {
    Fifty = 0,
    Sixty = 1,
}

#[derive(Debug, Clone, Copy)]
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
    /// Returns the bitmask (u8) for this fault type.
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

    /// Returns a human-readable description for this fault.
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

#[derive(Debug)]
pub enum RtdError {
    InvalidLeads,
    InvalidFilter,
    InvalidChipSelect,
    Init(String),
    Read(String),
    Fault(u8),
}
pub trait ErrorOrAny: std::error::Error + Any {}
impl ErrorOrAny for RtdError {}

impl fmt::Display for RtdError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            RtdError::InvalidLeads => write!(f, "Invalid lead count"),
            RtdError::InvalidFilter => write!(f, "Invalid filter frequency"),
            RtdError::InvalidChipSelect => write!(f, "Invalid chip select pin"),
            RtdError::Init(s) => write!(f, "Init error: {}", s),
            RtdError::Read(s) => write!(f, "Read error: {}", s),
            RtdError::Fault(status) => {
                let descriptions = decode_fault_status(*status);
                if descriptions.is_empty() {
                    write!(f, "Fault status: {:#010b} (no active faults)", status)
                } else {
                    write!(f, "Fault status: {:#010b} ({})", status, descriptions.join(", "))
                }
            }
        }
    }
}

impl StdError for RtdError {}

// Internal opaque holder - same for real/mock
struct RtdInner {
    spi: Box<dyn SpiBusTrait>,      // The SPI bus we're active on
    cs: Box<dyn OutputPinTrait>,    // Chip select - active when low
    calib: u32,
}


impl RtdReader {
    /// Construct with CS GPIO number (BCM), lead count, and filter frequency.
    /// Defaults: continuous mode, 400Ω ref resistor.
    /// Fault pin fixed to GPIO25 (pullup input).
    pub fn new(cs_pin: u8, leads: RTDLeads, filter: FilterHz) -> Result<Self, Box<dyn StdError + Send + Sync>> {
        let lead_num = leads as u8;
        if !(2..=4).contains(&lead_num) {
            return Err(Box::new(RtdError::InvalidLeads) as Box<dyn StdError + Send + Sync>);
        }
        let filter_bit = filter as u8;
        if filter_bit > 1 {
            return Err(Box::new(RtdError::InvalidFilter) as Box<dyn StdError + Send + Sync>);
        }
        if cs_pin == 0 || cs_pin > 27 {
            return Err(Box::new(RtdError::Init("Invalid chip select pin number".to_string())) as Box<dyn StdError + Send + Sync>);
        }

        #[cfg(not(feature = "mock"))]
        {
            // TODO: Understand if we need to provide parameters for these options
            // Right now, I love the simplicity of the interface, but they might be needed.
            let spi_result = RealSpi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode3)
                .map_err(|e| Box::new(RtdError::Init(e.to_string())) as Box<dyn StdError + Send + Sync>)?;
            let spi = Box::new(spi_result) as Box<dyn SpiBusTrait>;  // Now this works—RealSpi impls the trait
            let gpio = Gpio::new().map_err(|e| Box::new(RtdError::Init(e.to_string())) as Box<dyn StdError + Send + Sync>)?;
            let cs = Box::new(
                gpio.get(cs_pin)
                    .map_err(|e| Box::new(RtdError::Init(e.to_string())) as Box<dyn StdError + Send + Sync>)?
                    .into_output_high()
            ) as Box<dyn OutputPinTrait>;

            let mut inner = RtdInner { spi, cs, calib: 40000 };
            inner.init(lead_num, filter_bit).map_err(|e| Box::new(e) as Box<dyn StdError + Send + Sync>)?;
            Ok(Self { inner })
        }

        #[cfg(feature = "mock")]
        {
            let chip_state = Arc::new(RefCell::new(MockChipState::default()));
            let spi = Box::new(MockSpiBus {
                state: chip_state.clone(),
                fail_transfer: false,
            }) as Box<dyn SpiBusTrait>;
            let cs = Box::new(MockCsPin::default()) as Box<dyn OutputPinTrait>;

            let mut inner = RtdInner { spi, cs, calib: 40000 };
            inner.init(lead_num, filter_bit).map_err(|e| Box::new(e) as Box<dyn StdError + Send + Sync>)?;
            Ok(Self { inner })
        }
    }

    /// Read temperature in hundredths of °C (e.g., 2150 = 21.50°C).
    /// Integer math only; auto-clears faults on error.
    pub fn read_temp_100(&mut self) -> Result<i32, Box<dyn StdError + Send + Sync>> {
        self.inner.read_temp_100()
    }

    /// Update reference resistor calibration (ohms * 100, default 40000).
    pub fn set_calib(&mut self, calib: u32) {
        self.inner.calib = calib;
    }

    /// Returns true if the error is a MAX31865 chip fault (e.g., open circuit, overtemp).
    /// Use this to decide if you need to call `read_fault_status()` to clear and inspect.
    pub fn is_max_fault(&self, e: &dyn ErrorOrAny) -> bool {
        (e as &dyn Any).downcast_ref::<RtdError>().map_or(false, |err| matches!(err, RtdError::Fault(_)))
    }

    /// Read the MAX31865 fault status register (register 7).
    /// Returns the raw u8 status byte; use `decode_fault_status` for human-readable descriptions.
    /// Does not auto-clear faults—call `clear_fault` if needed after inspection.
    pub fn read_fault_status(&mut self) -> Result<u8, Box<dyn StdError + Send + Sync>> {
        self.inner.read_reg(7).map_err(|e| Box::new(e) as Box<dyn StdError + Send + Sync>)
    }

    /// Clear any active faults on the MAX31865 (writes to config register bits 6:4).
    /// Call this after inspecting faults via `read_fault_status` to resume normal operation.
    pub fn clear_fault(&mut self) -> Result<(), Box<dyn StdError + Send + Sync>> {
        self.inner.clear_fault().map_err(|e| Box::new(e) as Box<dyn StdError + Send + Sync>)
    }

    #[cfg(not(feature = "no_fp"))]
    pub fn get_temperature(&mut self) -> Result<f64, Box<dyn StdError + Send + Sync>> {
        let temp100 = self.inner.read_temp_100()?;
        Ok(temp100 as f64 / 100.0) // Convert
    }

    #[cfg(not(feature = "no_fp"))]
    pub fn get_resistance(&mut self) -> Result<f64, Box<dyn StdError + Send + Sync>> {
        let ohms100 = self.inner.read_ohms_100()?;
        Ok(ohms100 as f64 / 100.0) // Convert
    }

    #[cfg(feature = "mock")]
    /// Inject raw RTD value for mock (MSB/LSB; auto-injects LSB fault if bit0 set).
    pub fn inject_rtd_raw(&mut self, raw: u16) {
        if let Some(mock_spi) = self.inner.spi.as_any_mut().downcast_mut::<MockSpiBus>() {
            mock_spi.state.borrow_mut().set_rtd_raw(raw);
        }
    }

    #[cfg(feature = "mock")]
    /// Inject fault bits into status reg (persists until cleared).
    pub fn inject_fault(&mut self, bits: u8) {
        if let Some(mock_spi) = self.inner.spi.as_any_mut().downcast_mut::<MockSpiBus>() {
            mock_spi.state.borrow_mut().inject_fault(bits);
        }
    }

    #[cfg(feature = "mock")]
    /// Get current fault status for verification.
    pub fn get_fault_status(&self) -> u8 {
        if let Some(mock_spi) = self.inner.spi.as_any().downcast_ref::<MockSpiBus>() {
            mock_spi.state.borrow().regs[7]
        } else {
            0
        }
    }
    #[cfg(feature = "mock")]
    /// Set up next transfer to fail
    pub fn set_fail_transfer(&mut self) {
        if let Some(mock_spi) = self.inner.spi.as_any_mut().downcast_mut::<MockSpiBus>() {
            mock_spi.set_fail_transfer();
        }
    }

}

// Shared logic - exact same for real/mock (uses traits)
impl RtdInner {
    fn init(&mut self, leads: u8, filter: u8) -> Result<(), RtdError> {
        self.cs.set_high();

        // Config: bias on (7), auto-conversion on (6), one-shot off (5), leads (4), filter (0)
        let mut conf: u8 = 0xC0;  // Bits 7-6 on
        if leads == 3 { conf |= 0x10; }  // Bit 4 for 3-wire
        conf |= filter;  // Bit 0 for 50/60Hz
        self.write_reg(0x00, conf).map_err(|_| RtdError::Init("Config write failed".to_string()))?;

        self.clear_fault().map_err(|_| RtdError::Init("Clear fault failed".to_string()))?;
        Ok(())
    }

    fn read_ohms_100(&mut self) -> Result<u32, Box<dyn StdError + Send + Sync>> {
        // Read RTD raw: 3-byte transfer for 2-byte data (addr + dummies)
        self.cs.set_low();
        let write_buf = [0x01 | 0x80, 0x00, 0x00];  // Write: Read RTD_MSB addr + dummies
        let mut read_buf = [0x00, 0x00, 0x00];      // Read: Will be filled with dummy + MSB + LSB
        let transferred = self.spi.transfer(&mut read_buf, &write_buf)
            .map_err(|e| Box::new(RtdError::Read(format!("Transfer failed: {}", e))) as Box<dyn StdError + Send + Sync>)?;
        self.cs.set_high();

        if transferred != write_buf.len() {
            return Err(Box::new(RtdError::Read(format!(
                "Incomplete transfer: {} bytes (expected 3)", transferred
            ))) as Box<dyn StdError + Send + Sync>);
        }

        let raw = u16::from_be_bytes([read_buf[1], read_buf[2]]);  // MSB in [1], LSB in [2]
        if raw & 1 != 0 {
            // LSB indicates a fault; read full status from reg 7 for complete details
            match self.read_reg(7) {
                Ok(full_status) => {
                    Err(Box::new(RtdError::Fault(full_status)) as Box<dyn StdError + Send + Sync>)
                }
                Err(e) => {
                    // Fallback if status read fails (e is already RtdError)
                    Err(Box::new(e) as Box<dyn StdError + Send + Sync>)
                }
            }
        } else {
            // No fault: compute resistance (note: low-order bit is error bit, so shift right)
            let ohms100 = (((raw >> 1) as u32 * self.calib) + (1u32 << 13)) >> 14;  // Round to nearest
            Ok(ohms100)
        }
    }

    fn read_temp_100(&mut self) -> Result<i32, Box<dyn StdError + Send + Sync>> {
        // Simple integer PT100 approximation: T = 100 * (R/R0 - 1) / 0.00385
        // Scaled to avoid floats: R0=10000 (100*100), alpha=385 (0.00385*100000)
        let ohms100 = self.read_ohms_100()?; // Resistance in Ohms * 100
        let r = ohms100 as i64;
        let r0 = 10000i64;
        let alpha = 385i64;
        let num = (r * 100000) - (r0 * 100000);
        let den = alpha * r0;
        let temp = (num * 100) / den;  // Hundredths °C
        Ok(temp as i32)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, RtdError> {
        self.cs.set_low();
        let write_buf = [reg | 0x80, 0x00];  // Write: Read addr + dummy
        let mut read_buf = [0x00, 0x00];     // Read: Will be filled with dummy + value
        let transferred = self.spi.transfer(&mut read_buf, &write_buf)
            .map_err(|_| RtdError::Read("SPI transfer failed".to_string()))?;
        self.cs.set_high();
        if transferred != 2 {
            return Err(RtdError::Read(format!("Incomplete register read transfer: {} bytes (expected 2)", transferred)));
        }
        Ok(read_buf[1])
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), RtdError> {
        self.cs.set_low();
        let transferred = self.spi.write(&[reg, val])
            .map_err(|_| RtdError::Read("SPI write failed".to_string()))?;
        self.cs.set_high();
        if transferred != 2 {
            return Err(RtdError::Read(format!("Incomplete register write transfer: {} bytes (expected 2)", transferred)));
        }
        Ok(())
    }

    fn clear_fault(&mut self) -> Result<(), RtdError> {
        let mut conf = self.read_reg(0x00)?;
        conf |= 0x70;  // Bits 6:4 = 111 to clear faults
        self.write_reg(0x00, conf)
    }
}

// Traits require as_any/as_any_mut since SpiBusTrait: Any (for downcasting in mocks)
#[allow(dead_code)]  // Suppress dead_code warning for non-mock builds (methods used only in mock)
trait SpiBusTrait: Any {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    fn transfer(&mut self, read_buf: &mut [u8], write_buf: &[u8]) -> Result<usize, RtdError>;
    fn write(&mut self, words: &[u8]) -> Result<usize, RtdError>;
    #[cfg(feature = "mock")]
    fn set_fail_transfer(&mut self);
}

trait OutputPinTrait {
    fn set_high(&mut self) -> ();
    fn set_low(&mut self) -> ();
}

// Real rppal impls (gated by not(mock))
#[cfg(not(feature = "mock"))]
#[derive(Debug)]
struct RealSpi {
    inner: rppal::spi::Spi,
}

#[cfg(not(feature = "mock"))]
impl RealSpi {
    fn new(bus: Bus, slave: SlaveSelect, freq: u32, mode: Mode) -> Result<Self, SpiError> {
        rppal::spi::Spi::new(bus, slave, freq, mode).map(|inner| Self { inner })
    }
}

#[cfg(not(feature = "mock"))]
impl SpiBusTrait for RealSpi {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }

    fn transfer(&mut self, read_buf: &mut [u8], write_buf: &[u8]) -> Result<usize, RtdError> {
        // Test-only: Force failure for incomplete transfer coverage
        self.inner.transfer(read_buf, write_buf)
            .map_err(|e| RtdError::Read(e.to_string()))
    }

    fn write(&mut self, words: &[u8]) -> Result<usize, RtdError> {
        self.inner.write(words)
            .map_err(|e| RtdError::Read(e.to_string()))
    }
}

#[cfg(not(feature = "mock"))]
impl OutputPinTrait for rppal::gpio::OutputPin {
    fn set_high(&mut self) {
        let _ = self.set_high();
    }

    fn set_low(&mut self) {
        let _ = self.set_low();
    }
}

/***********************************************************************************
 *  Code from here to the end is just for mocking the behavior of the GPIO bus
 *  and the Max31865 temperature sensing chip.
 *  This is so we can test the behavior of the driver in the absence of hardware on any
 *  Rust platform.
 **********************************************************************************/

// Mock hardware (gated by "mock" feature)
#[cfg(feature = "mock")]
#[derive(Default, Debug)]
struct MockChipState {
    regs: [u8; 8],  // 0: config, 1: RTD_MSB, 2: RTD_LSB, 7: fault_status
    fault_pin_high: bool,
}

#[cfg(feature = "mock")]
impl MockChipState {
    fn set_rtd_raw(&mut self, raw: u16) {
        self.regs[1] = (raw >> 8) as u8;
        self.regs[2] = raw as u8;
        if raw & 1 != 0 {
            self.regs[7] |= 1;
            self.fault_pin_high = true;
        }
        }

    fn inject_fault(&mut self, bits: u8) {
        self.regs[7] |= bits;
        self.fault_pin_high = self.regs[7] != 0;
    }
    }

#[cfg(feature = "mock")]
#[derive(Debug, Default)]
struct MockSpiBus {
    state: Arc<RefCell<MockChipState>>,
    #[cfg(feature = "mock")]
    fail_transfer: bool,  // Test-only: Force incomplete transfer
}

#[cfg(feature = "mock")]
impl SpiBusTrait for MockSpiBus {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }

    fn set_fail_transfer(&mut self) {
        self.fail_transfer = true;
    }

    fn transfer(&mut self, read_buf: &mut [u8], write_buf: &[u8]) -> Result<usize, RtdError> {
        let len = std::cmp::min(read_buf.len(), write_buf.len());
        if len == 0 {
            return Ok(0);
        }
        if self.fail_transfer {
            self.fail_transfer = false;
            return Ok(254)
        }

        // Process from write_buf (commands/data), fill read_buf (responses/dummies)
        let addr = write_buf[0];
        let reg = addr & 0x7F;
        let is_read = addr & 0x80 != 0;
        let mut st = self.state.borrow_mut();

        // Fill read_buf[0] with dummy (always, as per SPI protocol)
        read_buf[0] = 0x00;

        if !is_read {
            // Write: Update state from write_buf[1...], fill read_buf[1..] with dummies
            if len >= 2 && reg < 8 {
                st.regs[reg as usize] = write_buf[1];
                if reg == 0 && (write_buf[1] & 0x70) == 0x70 {
                    st.regs[7] = 0;
                    st.fault_pin_high = false;
                }
            }
            for i in 1..len {
                read_buf[i] = 0x00;  // Dummy response for writes
            }
        } else {
            // Read: Fill read_buf[1..] from state
            if len >= 2 && reg < 8 {
                read_buf[1] = st.regs[reg as usize];
            }
            if reg == 1 && len >= 3 {
                read_buf[1] = st.regs[1];
                read_buf[2] = st.regs[2];
                // Force LSB D0=1 on RTD reads if any fault active (per datasheet: faults invalidate conversion)
                if st.regs[7] != 0 {
                    read_buf[2] |= 1;
                }
            } else if len > 2 {
                // Extra dummies for longer reads
                for i in 2..len {
                    read_buf[i] = 0x00;
                }
            }
        }
        Ok(len)
    }

    fn write(&mut self, words: &[u8]) -> Result<usize, RtdError> {
        // Simulate write: transfer with dummy read_buf (zeros, discarded)
        let len = words.len();
        let mut dummy_read = vec![0u8; len];
        let transferred = self.transfer(&mut dummy_read, words)?;
        if transferred != len {
            return Err(RtdError::Read(format!(
                "Incomplete transfer: {} bytes (expected {})", transferred, len
            )))
        }
        Ok(transferred)
    }
}

#[cfg(feature = "mock")]
#[derive(Default, Debug)]
struct MockCsPin {
    high: bool,
}

#[cfg(feature = "mock")]
impl OutputPinTrait for MockCsPin {
    fn set_high(&mut self) {
        self.high = true;
    }

    fn set_low(&mut self) {
        self.high = false;
    }
}

