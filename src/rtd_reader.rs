///! Public simplified wrapper API (contains only RTDReader)
use crate::private::{Max31865, Error as InternalError};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

// use rppal::gpio::{Gpio, OutputPin as GpioOutputPin};
// use rppal::spi::{Spi, Bus, SlaveSelect, Mode as SpiMode};
use crate::{RtdError, RTDLeads, FilterHz};  // Root public enum

/// Simplified high-level interface for Raspberry Pi (continuous mode only).
/// Hides SPI/GPIO setup, RDY pin (unused), and low-level details.
/// Assumes PT100 sensor; configure with CS pin, leads, and filter.
pub struct RTDReader<SPI, NCS> 
where
    SPI: SpiBus<u8>,
    NCS: OutputPin
{
    inner: Max31865<SPI, NCS>,
}

impl<SPI, NCS> RTDReader<SPI, NCS>    
where
    SPI: SpiBus<u8>,
    NCS: OutputPin
{
    /// Create a new RTDReader (Raspberry Pi only).
    ///
    /// # Arguments
    /// * `ncs` - GPIO pin for Chip Select (NCS, active low).
    /// * `spi` - Spi bus.
    /// * `leads` - Number of wires in the RTD setup (2/3/4).
    /// * `filter` - Noise filter based on mains frequency (50/60 Hz).
    ///
    /// Configures continuous mode (vbias=true, auto-conversion=true, one-shot=false).
    /// Defaults to 400Ω calibration. RDY pin is not used (can float).
    pub fn new(ncs: NCS, spi: SPI, leads: RTDLeads, filter: FilterHz) -> Result<Self, RtdError> {
        let mut inner = Max31865::new(spi, ncs).map_err(|e| RtdError::Init(match e {
            InternalError::GpioError => "NCS pin setup failed".to_string(),
            _ => "MAX31865 init failed".to_string(),
        }))?;

        inner.configure(true, true, leads, filter)
            .map_err(|e| RtdError::Init(format!("Configure failed: {:?}", e)))?;

        Ok(RTDReader { inner })
    }

    /// Read temperature in °C as f64 (PT100 lookup).
    pub fn get_temperature(&mut self) -> Result<f64, RtdError> {
        self.inner.read_temperature().map_err(map_internal_error)
    }

    /// Read resistance in ohms as f64.
    pub fn get_resistance(&mut self) -> Result<f64, RtdError> {
        self.inner.read_resistance().map_err(map_internal_error)
    }

    /// Read temperature as scaled integer (degrees Celsius * 100).
    pub fn read_temp_100(&mut self) -> Result<i32, RtdError> {
        self.inner.read_default_conversion().map_err(map_internal_error)
    }

    /// Read resistance as scaled integer (ohms * 100).
    pub fn get_ohms_100(&mut self) -> Result<u32, RtdError> {
        self.inner.read_ohms().map_err(map_internal_error)
    }

    /// Read raw RTD value (u16, for testing/low-level).
    pub fn get_raw_data(&mut self) -> Result<u16, RtdError> {
        self.inner.read_raw().map_err(map_internal_error)
    }

    /// Check if an error is a MAX31865 fault (RtdError::Fault variant).
    pub fn is_max_fault(&self, e: &RtdError) -> bool {
        matches!(e, RtdError::Fault(_))
    }

    /// Read fault status (u8 from reg 0x07; auto-clears).
    pub fn read_fault_status(&mut self) -> Result<u8, RtdError> {
        self.inner.read_fault_status().map_err(map_internal_error)
    }

    /// Clear any latched faults (no-op if none).
    pub fn clear_fault(&mut self) -> Result<(), RtdError> {
        self.inner.clear_fault().map_err(|e| RtdError::Read(match e {
            InternalError::SpiErrorTransfer => "Clear fault SPI write failed".to_string(),
            _ => "Clear fault failed".to_string(),
        }))
    }

    /// Set calibration (ohms * 100, e.g., 40000 for 400Ω).
    pub fn set_calibration(&mut self, calib: u32) {
        self.inner.set_calibration(calib);
    }
}

/// Map internal low-level errors to public RtdError.
fn map_internal_error(e: InternalError) -> RtdError {
    match e {
        InternalError::SpiErrorTransfer | InternalError::GpioError => {
            RtdError::Read("SPI/GPIO transfer failed".to_string())
        }
        InternalError::MAXError => RtdError::Fault(0),  // Placeholder; call read_fault_status() for real status
    }
}
