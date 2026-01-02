use super::*;

#[derive(Debug)]
pub enum Error {
    /// Error transferring data to/from Max31865 chip registers
    SpiErrorTransfer,
    /// Error setting the state of a pin in the GPIO bus
    GpioError,
    /// The Max31865 chip declared an error when converting temperatures.
    /// Use `read_fault_status()` for details.
    MAXError,
}

pub struct Max31865<SPI, NCS> {
    spi: SPI,
    ncs: NCS,
    calibration: u32,
}

impl<SPI, NCS> Max31865<SPI, NCS>
where
    SPI: SpiBus<u8>,
    NCS: OutputPin,
{
    /// Create a new MAX31865 module (internal use only).
    pub fn new(spi: SPI, mut ncs: NCS) -> Result<Max31865<SPI, NCS>, Error> {
        let default_calib = 40000;

        ncs.set_high().map_err(|_| Error::GpioError)?;
        let max31865 = Max31865 {
            spi,
            ncs,
            calibration: default_calib, /* value in ohms multiplied by 100 */
        };

        Ok(max31865)
    }

    /// Updates the devices configuration (internal use only).
    pub fn configure(
        &mut self,
        vbias: bool,
        conversion_mode: bool,
        sensor_type_enum: RTDLeads,  // From public RTDLeads cast
        filter_mode_enum: FilterHz,   // From public FilterHz cast
    ) -> Result<(), Error> {

        // Compute sensor type and filter mode bits directly
        let sensor_type = match sensor_type_enum {
            RTDLeads::Three => 1u8,
            RTDLeads::Two | RTDLeads::Four => 0u8,  // Two or Four = 0
        };
        let filter_mode = match filter_mode_enum {
            FilterHz::Fifty => 1u8,  // Fifty = 1 (low order bit)
            FilterHz::Sixty => 0u8,  // Sixty = 0 (no lower order bits)
        };
        let conf: u8 = ((vbias as u8) << 7)
            | ((conversion_mode as u8) << 6)
            | (sensor_type << 4)  // Bit 4: sensor type (0 for 2/4-wire, 1 for 3-wire)
            | filter_mode;          // Bit 0: filter (0 for 60Hz, 1 for 50Hz)

        self.write(Register::CONFIG, conf)?;
        self.clear_fault()?; // Unlatch any boot faults (mimics Adafruit init)

        Ok(())
    }

    /// Clear latched faults (config reg bit 1 = 1)
    pub fn clear_fault(&mut self) -> Result<(), Error> {
        self.write(Register::CONFIG, 0x02)
    }

    /// Read and clear fault status reg (0x07) for bit-level diagnostics (u8 LSB)
    pub fn read_fault_status(&mut self) -> Result<u8, Error> {
        let status = self.read(Register::FAULT_STATUS)?;
        self.clear_fault()?; // Clear after read (if auto-clear needed)
        Ok(status)
    }

    /// Set the calibration reference resistance (internal use only).
    pub fn set_calibration(&mut self, calib: u32) {
        self.calibration = calib;
    }

    /// Read the raw resistance value.
    /// The output value is the value in Ohms multiplied by 100.
    pub fn read_ohms(&mut self) -> Result<u32, Error> {
        let raw = self.read_raw()?;
        let ohms = ((raw >> 1) as u32 * self.calibration) >> 15;
        Ok(ohms)
    }

    /// Read resistance in ohms as f64
    pub fn read_resistance(&mut self) -> Result<f64, Error> {
        let ohms_raw = self.read_ohms()?; // u32 *100;
        Ok(ohms_raw as f64 / 100.0)
    }

    /// Read temperature in °C as f64
    pub fn read_temperature(&mut self) -> Result<f64, Error> {
        let temp_raw = self.read_default_conversion()?; // i32 *100
        Ok(temp_raw as f64 / 100.0)
    }

    /// Read the raw resistance value and then perform conversion to degrees Celsius.
    /// The output value is the value in degrees Celsius multiplied by 100.
    pub fn read_default_conversion(&mut self) -> Result<i32, Error> {
        let ohms = self.read_ohms()?;
        let temp = super::temp_conversion::LOOKUP_VEC_PT100.lookup_temperature(ohms as i32);
        Ok(temp)
    }

    /// Read the raw RTD value.
    /// The raw value is the value of the combined MSB and LSB registers.
    /// The first 15 bits specify the ohmic value in relation to the reference
    /// resistor (i.e. 2^15 - 1 would be the exact same resistance as the reference
    /// resistor). See manual for further information.
    /// The last bit specifies if the conversion was successful.
    pub fn read_raw(&mut self) -> Result<u16, Error> {
        let buffer = self.read_two(Register::RTD_MSB)?; // Single read_two on MSB clocks MSB + LSB
        let raw = ((buffer[0] as u16) << 8) | (buffer[1] as u16); // buffer[0] = MSB, [1] = LSB
        if raw & 1 != 0 { // LSB bit 0 = 1 = fault during read
            return Err(Error::MAXError);
        }
        Ok(raw)
    }

    fn read(&mut self, reg: Register) -> Result<u8, Error> {
        let mut read_buffer = [0u8; 2]; // 2 bytes: dummy + data
        let mut write_buffer = [0u8; 2];
        write_buffer[0] = reg.read_address(); // Read addr for reg (e.g., 0x81 for 0x01)
        write_buffer[1] = 0; // Dummy data
        self.ncs.set_low().map_err(|_| Error::GpioError)?;
        self.spi
            .transfer(&mut read_buffer, &write_buffer)
            .map_err(|_| Error::SpiErrorTransfer)?;
        self.ncs.set_high().map_err(|_| Error::GpioError)?;
        Ok(read_buffer[1]) // Return result (ignore dummy [0])
    }

    fn read_two(&mut self, reg: Register) -> Result<[u8; 2], Error> {
        // The hardware is full duplex - you have to read and write the same number of bytes.
        // The first byte you write is the register offset, and the remaining
        // bytes are ignored when reading. To read two bytes you write three.
        // The two bytes we read are in the last two of the three bytes read.
        // NOTE: It reads and writes the minimum size of the read and write buffers
        let mut read_buffer = [0u8; 3]; // 3 bytes: dummy + MSB + LSB
        let mut write_buffer = [0u8; 3];
        write_buffer[0] = reg.read_address(); // Read addr for reg (e.g., 0x81 for 0x01)
        write_buffer[1] = 0; // Dummy for MSB
        write_buffer[2] = 0; // Dummy for LSB
        self.ncs.set_low().map_err(|_| Error::GpioError)?;
        self.spi
            .transfer(&mut read_buffer, &write_buffer)
            .map_err(|_| Error::SpiErrorTransfer)?;
        self.ncs.set_high().map_err(|_| Error::GpioError)?;
        Ok([read_buffer[1], read_buffer[2]]) // Return MSB, LSB (ignore dummy [0])
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), Error> {
        self.ncs.set_low().map_err(|_| Error::GpioError)?;
        self.spi
            .write(&[reg.write_address(), val])
            .map_err(|_| Error::SpiErrorTransfer)?;
        self.ncs.set_high().map_err(|_| Error::GpioError)?;
        Ok(())
    }
}

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Register {    // All the lovely Max31865 register offsets
    CONFIG = 0x00,
    RTD_MSB = 0x01,
    RTD_LSB = 0x02,
    HIGH_FAULT_THRESHOLD_MSB = 0x03,
    HIGH_FAULT_THRESHOLD_LSB = 0x04,
    LOW_FAULT_THRESHOLD_MSB = 0x05,
    LOW_FAULT_THRESHOLD_LSB = 0x06,
    FAULT_STATUS = 0x07,
}

const R: u8 = 0 << 7;
const W: u8 = 1 << 7;

impl Register {
    fn read_address(&self) -> u8 {
        *self as u8 | R
    }

    fn write_address(&self) -> u8 {
        *self as u8 | W
    }
}