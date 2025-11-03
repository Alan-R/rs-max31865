# max31865

An easy-to-use driver for the MAX31865 RTD to Digital converter

## [Documentation](https://github.com/Alan-R/simple-max31865)

## What works

- reading temperatures in and resistance - either as f64 or scaled integers
- configuring chip select pin, RTD type and filter frequency
- setting the resistance calibration value
- Fault detection and recovery
- Extensive hardware-level mocking allowing driver testing without hardware
- Support for Raspberry Pi
- Tested with hardware from Playing With Fusion + PT100 sensor.

## TODO

- [ ] Ability to configure the SPI bus
- [ ] Compatibility with non-Raspberry Pi platforms
- [ ] Add more configuration options as needed.
- [ ] Write examples

## Quick Start

### Raspberry Pi OS Configuration

- enable GPIO/SPI via raspi-config or your OS settings before running.

### Add to `Cargo.toml`

    \[dependencies]\
    simple-max31865 = "0.1"

### Basic Usage (Raspberry Pi)

```rust
use simple_max31865::{RTDReader, RTDLeads, FilterHz};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut max = RTDReader::new(24, RTDLeads::Three, FilterHz::Sixty)?; // CS pin 24, 3-wire PT100, 60Hz filter
    let temperature = max.get_temperature()?;
    println!("Temperature: {:.2}°C", temperature);
    let resistance = max.get_resistance()?;
    println!("Resistance: {:.2} Ω", resistance);
    Ok(())
}
```

### Features

- `mock`: Enables hardware emulation for unit testing without real hardware (SPI/GPIO/MAX31865 chip).
  Supports data/fault injection for error coverage.
- `no_fp`: Disables floating-point APIs (e.g., returns scaled integers for temperature/resistance).

## Examples

Full examples (e.g., basic reading, fault handling) will be added to the examples/ directory soon.
The Quick Start above demonstrates core usage.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
  at your option.

# References

- [MAX31865 Datasheet](https://www.playingwithfusion.com/files/max31865.pdf)
- [PT100 Sensor wiring diagrams](https://www.playingwithfusion.com/docs/1203) - a
  great reference on wiring the various types of PT100 sensors.

# Credits and License

The simple-max31865 crate is derived from version 1.0.1 of
the [rs-max31865](https://github.com/emeric-martineau/rs-max31865)
crate by Rudi Horn and Emeric Martineau, with significant modifications:

- Greatly simplified, opaque API hiding hardware details.
- Added Raspberry Pi support via rppal.
- Built-in mocking for hardware-free unit testing.
- Optional floating-point temperature and resistance reads.

Original code snippets (e.g., register configs) and temperature conversion etc
are reused under the MIT/Apache-2.0 license. See the original repo for their contributions.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.

