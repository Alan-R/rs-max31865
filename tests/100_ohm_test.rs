#[cfg(test)]
#[cfg_attr(
    not(any(target_arch = "arm", target_arch = "aarch64")),
    allow(unused_imports, unused_variables, unused_assignments, dead_code)
)]
mod tests {
    use std::env;
    use simple_max31865::{RtdReader, RTDLeads, FilterHz, decode_fault_status};
    /// These tests are usable ONLY when connected to real hardware with a 100 ohm resistor
    /// connected in place of a PT100 sensor like the ASCII art shows.
    /// The good news is that resistors can be tested,
    /// and they basically don't fail - although you *can* screw up the wiring ;-).

    const NOMINAL_OHMS: f64 = 100.0;

    /// Get tolerance % from env (e.g., 5.0 for 5%; default 5.0 for common 1-5% resistors).
    /// Returns fraction (e.g., 0.05) for math use.
    fn get_tolerance_fraction() -> f64 {
        let pct_str = env::var("RESISTOR_TOLERANCE_PCT").unwrap_or_else(|_| "5.0".to_string());
        match pct_str.parse::<f64>() {
            Ok(pct) => pct / 100.0,
            Err(_) => {
                eprintln!("Warning: Invalid RESISTOR_TOLERANCE_PCT='{}'; defaulting to 5.0%", pct_str);
                0.05  // 5% fraction
            }
        }
    }

    fn min_ohms() -> f64 {
        let tolerance = get_tolerance_fraction();
        NOMINAL_OHMS * (1.0 - tolerance)
    }

    fn max_ohms() -> f64 {
        let tolerance = get_tolerance_fraction();
        NOMINAL_OHMS * (1.0 + tolerance)
    }

    /// PT100 linear approximation: T = (R / 100 - 1) / 0.00385 (valid 0–300°C, per datasheet).
    const fn pt100_temperature_from_ohms(ohms: f64) -> f64 {
        (ohms / 100.0 - 1.0) / 0.00385
    }

    fn min_temp_c() -> f64 {
        pt100_temperature_from_ohms(min_ohms())
    }

    fn max_temp_c() -> f64 {
        pt100_temperature_from_ohms(max_ohms())
    }

    fn min_temp_100() -> i32 {
        (min_temp_c() * 100.0) as i32
    }

    fn max_temp_100() -> i32 {
        (max_temp_c() * 100.0) as i32
    }

    fn tolerance_pct() -> f64 {
        get_tolerance_fraction() * 100.0
    }

    /// Get CS pin from env or default to 8 (common for MAX31865 on RPi).
    fn get_cs_pin() -> u8 {
        env::var("CS_PIN")
            .ok()
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(8)
    }

    /// Wiring diagram for 100Ω resistor test (2-wire mode; set RTDLeads::Two in setup).
    /// [Your annotations here—e.g., emphasize CS_PIN matching, no FAULT/DRDY wiring needed.]
    /// Remove any jumper on breakout for open-circuit fault detection.
    /// For 3-wire: Add matching 100Ω from Force+ to RTDIN+ (see datasheet Fig 20).
    /// For 4-wire: Add sense wires from RTDIN+/RTDIN- to RTD ends.
    ///
    /// ASCII Art (Typical MAX31865 Breakout Board):
    ///
    ///   RPi GPIO          MAX31865 Pins
    ///   ----------------  ----------------
    ///   3.3V     -->      VDD
    ///   GND      -->      GND
    ///   GPIO 8   -->      CS (Chip Select) - MUST MATCH ENV VAR CS_PIN
    ///   GPIO 10  -->      DIN (MOSI)
    ///   GPIO 9   -->      DOUT (MISO)
    ///   GPIO 11  -->      SCLK
    ///   (No FAULT/DRDY wiring needed; driver polls status)
    ///
    ///   Test Wiring (2-Wire, 100Ω Resistor - Full 4-Pin Setup):
    ///
    ///             +3.3V
    ///                |
    ///                v (Excitation)
    ///   MAX Force+ ---[100Ω Resistor (RTD simulation)]--- MAX Force- --- GND
    ///                |                                      |
    ///                | (Sense jumper/wire on breakout)      |
    ///                v                                      v
    ///   MAX RTDIN+ ----------------------------------------- MAX RTDIN-
    ///
    ///   Notes:
    ///   - Resistor between Force+/Force- (~0°C PT100 sim). Jumper RTDIN to Force for 2-wire sense.
    ///   - For open fault: Disconnect resistor (expect bits 3/4 in status).
    ///   - Use 0.1-1% 400/430Ω ref resistor (set_calib(40000/43000)).
    ///   - Power cycle after wiring. Run: RESISTOR_TOLERANCE_PCT=5 CS_PIN=8 cargo test -- --ignored test_get_resistance_hardware
    ///   - Expected: ~100Ω / ~0°C within tolerance % (printed in output).

    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn setup_driver(leads: RTDLeads, filter: FilterHz) -> RtdReader {
        let cs_pin = get_cs_pin();
        RtdReader::new(cs_pin, leads, filter).unwrap()
    }

    #[test]
    fn test_env_pin_parsing() {
        // Simulate env var
        env::set_var("CS_PIN", "23");

        let cs_pin = get_cs_pin();
        assert_eq!(cs_pin, 23, "Should parse CS_PIN=23 from env");

        // Reset
        env::remove_var("CS_PIN");

        // Fallback
        let default_cs = get_cs_pin();
        assert_eq!(default_cs, 8, "Should fallback to 8 if unset");
    }

    #[test]
    fn test_tolerance_env_parsing() {
        // Test % parsing to fraction (epsilon for f64 precision)
        let epsilon = 1e-10;
        env::set_var("RESISTOR_TOLERANCE_PCT", "1.1");
        let frac = get_tolerance_fraction();
        assert!((frac - 0.011).abs() < epsilon, "1.1% → ~0.011 fraction (got {})", frac);

        env::set_var("RESISTOR_TOLERANCE_PCT", "20");
        let frac = get_tolerance_fraction();
        assert!((frac - 0.20).abs() < epsilon, "20% → ~0.20 fraction (got {})", frac);

        // Invalid fallback
        env::set_var("RESISTOR_TOLERANCE_PCT", "abc");
        let frac = get_tolerance_fraction();
        assert!((frac - 0.05).abs() < epsilon, "Invalid → default ~0.05 (5%) (got {})", frac);

        env::remove_var("RESISTOR_TOLERANCE_PCT");
        let frac = get_tolerance_fraction();
        assert!((frac - 0.05).abs() < epsilon, "Unset → default ~0.05 (5%) (got {})", frac);
    }

    #[test]
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_invalid_params() {
        // new() validates leads (2-4) and filter (0-1)
        let valid = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty);
        assert!(valid.is_ok(), "Valid params should succeed");

        // Invalid CS pin (e.g., >31 for RPi; fails GPIO init)
        let invalid_cs = RtdReader::new(99, RTDLeads::Two, FilterHz::Fifty);
        assert!(invalid_cs.is_err(), "Invalid CS should fail");
        if let Err(e) = invalid_cs {
            assert!(format!("{}", e).contains("Init"), "Should be Init error from GPIO");
        }
    }

    #[test]
    #[ignore] // Manual: Wire 100Ω as per diagram (2-wire)
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_read_temp_100_hardware() {
        let mut reader = setup_driver(RTDLeads::Two, FilterHz::Fifty);

        let temp100 = reader.read_temp_100().unwrap();
        let pct = tolerance_pct();
        println!("Temp (hundredths °C): {} (tolerance: {}%)", temp100, pct);
        assert!(temp100 >= min_temp_100() && temp100 <= max_temp_100(),
                "Temp100 {} should be {} to {} (±{}% for 100Ω)", temp100, min_temp_100(), max_temp_100(), pct);
    }

    #[test]
    #[ignore] // Manual: Wire 100Ω as per diagram (2-wire)
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_get_resistance_hardware() {
        let mut reader = setup_driver(RTDLeads::Two, FilterHz::Fifty);

        let resistance = reader.get_resistance().unwrap();
        let pct = tolerance_pct();
        println!("Resistance (Ω f64): {} (tolerance: {}%)", resistance, pct);
        assert!(resistance >= min_ohms() && resistance <= max_ohms(),
                "Resistance {} should be {:.1} to {:.1} (±{}%)", resistance, min_ohms(), max_ohms(), pct);
    }

    #[test]
    #[ignore] // Manual: Wire 100Ω as per diagram (2-wire)
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_get_temperature_hardware() {
        let mut reader = setup_driver(RTDLeads::Two, FilterHz::Fifty);

        let temperature = reader.get_temperature().unwrap();
        let pct = tolerance_pct();
        println!("Temperature (°C f64): {} (tolerance: {}%)", temperature, pct);
        assert!(temperature >= min_temp_c() && temperature <= max_temp_c(),
                "Temperature {} should be {:.1} to {:.1} (±{}%)", temperature, min_temp_c(), max_temp_c(), pct);
    }

    #[test]
    #[ignore] // Manual: Wire 100Ω as per diagram (2-wire; expect no fault)
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_read_fault_status_hardware() {
        let mut reader = setup_driver(RTDLeads::Two, FilterHz::Fifty);

        let status = reader.read_fault_status().unwrap();
        println!("Fault status: 0x{:02X}", status);
        assert_eq!(status, 0, "Should be 0x00 (no fault with wired resistor)");
        assert!(decode_fault_status(status).is_empty(), "No active faults");
    }

    #[test]
    #[ignore] // Manual: Wire 100Ω as per diagram (2-wire; clear no-op)
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    fn test_clear_fault_hardware() {
        let mut reader = setup_driver(RTDLeads::Two, FilterHz::Fifty);

        let status_before = reader.read_fault_status().unwrap();
        println!("Status before: 0x{:02X}", status_before);
        assert_eq!(status_before, 0, "Before should be 0x00");

        reader.clear_fault().unwrap(); // Safe no-op
        let status_after = reader.read_fault_status().unwrap();
        println!("Status after: 0x{:02X}", status_after);
        assert_eq!(status_after, 0, "After should remain 0x00");
    }
}