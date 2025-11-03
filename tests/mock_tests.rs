#[cfg(test)]
#[cfg(feature = "mock")]
mod tests {
    use simple_max31865::{RtdReader, RTDLeads, FilterHz, RtdError, decode_fault_status, MaxFault, ErrorOrAny};
    use std::error::Error as StdError;
    use std::panic;

    #[test]
    fn test_mock_creation_valid() {
        // Valid setup
        let reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty);
        assert!(reader.is_ok(), "Valid params should create mock reader");
    }

    #[test]
    fn test_mock_creation_invalid_cs_pin() {
        // Invalid CS pin (e.g., 99 invalid for GPIO)
        let result = RtdReader::new(99, RTDLeads::Two, FilterHz::Fifty);
        match result {
            Err(e) => assert!(matches!(&*e.downcast_ref::<RtdError>().unwrap(), RtdError::Init(_)), "Should be Init error from mock GPIO"),
            Ok(_) => panic!("Invalid CS pin should fail"),
        }
    }

    #[test]
    fn test_read_temp_100_nominal() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw for ~100Ω with default 400Ω ref: (100 / 400) * 32768 = 8192
        let raw_100ohm = 8192u16;
        reader.inject_rtd_raw(raw_100ohm);

        let temp = reader.read_temp_100().unwrap();
        assert!((temp as f64 / 100.0 - 0.0).abs() < 0.1, "Nominal 100Ω should be ~0°C (got {})", temp as f64 / 100.0);
    }

    #[test]
    fn test_get_resistance_nominal() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw for ~100Ω
        let raw_100ohm = 8192u16;
        reader.inject_rtd_raw(raw_100ohm);

        let resistance = reader.get_resistance().unwrap();
        assert!((resistance - 100.0).abs() < 0.1, "Nominal 100Ω should be ~100Ω (got {})", resistance);
    }

    #[test]
    fn test_get_temperature_nominal() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw for ~100Ω
        let raw_100ohm = 8192u16;
        reader.inject_rtd_raw(raw_100ohm);

        let temp = reader.get_temperature().unwrap();
        assert!((temp - 0.0).abs() < 0.1, "Nominal 100Ω should be ~0°C (got {})", temp);
    }

    #[test]
    fn test_set_calib_scaling() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw for 100Ω with default 400Ω ref: (100 / 400) * 32768 = 8192 (even)
        let raw_100ohm_400 = 8192u16;
        reader.inject_rtd_raw(raw_100ohm_400);

        // Default calib (40000) should read ~100Ω
        let resistance_default = reader.get_resistance().unwrap();
        assert!((resistance_default - 100.0).abs() < 0.1, "Default calib ~100Ω (got {})", resistance_default);

        // Change to 430Ω ref; re-inject raw for *same physical* 100Ω RTD (simulates RREF hardware swap)
        // New raw: (100 / 430) * 32768 ≈ 7620.465; use 7620 (even, yields ~99.999Ω after integer math)
        reader.set_calib(43000);
        let raw_100ohm_430 = 7620u16;
        reader.inject_rtd_raw(raw_100ohm_430);
        let resistance_calib = reader.get_resistance().unwrap();
        assert!((resistance_calib - 100.0).abs() < 0.1, "43000 calib ~100Ω (got {})", resistance_calib);
    }

    #[test]
    fn test_read_fault_status_no_fault() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        let status = reader.read_fault_status().unwrap();
        assert_eq!(status, 0, "Initial status should be 0 (no faults)");
        assert!(decode_fault_status(status).is_empty(), "No faults expected");
    }

    #[test]
    fn test_is_max_fault() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Dummy non-fault error (local RtdError)
        let dummy_err = RtdError::Read("dummy non-fault".to_string());
        let err_ref: &dyn ErrorOrAny = &dummy_err;
        assert!(!reader.is_max_fault(err_ref), "Non-fault error should be false");

        // Fault error - should be true
        reader.inject_fault(MaxFault::RtdInPlusOpen.bit());
        let result = reader.read_temp_100();
        if let Err(e) = result {
            if let Some(err) = e.downcast_ref::<RtdError>() {
                let err_ref: &dyn ErrorOrAny = err;
                assert!(reader.is_max_fault(err_ref), "Fault read should be detected");
            } else {
                panic!("Expected RtdError for fault");
            }
        } else {
            panic!("Expected fault error");
        }
    }

    #[test]
    fn test_inject_fault_open_circuit() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject open circuit (bit 3)
        let fault_bit = MaxFault::RtdInPlusOpen.bit();
        reader.inject_fault(fault_bit);

        // Read should return Fault with correct status
        match reader.read_temp_100() {
            Ok(_) => panic!("Fault injection should cause error"),
            Err(e) => {
                if let Some(err) = e.downcast_ref::<RtdError>() {
                    match err {
                        RtdError::Fault(status) => {
                            assert_eq!(*status, fault_bit, "Should have open circuit bit set");
                            let faults = decode_fault_status(*status);
                            assert_eq!(faults.len(), 1, "Should detect 1 fault");
                            assert_eq!(faults[0], MaxFault::RtdInPlusOpen.description(), "Should be 'RTD IN+ Open Circuit'");
                        }
                        _ => panic!("Expected Fault variant"),
                    }
                } else {
                    panic!("Expected RtdError");
                }
            }
        }
    }

    #[test]
    fn test_inject_fault_overtemp() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject overtemp (bit 5)
        let fault_bit = MaxFault::RtdUnderOrOvertemp.bit();
        reader.inject_fault(fault_bit);

        // Read should return Fault
        match reader.read_temp_100() {
            Ok(_) => panic!("Fault injection should cause error"),
            Err(e) => {
                if let Some(err) = e.downcast_ref::<RtdError>() {
                    match err {
                        RtdError::Fault(status) => {
                            assert_eq!(*status, fault_bit, "Should have overtemp bit set");
                        }
                        _ => panic!("Expected Fault variant"),
                    }
                } else {
                    panic!("Expected RtdError");
                }
            }
        }

        // Clear fault
        reader.clear_fault().unwrap();
        let status_after = reader.get_fault_status();
        assert_eq!(status_after, 0, "Clear should reset status to 0");
    }

    #[test]
    fn test_inject_all_faults() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject all faults (bits 0-7)
        let all_bits = 0xFFu8;
        reader.inject_fault(all_bits);

        // Read should return Fault with all bits
        match reader.read_temp_100() {
            Ok(_) => panic!("All faults should cause error"),
            Err(e) => {
                if let Some(err) = e.downcast_ref::<RtdError>() {
                    match err {
                        RtdError::Fault(status) => {
                            assert_eq!(*status, all_bits, "Should have all bits set");
                            let faults = decode_fault_status(*status);
                            assert_eq!(faults.len(), 8, "Should detect all 8 faults");
                            // Check a couple descriptions
                            assert!(faults.contains(&MaxFault::RtdInPlusOpen.description()));
                            assert!(faults.contains(&MaxFault::RtdUnderOrOvertemp.description()));
                        }
                        _ => panic!("Expected Fault variant"),
                    }
                } else {
                    panic!("Expected RtdError");
                }
            }
        }

        // Clear all
        reader.clear_fault().unwrap();
        assert_eq!(reader.get_fault_status(), 0, "Clear all should reset to 0");
    }

    #[test]
    fn test_edge_short_circuit() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw 0 (short circuit; even, no LSB fault)
        reader.inject_rtd_raw(0u16);

        let resistance = reader.get_resistance().unwrap();
        assert!((resistance - 0.0).abs() < 0.1, "Short should read ~0Ω (got {})", resistance);

        let temp = reader.get_temperature().unwrap();
        assert!((temp + 259.7).abs() < 1.0, "Short should be ~-259.7°C (got {})", temp);  // Exact from integer PT100 approx
    }

    #[test]
    fn test_edge_max_raw() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject max raw without LSB fault (even; 14-bit max >>1 = 16383)
        let max_raw = 32766u16;
        reader.inject_rtd_raw(max_raw);

        let resistance = reader.get_resistance().unwrap();
        assert!((resistance - 400.0).abs() < 1.0, "Max raw should be ~400Ω (got {})", resistance);

        let temp = reader.get_temperature().unwrap();
        assert!((temp - 779.0).abs() < 1.0, "Max raw should be ~779°C (got {})", temp);  // Exact from integer PT100 approx
    }

    #[test]
    fn test_lsb_fault_bit() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();

        // Inject raw with LSB fault bit set (odd number, e.g., 8193 = 8192 + 1)
        let raw_with_fault = 8193u16; // 100Ω raw + bit 0
        reader.inject_rtd_raw(raw_with_fault);

        // Should trigger fault read (ignores raw value, reads status)
        match reader.read_temp_100() {
            Ok(_) => panic!("LSB fault bit should trigger fault"),
            Err(e) => assert!(matches!(&*e.downcast_ref::<RtdError>().unwrap(), RtdError::Fault(_)), "LSB fault should cause Fault error"),
        }
    }

    #[test]
    #[cfg(not(feature = "no_fp"))]
    fn test_pt100_approximation_accuracy() {
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();
        let rref = 400.0;  // Default reference resistor (calib=40000)
        let r0 = 100.0;    // PT100 base resistance
        let alpha = 0.00385;  // PT100 coefficient
        let tolerance = 0.1;  // Absolute °C error allowed (covers integer rounding)

        // Test every 5°C from -50 to +500°C
        for t_input in (-50..=500).step_by(5).map(|t| t as f64) {
            // Exact resistance: R = R0 * (1 + alpha * T)
            let r_exact = r0 * (1.0 + alpha * t_input);

            // Corresponding raw: (R / RREF) * 32768, floored to u16, even (no LSB fault)
            let raw_f64 = (r_exact / rref) * 32768.0;
            let mut raw = raw_f64.floor() as u16;
            raw &= !1u16;  // Ensure even (clear LSB=0 to avoid fault)

            // Clamp raw to valid range [0, 32766] (even max)
            if raw > 32766 {
                raw = 32766;
            }

            // Inject and read
            reader.inject_rtd_raw(raw);
            let t_computed = reader.get_temperature().unwrap();

            // Assert accuracy
            let error = (t_computed - t_input).abs();
            assert!(error < tolerance,
                    "At {}°C: exact R={}Ω, raw={}, computed {}°C (error {}°C > {})",
                    t_input, r_exact, raw, t_computed, error, tolerance);
        }
    }


    #[test]
    #[cfg(not(feature = "no_fp"))]
    fn test_no_fp_mode_compiles() {
        // This test ensures f64 methods are available when no_fp is off
        let mut reader = RtdReader::new(8, RTDLeads::Two, FilterHz::Fifty).unwrap();
        let _ = reader.get_resistance(); // Should compile
        let _ = reader.get_temperature(); // Should compile
    }

    #[test]
    #[cfg(feature = "no_fp")]
    fn test_no_fp_mode_f64_unavailable() {
        // This won't compile if no_fp is enabled (missing methods), but cfg ensures it's skipped
        // Just a placeholder to document the behavior
        compile_error!("f64 methods should be unavailable with no_fp feature");
    }
}