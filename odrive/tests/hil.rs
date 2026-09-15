#![cfg(target_os = "linux")]

//! Hardware-in-the-loop test coverage for a real ODrive over SocketCAN.
//!
//! Run with:
//!
//! ```text
//! ODRIVE_HIL_INTERFACE=can0 \
//! ODRIVE_HIL_NODE=0 \
//! ODRIVE_HIL_EXPECTED_FIRMWARE=0.6.11 \
//! ODRIVE_HIL_ENDPOINTS=/path/to/flat_endpoints.json \
//! ODRIVE_HIL_CONFIG='{"can.config.protocol":1}' \
//! ODRIVE_HIL_SDO_ENDPOINT=3 \
//! ODRIVE_HIL_SDO_KIND=float \
//! cargo test -p odrive --test hil -- --ignored --nocapture
//! ```
//!
//! The test is deliberately ignored by default. It erases the stored configuration,
//! writes configuration, saves it, and sends motor-control commands. Use a
//! mechanically safe test fixture and a device dedicated to testing.

use odrive::{
    AxisState, ControlMode, InputMode,
    can::{ODrive, ValueKind},
};
use serde_json::Value as JsonValue;
use socketcan::tokio::CanSocket;
use std::{env, fs, io, time::Duration};
use tokio::time::{sleep, timeout};

fn required(name: &str) -> String {
    env::var(name).unwrap_or_else(|_| panic!("{name} must be set for the ODrive HIL test"))
}

fn parse_kind(value: &str) -> ValueKind {
    match value {
        "bool" => ValueKind::Bool,
        "uint8" => ValueKind::U8,
        "int8" => ValueKind::I8,
        "uint16" => ValueKind::U16,
        "int16" => ValueKind::I16,
        "uint32" => ValueKind::U32,
        "int32" => ValueKind::I32,
        "float" => ValueKind::Float,
        _ => panic!("unsupported ODRIVE_HIL_SDO_KIND: {value}"),
    }
}

async fn exercise(odrive: &ODrive) -> io::Result<()> {
    // This must remain the first device operation: it removes state left by a
    // previous run before any configuration or motion command is sent.
    odrive.erase_configuration().await?;
    sleep(Duration::from_secs(2)).await;
    odrive.reboot().await?;
    sleep(Duration::from_secs(5)).await;

    let version = odrive.get_version().await?;
    let actual_firmware = format!(
        "{}.{}.{}",
        version.fw_version_major(),
        version.fw_version_minor(),
        version.fw_version_revision()
    );
    assert_eq!(
        actual_firmware,
        required("ODRIVE_HIL_EXPECTED_FIRMWARE"),
        "connected firmware does not match the firmware used for this HIL setup"
    );
    assert!(
        !version.fw_version_unreleased(),
        "unreleased firmware is not supported by this fixture"
    );
    assert!(version.protocol_version() > 0);

    let initial_error = odrive.get_error().await?;
    assert_eq!(
        initial_error.active_errors().bits(),
        0,
        "factory reset left an active axis error"
    );
    odrive.clear_errors(false).await?;
    odrive.clear_errors(true).await?;
    odrive.set_axis_state(AxisState::Idle).await?;

    // Exercise every controller/input enum through the CAN command path while
    // keeping the input inactive and the axis idle.
    for control in [
        ControlMode::VoltageControl,
        ControlMode::TorqueControl,
        ControlMode::VelocityControl,
        ControlMode::PositionControl,
    ] {
        for input in [
            InputMode::Inactive,
            InputMode::Passthrough,
            InputMode::VelocityRamp,
            InputMode::PositionFilter,
            InputMode::MixChannels,
            InputMode::TrapezoidalTrajectory,
            InputMode::TroqueRamp,
            InputMode::Mirror,
            InputMode::Tuning,
        ] {
            odrive.set_controller_mode(control, input).await?;
        }
    }

    odrive.set_input_position(0.0, 0, 0).await?;
    odrive.set_input_velocity(0.0, 0.0).await?;
    odrive.set_input_torque(0.0).await?;
    odrive.set_limits(1.0, 1.0).await?;
    odrive.set_trajectory_velocity_limit(1.0).await?;
    odrive.set_trajectory_acceleration_limit(1.0, 1.0).await?;
    odrive.set_trajectory_inertia(0.0).await?;
    odrive.set_absolute_position(0.0).await?;
    odrive.set_position_gain(1.0).await?;
    odrive.set_velocity_gains(0.1, 0.0).await?;

    // Exercise all request/response telemetry methods.
    let _ = odrive.get_encoder_estimates().await?;
    let _ = odrive.get_iq().await?;
    let _ = odrive.get_temperature().await?;
    let _ = odrive.get_bus_voltage_current().await?;
    let _ = odrive.get_torques().await?;
    let _ = odrive.get_powers().await?;

    // Read and write the same explicitly selected endpoint. This proves both
    // SDO directions without changing the fixture's configured value.
    let endpoint = required("ODRIVE_HIL_SDO_ENDPOINT")
        .parse::<u16>()
        .expect("ODRIVE_HIL_SDO_ENDPOINT must be a u16");
    let kind = parse_kind(&required("ODRIVE_HIL_SDO_KIND"));
    let original = odrive.sdo_read(endpoint, kind).await?;
    odrive.sdo_write(endpoint, original).await?;
    let round_trip = odrive.sdo_read(endpoint, kind).await?;
    assert_eq!(format!("{round_trip:?}"), format!("{original:?}"));

    let endpoint_file = required("ODRIVE_HIL_ENDPOINTS");
    let endpoints_json: JsonValue = serde_json::from_str(&fs::read_to_string(endpoint_file)?)?;
    let endpoints = odrive::flat_endpoints::FlatEndpoints::from_json(endpoints_json)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "invalid flat endpoints file"))?;
    let config: JsonValue = serde_json::from_str(&required("ODRIVE_HIL_CONFIG"))?;
    odrive.apply_configuration(&endpoints, &config).await?;

    // Persistence and lifecycle coverage. The test fixture must be prepared so
    // that saving this configuration is safe.
    odrive.save_configuration().await?;
    odrive.reboot().await?;
    sleep(Duration::from_secs(5)).await;
    let _ = odrive.get_version().await?;
    odrive.clear_errors(false).await?;
    odrive.set_axis_state(AxisState::Idle).await?;

    // E-stop is sent last among the recoverable commands and must be visible
    // through the error query. Clear it before optional DFU entry.
    odrive.estop().await?;
    let after_estop = odrive.get_error().await?;
    assert!(
        after_estop
            .disarm_reason()
            .contains(odrive::AxisErrors::ESTOP_REQUESTED)
    );
    odrive.clear_errors(false).await?;

    if env::var_os("ODRIVE_HIL_ENTER_DFU").is_some() {
        // DFU intentionally ends CAN communication; no commands may follow.
        odrive.enter_dfu_mode2().await?;
    }

    Ok(())
}

#[tokio::test]
#[ignore = "requires a dedicated ODrive connected to SocketCAN; see tests/hil.rs"]
async fn odrive_hil() {
    let interface = env::var("ODRIVE_HIL_INTERFACE").unwrap_or_else(|_| "can0".to_owned());
    let axis = required("ODRIVE_HIL_NODE")
        .parse::<u8>()
        .expect("ODRIVE_HIL_NODE must be a u8");
    assert!(
        axis < 64,
        "ODRIVE_HIL_NODE must fit the 6-bit CANSimple node id"
    );

    let socket = CanSocket::open(&interface).expect("failed to open ODRIVE_HIL_INTERFACE");
    let odrive = ODrive::new(socket, axis);
    let result = timeout(Duration::from_secs(60), exercise(&odrive)).await;
    // Best-effort safety cleanup. This also runs when an I/O operation returns
    // an error, though not when an assertion panics or DFU was requested.
    if env::var_os("ODRIVE_HIL_ENTER_DFU").is_none() {
        let _ = odrive.set_axis_state(AxisState::Idle).await;
    }
    result
        .expect("HIL test timed out waiting for the ODrive")
        .expect("ODrive HIL operation failed");
}
