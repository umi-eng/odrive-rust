#![doc = include_str!("../README.md")]

pub mod can;
#[cfg(feature = "flat-endpoints")]
pub mod flat_endpoints;

use bitflags::bitflags;

bitflags! {
    /// Axis error.
    ///
    /// [Reference](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct AxisErrors: u32 {
        const INITIALIZING = 0x1;
        const SYSTEM_LEVEL = 0x2;
        const TIMING_ERROR = 0x4;
        const MISSING_ESTIMATE = 0x8;
        const BAD_CONFNIG = 0x10;
        const DRV_FAULT = 0x20;
        const MISSING_INPUT = 0x40;
        const DC_BUS_OVER_VOLTAGE = 0x100;
        const DC_BUS_UNDER_VOLTAGE = 0x200;
        const DC_BUS_OVER_CURRENT = 0x400;
        const DC_BUS_OVER_REGEN_CURRENT = 0x800;
        const CURRENT_LIMIT_VIOLATION = 0x1000;
        const MOTOR_OVER_TEMP = 0x2000;
        const INVERTER_OVER_TEMP = 0x4000;
        const VELOCITY_LIMIT_VIOLATION = 0x8000;
        const POSITION_LIMIT_VIOLATION = 0x10000;
        const WATCHDOG_TIMER_EXPIRED = 0x1000000;
        const ESTOP_REQUESTED = 0x2000000;
        const SPINOUT_DETECTED = 0x4000000;
        const BRAKE_RESISTOR_DISARMED = 0x8000000;
        const THERMISTOR_DISCONNECTED = 0x10000000;
        const CALIBRATION_ERROR = 0x40000000;
        const _ = !0; // the source may set any flags
    }
}

/// Axis state.
///
/// [Reference](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AxisState {
    /// Will fall through to idle.
    Undefined = 0,
    /// Disable motor PWM and do nothing.
    Idle = 1,
    /// Run the startup procedure.
    StartupSequence = 2,
    /// Run all essential calibration procedures for the current ODrive
    /// configuration.
    FullCalibration = 3,
    /// Measure phase resistance and phase inductance of the motor.
    MotorCalibration = 4,
    /// Turn the motor in one direction until the encoder index is traversed.
    EncoderIndexSearch = 6,
    /// Turn the motor in one direction for a few seconds and then back to
    /// measure the offset between the encoder position and the electrical
    /// phase.
    EncoderOffsetCalibration = 7,
    /// Run closed loop control.
    ClosedLoopControl = 8,
    /// Run lockin spin.
    LockinSpin = 9,
    /// Run encoder direction search.
    EncoderDirFind = 10,
    /// Run axis homing function.
    Homing = 11,
    /// Rotate the motor in lockin and calibrate hall polarity.
    EncoderHallPolarityCalibration = 12,
    /// Rotate the motor for 30s to calibrate hall sensor edge offsets.
    EncoderHallPhaseCalibration = 13,
    /// Calibrate the anticogging algorithm.
    AnticoggingCalibration = 14,
    /// Calibrate harmonic compensation.
    HarmonicCalibration = 15,
    /// Calibrate harmonic compensation for commutation encoder.
    HarmonicCalibrationCommutation = 16,
}

/// Procedure result.
///
/// [Reference](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.ProcedureResult)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProcedureResult {
    /// The procedure finished without any faults.
    Success = 0,
    /// The procedure has not yet finished.
    Busy = 1,
    /// The last procedure was cancelled by the user.
    Cancelled = 2,
    /// A fault was encountered and the axis has been disarmed.
    Disarmed = 3,
    /// The procedure component did not respond as expected.
    NoResponse = 4,
    /// The configuration values for pole pairs and and/or encoder counts per
    /// revolution do not corroborate the measured rotations. Please verify
    /// these settings match the hardware.
    PolePairCprMismatch = 5,
    /// The measured motor phase resistance is outside of the plausible range or
    /// bad calibration parameters were used.
    PhaseResistanceOutOfRange = 6,
    /// The measured motor phase inductance is outside of the plausible range or
    /// bad calibration parameters were used.
    PhaseInductanceOutOfRange = 7,
    /// The motor phase resistances are not balanced.
    UnbalancedPhases = 8,
    /// The configuration value of motor type is not valid.
    InvalidMotorType = 9,
    /// During hall encoder calibration the ODrive detected too many bad hall
    /// states.
    IllegalHallState = 10,
    /// A timeout occurred.
    Timeout = 11,
    /// Homing was requested without enabling the endstop.
    HomingWithoutEndstop = 12,
    /// The requested state was not a valid axis state.
    InvalidState = 13,
    /// The requested state could not be entered because the axis is not calibrated.
    NotCalibrated = 14,
    /// The calibration did not converge.
    NotConverging = 15,
}

/// Control mode.
///
/// [Reference](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlMode {
    /// This mode is not used internally.
    VoltageControl = 0,
    /// Use only the inner torque control loop.
    TorqueControl = 1,
    /// Use both the inner torque control loop and the velocity control loop.
    VelocityControl = 2,
    /// Use the inner torque loop, the velocity control loop, and the outer
    /// position control loop.
    PositionControl = 3,
}

/// Input mode.
///
/// [Reference](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InputMode {
    /// Disable inputs. Setpoints retainr their last value.
    Inactive = 0,
    /// Pass inputs through to setpoints directly.
    Passthrough = 1,
    /// Ramps a velocity command from the current value to the target value.
    VelocityRamp = 2,
    /// Implements a 2nd order position tracking filter.
    PositionFilter = 3,
    /// Not implemented.
    MixChannels = 4,
    /// Implements an online trapezoidal trajectory planner.
    TrapezoidalTrajectory = 5,
    /// Ramp a torque command from the current value to the target value.
    TroqueRamp = 6,
    /// Electronic mirroring between two axes.
    Mirror = 7,
    /// Tuning mode.
    Tuning = 8,
}
