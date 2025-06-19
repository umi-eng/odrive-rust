use crate::{AxisErrors, AxisState, ControlMode, InputMode};
use cansimple::Id;
use embedded_can::Frame;
use socketcan::{CanFrame, tokio::CanSocket};
use std::io;

/// ODrive driver.
///
/// Implemented using a tokio-async CAN socket.
pub struct ODrive {
    interface: CanSocket,
    axis: u8,
}

impl ODrive {
    /// Creates a new ODrive interface.
    pub fn new(interface: CanSocket, axis: u8) -> Self {
        Self { interface, axis }
    }

    /// Get version information.
    pub async fn get_version(&self) -> io::Result<Version> {
        let id = Id::new(self.axis, 0x00);

        // request the message with an rtr frame
        self.interface
            .write_frame(CanFrame::new_remote(id, 0).unwrap())
            .await?;

        let frame = loop {
            let frame = self.interface.read_frame().await?;
            if frame.id() == id.into() {
                break frame;
            }
        };

        if frame.data().len() != 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Frame data length invalid: {} != 8", frame.data().len()),
            ));
        }

        Ok(Version {
            protocol_version: frame.data()[0],
            hw_version_major: frame.data()[1],
            hw_version_minor: frame.data()[2],
            hw_version_variant: frame.data()[3],
            fw_version_major: frame.data()[4],
            fw_version_minor: frame.data()[5],
            fw_version_revision: frame.data()[6],
            fw_version_unreleased: frame.data()[7] == 1,
        })
    }

    /// Cause the axis to disarm.
    pub async fn estop(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x02), &[]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get errors.
    pub async fn get_error(&self) -> io::Result<Error> {
        let id = Id::new(self.axis, 0x03);

        // request the message with an rtr frame
        self.interface
            .write_frame(CanFrame::new_remote(id, 0).unwrap())
            .await?;

        let frame = loop {
            let frame = self.interface.read_frame().await?;
            if frame.id() == id.into() {
                break frame;
            }
        };

        if frame.data().len() != 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Frame data length invalid: {} != 8", frame.data().len()),
            ));
        }

        let data = frame.data();

        Ok(Error {
            active_errors: AxisErrors::from_bits_truncate(u32::from_le_bytes([
                data[0], data[1], data[2], data[3],
            ])),
            disarm_reason: AxisErrors::from_bits_truncate(u32::from_le_bytes([
                data[4], data[5], data[6], data[7],
            ])),
        })
    }

    /// Change the axis state.
    pub async fn set_axis_state(&self, state: AxisState) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x07), &(state as u32).to_le_bytes()).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get the encoder estimates.
    pub async fn get_encoder_estimates(&self) -> io::Result<EncoderEstimates> {
        let id = Id::new(self.axis, 0x09);

        // request the message with an rtr frame
        self.interface
            .write_frame(CanFrame::new_remote(id, 0).unwrap())
            .await?;

        let frame = loop {
            let frame = self.interface.read_frame().await?;
            if frame.id() == id.into() {
                break frame;
            }
        };

        if frame.data().len() != 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("Frame data length invalid: {} != 8", frame.data().len()),
            ));
        }

        let data = frame.data();

        Ok(EncoderEstimates {
            position: f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            velocity: f32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        })
    }

    /// Set the control loop mode.
    pub async fn set_controller_mode(
        &self,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> io::Result<()> {
        let mut data = vec![];
        data.extend((control_mode as u8).to_le_bytes());
        data.extend((input_mode as u8).to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0b), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set input position.
    ///
    /// - `position` rev.
    /// - `velocity` 0.001 rev/s (default).
    /// - `torque` 0.001 Nm (default).
    ///
    /// Velocity and torque scale is configurable.
    pub async fn set_input_position(
        &self,
        position: f32,
        velocity: i16,
        torque: i16,
    ) -> io::Result<()> {
        let mut data = vec![];
        data.extend(position.to_le_bytes());
        data.extend(velocity.to_le_bytes());
        data.extend(torque.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0c), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set input velocity.
    ///
    /// - `velocity` rev/s.
    /// - `torque` Nm.
    pub async fn set_input_velocity(&self, velocity: f32, torque: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(velocity.to_le_bytes());
        data.extend(torque.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0d), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set input torque.
    ///
    /// - `torque` amps.
    pub async fn set_input_torque(&self, torque: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(torque.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0e), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set limits.
    ///
    /// - `velocity` limit rev/s.
    /// - `current` limit amps.
    pub async fn set_lmits(&self, velocity: f32, current: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(velocity.to_le_bytes());
        data.extend(current.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0f), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set trajectory velocity limit.
    ///
    /// `velocity` limit rev/s.
    pub async fn set_trajectory_velocity_limit(&self, velocity: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(velocity.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x11), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set trajectory acceleration limits.
    ///
    /// `acceleration` limit rev/s^2.
    /// `deceleration` limit rev/s^2.
    pub async fn set_trajectory_acceleration_limit(
        &self,
        acceleration: f32,
        deceleration: f32,
    ) -> io::Result<()> {
        let mut data = vec![];
        data.extend(acceleration.to_le_bytes());
        data.extend(deceleration.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x12), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set trajectory inertia.
    ///
    /// `inertia` Nm/(rev/s^2).
    pub async fn set_trajectory_inertia(&self, inertia: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(inertia.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x13), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Reboot the device.
    pub async fn reboot(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16), &[0]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Save configuration.
    pub async fn save_configuration(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16), &[1]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Erase configuration.
    pub async fn erase_configuration(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16), &[2]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Enter DFU mode 2.
    pub async fn enter_dfu_mode2(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16), &[3]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Clear disarm reason and procedure result.
    pub async fn clear_errors(&self, identify: bool) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x18), &[identify as u8]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set the absolute position estimate.
    ///
    /// - `position` rev.
    pub async fn set_absolute_position(&self, position: f32) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x19), &position.to_le_bytes()).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set position gain.
    ///
    /// - `gain` (rev/s)/rev.
    pub async fn set_position_gain(&self, gain: f32) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x1a), &gain.to_le_bytes()).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set velocity gains.
    ///
    /// - `gain` Nm/(rev/s).
    /// - `integrator_gain` Nm/rev.
    pub async fn set_velocity_gains(&self, gain: f32, integrator_gain: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(gain.to_le_bytes());
        data.extend(integrator_gain.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x1b), &data).unwrap();
        self.interface.write_frame(frame).await
    }
}

/// Version information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    pub protocol_version: u8,
    pub hw_version_major: u8,
    pub hw_version_minor: u8,
    pub hw_version_variant: u8,
    pub fw_version_major: u8,
    pub fw_version_minor: u8,
    pub fw_version_revision: u8,
    pub fw_version_unreleased: bool,
}

/// Error message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Error {
    pub active_errors: AxisErrors,
    pub disarm_reason: AxisErrors,
}

/// Encoder estimates.
#[derive(Debug, Clone, Copy)]
pub struct EncoderEstimates {
    /// Position estimate in revolutions
    pub position: f32,
    /// Velocity estimate in rev/s
    pub velocity: f32,
}
