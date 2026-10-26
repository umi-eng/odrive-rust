//! # CAN interface for ODrives

#[cfg(feature = "flat-endpoints")]
use crate::flat_endpoints::FlatEndpoints;
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
        let id = Id::new(self.axis, 0x00).unwrap();

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

        Ok(Version::new(frame.data().try_into().unwrap()))
    }

    /// Cause the axis to disarm.
    pub async fn estop(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x02).unwrap(), &[]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get errors.
    pub async fn get_error(&self) -> io::Result<Error> {
        let id = Id::new(self.axis, 0x03).unwrap();

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

        Ok(Error::new(data.try_into().unwrap()))
    }

    /// Write an arbitrary parameter.
    pub async fn sdo_write(&self, endpoint: u16, value: Value) -> io::Result<()> {
        let id = Id::new(self.axis, 0x04).unwrap();

        let mut data = vec![];
        data.push(1); // opcode = write
        data.extend(endpoint.to_le_bytes());
        data.push(0); // reserved
        data.extend(value.to_le_bytes());

        self.interface
            .write_frame(CanFrame::new(id, &data).unwrap())
            .await
    }

    /// Request an arbitrary parameter without waiting for the reply.
    ///
    /// The reply arrives as [`Message::SdoReply`] from [`feedback`], for callers that read
    /// every frame in one place and can't block on a single reply.
    pub async fn sdo_request(&self, endpoint: u16) -> io::Result<()> {
        self.sdo_read_request(endpoint, 0).await
    }

    /// Request the ID of the ODrive's endpoint list (its JSON descriptor) without waiting.
    ///
    /// The upper 16 bits of the ID are the CRC16 of the JSON, the same value as the `crc` field
    /// of the `flat_endpoints.json` it was published with, so it identifies the endpoint list
    /// even between builds that report the same version. The reply arrives as a
    /// [`Message::SdoReply`] for endpoint 0; read it as [`ValueKind::U32`].
    ///
    /// Reading endpoint 0 with `0xFFFFFFFF` in the value field isn't in ODrive's CAN
    /// documentation. It's what odrivetool sends to identify a device, confirmed on firmware
    /// 0.6.12: the ID's upper 16 bits matched both the published `crc` and ODrive's documented
    /// CRC16 over the JSON the device sent.
    pub async fn request_json_id(&self) -> io::Result<()> {
        self.sdo_read_request(0, u32::MAX).await
    }

    async fn sdo_read_request(&self, endpoint: u16, value: u32) -> io::Result<()> {
        let id = Id::new(self.axis, 0x04).unwrap();

        let mut data = vec![];
        data.push(0); // opcode = read
        data.extend(endpoint.to_le_bytes());
        data.push(0); // reserved
        data.extend(value.to_le_bytes());

        self.interface
            .write_frame(CanFrame::new(id, &data).unwrap())
            .await
    }

    /// Read an arbitrary parameter.
    pub async fn sdo_read(&self, endpoint: u16, kind: ValueKind) -> io::Result<Value> {
        self.sdo_request(endpoint).await?;

        let id = Id::new(self.axis, 0x05).unwrap();
        loop {
            let frame = self.interface.read_frame().await?;
            if frame.id() != id.into() {
                continue;
            }
            if let Some(reply) = SdoReply::new(frame.data())
                && reply.endpoint() == endpoint
            {
                return Ok(reply.value(kind));
            }
        }
    }

    /// Change the axis state.
    pub async fn set_axis_state(&self, state: AxisState) -> io::Result<()> {
        let frame = CanFrame::new(
            Id::new(self.axis, 0x07).unwrap(),
            &(state as u32).to_le_bytes(),
        )
        .unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get the encoder estimates.
    pub async fn get_encoder_estimates(&self) -> io::Result<EncoderEstimate> {
        let id = Id::new(self.axis, 0x09).unwrap();

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

        Ok(EncoderEstimate::new(data.try_into().unwrap()))
    }

    /// Set the control loop mode.
    pub async fn set_controller_mode(
        &self,
        control_mode: ControlMode,
        input_mode: InputMode,
    ) -> io::Result<()> {
        let mut data = vec![];
        data.extend((control_mode as u32).to_le_bytes());
        data.extend((input_mode as u32).to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0b).unwrap(), &data).unwrap();
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
        let frame = CanFrame::new(Id::new(self.axis, 0x0c).unwrap(), &data).unwrap();
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
        let frame = CanFrame::new(Id::new(self.axis, 0x0d).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set input torque.
    ///
    /// - `torque` Nm.
    pub async fn set_input_torque(&self, torque: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(torque.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0e).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set limits.
    ///
    /// - `velocity` limit rev/s.
    /// - `current` limit amps.
    pub async fn set_limits(&self, velocity: f32, current: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(velocity.to_le_bytes());
        data.extend(current.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x0f).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set trajectory velocity limit.
    ///
    /// `velocity` limit rev/s.
    pub async fn set_trajectory_velocity_limit(&self, velocity: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(velocity.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x11).unwrap(), &data).unwrap();
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
        let frame = CanFrame::new(Id::new(self.axis, 0x12).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set trajectory inertia.
    ///
    /// `inertia` Nm/(rev/s^2).
    pub async fn set_trajectory_inertia(&self, inertia: f32) -> io::Result<()> {
        let mut data = vec![];
        data.extend(inertia.to_le_bytes());
        let frame = CanFrame::new(Id::new(self.axis, 0x13).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get motor current.
    ///
    /// Response: (setpoint, measured)
    pub async fn get_iq(&self) -> io::Result<(f32, f32)> {
        let id = Id::new(self.axis, 0x14).unwrap();

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

        Ok((
            f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            f32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        ))
    }

    /// Get temperature.
    pub async fn get_temperature(&self) -> io::Result<Temperature> {
        let id = Id::new(self.axis, 0x15).unwrap();

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

        Ok(Temperature::new(data.try_into().unwrap()))
    }

    /// Reboot the device.
    pub async fn reboot(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16).unwrap(), &[0]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get bus voltage and current.
    pub async fn get_bus_voltage_current(&self) -> io::Result<BusVoltageCurrent> {
        let id = Id::new(self.axis, 0x17).unwrap();

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

        Ok(BusVoltageCurrent::new(data.try_into().unwrap()))
    }

    /// Save configuration.
    pub async fn save_configuration(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16).unwrap(), &[1]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Erase the stored configuration.
    pub async fn erase_configuration(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16).unwrap(), &[2]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Enter DFU mode 2.
    pub async fn enter_dfu_mode2(&self) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x16).unwrap(), &[3]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Clear disarm reason and procedure result.
    pub async fn clear_errors(&self, identify: bool) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x18).unwrap(), &[identify as u8]).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set the absolute position estimate.
    ///
    /// - `position` rev.
    pub async fn set_absolute_position(&self, position: f32) -> io::Result<()> {
        let frame =
            CanFrame::new(Id::new(self.axis, 0x19).unwrap(), &position.to_le_bytes()).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Set position gain.
    ///
    /// - `gain` (rev/s)/rev.
    pub async fn set_position_gain(&self, gain: f32) -> io::Result<()> {
        let frame = CanFrame::new(Id::new(self.axis, 0x1a).unwrap(), &gain.to_le_bytes()).unwrap();
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
        let frame = CanFrame::new(Id::new(self.axis, 0x1b).unwrap(), &data).unwrap();
        self.interface.write_frame(frame).await
    }

    /// Get torque values.
    pub async fn get_torques(&self) -> io::Result<Torque> {
        let id = Id::new(self.axis, 0x1c).unwrap();

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

        Ok(Torque::new(data.try_into().unwrap()))
    }

    /// Get power values.
    pub async fn get_powers(&self) -> io::Result<Power> {
        let id = Id::new(self.axis, 0x1d).unwrap();

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

        Ok(Power::new(data.try_into().unwrap()))
    }

    #[cfg(feature = "flat-endpoints")]
    pub async fn apply_configuration(
        &self,
        endpoints: &FlatEndpoints,
        config: &serde_json::Value,
    ) -> io::Result<()> {
        let Some(items) = config.as_object() else {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "Expected object",
            ));
        };

        for (key, value) in items.iter() {
            let Some((id, kind)) = endpoints.get(key) else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Configuration endpoint not found in flat endpoints",
                ));
            };

            let Some(value) = Value::try_from_json(value, kind) else {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Configuration value not able to be converted into an SDO value",
                ));
            };

            let endpoint = u16::try_from(id).map_err(|_| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Endpoint ID out of range for u16",
                )
            })?;
            self.sdo_write(endpoint, value).await?;
        }

        Ok(())
    }
}

/// Parses an ODrive feedback frame.
///
/// Returns the sending node ID and the decoded message type. Frames with an
/// extended ID, an invalid payload length, or a command that is not an ODrive
/// feedback message return `None`.
pub fn feedback(frame: &impl Frame) -> Option<(u8, Message)> {
    let id = match frame.id() {
        embedded_can::Id::Standard(id) => Id::from(id),
        embedded_can::Id::Extended(_) => return None,
    };

    // SDO replies are as long as their value, so they're decoded before the fixed-length
    // conversion the other messages need
    if id.command() == 0x05 {
        return SdoReply::new(frame.data()).map(|reply| (id.node(), Message::SdoReply(reply)));
    }

    let payload = frame.data().try_into().ok()?;
    let message = match id.command() {
        0x00 => Message::Version(Version::new(payload)),
        0x01 => Message::Heartbeat(Heartbeat::new(payload)),
        0x03 => Message::Error(Error::new(payload)),
        0x09 => Message::EncoderEstimates(EncoderEstimate::new(payload)),
        0x14 => Message::Iq(Iq::new(payload)),
        0x15 => Message::Temperature(Temperature::new(payload)),
        0x17 => Message::BusVoltageCurrent(BusVoltageCurrent::new(payload)),
        0x1c => Message::Torque(Torque::new(payload)),
        0x1d => Message::Power(Power::new(payload)),
        _ => return None,
    };

    Some((id.node(), message))
}

/// A message periodically sent by an ODrive.
#[derive(Debug, Clone, Copy)]
pub enum Message {
    Version(Version),
    Heartbeat(Heartbeat),
    Error(Error),
    EncoderEstimates(EncoderEstimate),
    Iq(Iq),
    Temperature(Temperature),
    BusVoltageCurrent(BusVoltageCurrent),
    Torque(Torque),
    Power(Power),
    SdoReply(SdoReply),
}

fn float(payload: &[u8; 8], offset: usize) -> f32 {
    f32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap())
}

/// Version information.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Version {
    payload: [u8; 8],
}

impl Version {
    pub(crate) fn new(payload: [u8; 8]) -> Self {
        Self { payload }
    }
    /// Returns the CAN protocol version reported by the ODrive.
    pub fn protocol_version(&self) -> u8 {
        self.payload[0]
    }

    /// Returns the hardware major version.
    pub fn hw_version_major(&self) -> u8 {
        self.payload[1]
    }

    /// Returns the hardware minor version.
    pub fn hw_version_minor(&self) -> u8 {
        self.payload[2]
    }

    /// Returns the hardware variant.
    pub fn hw_version_variant(&self) -> u8 {
        self.payload[3]
    }

    /// Returns the firmware major version.
    pub fn fw_version_major(&self) -> u8 {
        self.payload[4]
    }

    /// Returns the firmware minor version.
    pub fn fw_version_minor(&self) -> u8 {
        self.payload[5]
    }

    /// Returns the firmware revision.
    pub fn fw_version_revision(&self) -> u8 {
        self.payload[6]
    }

    /// Returns whether the firmware is marked as unreleased.
    pub fn fw_version_unreleased(&self) -> bool {
        self.payload[7] == 1
    }
}

/// Heartbeat status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat {
    payload: [u8; 8],
}

impl Heartbeat {
    fn new(payload: [u8; 8]) -> Self {
        Self { payload }
    }
    /// Returns the active axis error flags.
    pub fn axis_error(&self) -> AxisErrors {
        AxisErrors::from_bits_retain(u32::from_le_bytes(self.payload[0..4].try_into().unwrap()))
    }

    /// Returns the current axis state value.
    pub fn axis_state(&self) -> u8 {
        self.payload[4]
    }

    /// Returns the procedure result value.
    pub fn procedure_result(&self) -> u8 {
        self.payload[5]
    }

    /// Returns whether the current trajectory is complete.
    pub fn trajectory_done(&self) -> bool {
        self.payload[6] != 0
    }
}

/// Error message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Error {
    payload: [u8; 8],
}

impl Error {
    fn new(payload: [u8; 8]) -> Self {
        Self { payload }
    }
    /// Returns the currently active axis error flags.
    pub fn active_errors(&self) -> AxisErrors {
        AxisErrors::from_bits_retain(u32::from_le_bytes(self.payload[0..4].try_into().unwrap()))
    }

    /// Returns the error flags that caused the most recent disarm.
    pub fn disarm_reason(&self) -> AxisErrors {
        AxisErrors::from_bits_retain(u32::from_le_bytes(self.payload[4..8].try_into().unwrap()))
    }
}

/// Reply to an SDO read (TxSdo).
///
/// The ODrive sends only as many value bytes as the endpoint's type needs, for example 1 for a
/// bool, so a reply can be shorter than 8 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoReply {
    endpoint: u16,
    value: [u8; 4],
}

impl SdoReply {
    /// Decodes a TxSdo payload: reserved, endpoint ID (2 bytes), reserved, then up to 4 value
    /// bytes. Missing value bytes are zero.
    fn new(data: &[u8]) -> Option<Self> {
        if data.len() < 4 || data.len() > 8 {
            return None;
        }
        let endpoint = u16::from_le_bytes([data[1], data[2]]);
        let mut value = [0; 4];
        value[..data.len() - 4].copy_from_slice(&data[4..]);
        Some(Self { endpoint, value })
    }

    /// Returns the endpoint ID this reply is for.
    pub fn endpoint(&self) -> u16 {
        self.endpoint
    }

    /// Returns the value, decoded as `kind`.
    pub fn value(&self, kind: ValueKind) -> Value {
        decode_value(&self.value, kind)
    }
}

fn decode_value(data: &[u8; 4], kind: ValueKind) -> Value {
    match kind {
        ValueKind::Bool => Value::Bool(data[0] == 1),
        ValueKind::U8 => Value::U8(data[0]),
        ValueKind::I8 => Value::I8(i8::from_le_bytes([data[0]])),
        ValueKind::U16 => Value::U16(u16::from_le_bytes([data[0], data[1]])),
        ValueKind::I16 => Value::I16(i16::from_le_bytes([data[0], data[1]])),
        ValueKind::U32 => Value::U32(u32::from_le_bytes(*data)),
        ValueKind::I32 => Value::I32(i32::from_le_bytes(*data)),
        ValueKind::Float => Value::Float(f32::from_le_bytes(*data)),
    }
}

macro_rules! float_pair_message {
    ($name:ident, $first:ident, $second:ident, $first_doc:literal, $second_doc:literal) => {
        #[derive(Debug, Clone, Copy)]
        pub struct $name {
            payload: [u8; 8],
        }
        impl $name {
            fn new(payload: [u8; 8]) -> Self {
                Self { payload }
            }
            #[doc = $first_doc]
            pub fn $first(&self) -> f32 {
                float(&self.payload, 0)
            }
            #[doc = $second_doc]
            pub fn $second(&self) -> f32 {
                float(&self.payload, 4)
            }
        }
    };
}

float_pair_message!(
    EncoderEstimate,
    position,
    velocity,
    "Position estimate in revolutions.",
    "Velocity estimate in revolutions per second."
);
float_pair_message!(
    Iq,
    setpoint,
    measured,
    "Iq setpoint in amps.",
    "Measured Iq in amps."
);
float_pair_message!(
    Temperature,
    fet,
    motor,
    "FET temperature in Celsius.",
    "Motor temperature in Celsius."
);
float_pair_message!(
    BusVoltageCurrent,
    voltage,
    current,
    "Bus voltage in volts.",
    "Bus current in amps."
);
float_pair_message!(
    Torque,
    target,
    estimate,
    "Torque target in Nm.",
    "Torque estimate in Nm."
);
float_pair_message!(
    Power,
    electrical,
    mechanical,
    "Electrical power in watts.",
    "Mechanical power in watts."
);

/// Arbitrary parameter value.
#[derive(Debug, Clone, Copy)]
pub enum Value {
    Bool(bool),
    U8(u8),
    I8(i8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    Float(f32),
}

impl Value {
    /// Convert to a const length slice.
    ///
    /// Any unused bytes will be zero.
    pub fn to_le_bytes(&self) -> [u8; 4] {
        match *self {
            Self::Bool(b) => [b as u8, 0, 0, 0],
            Self::U8(u) => [u, 0, 0, 0],
            Self::I8(i) => [i.to_le_bytes()[0], 0, 0, 0],
            Self::U16(u) => [u.to_le_bytes()[0], u.to_le_bytes()[1], 0, 0],
            Self::I16(i) => [i.to_le_bytes()[0], i.to_le_bytes()[1], 0, 0],
            Self::U32(u) => u.to_le_bytes(),
            Self::I32(i) => i.to_le_bytes(),
            Self::Float(f) => f.to_le_bytes(),
        }
    }

    #[cfg(feature = "flat-endpoints")]
    pub fn try_from_json(value: &serde_json::Value, kind: ValueKind) -> Option<Self> {
        match kind {
            ValueKind::Bool => value.as_bool().map(Self::Bool),
            ValueKind::U8 => value.as_u64().map(|b| Self::U8(b as u8)),
            ValueKind::I8 => value.as_i64().map(|b| Self::I8(b as i8)),
            ValueKind::U16 => value.as_u64().map(|b| Self::U16(b as u16)),
            ValueKind::I16 => value.as_i64().map(|b| Self::I16(b as i16)),
            ValueKind::U32 => value.as_u64().map(|b| Self::U32(b as u32)),
            ValueKind::I32 => value.as_i64().map(|b| Self::I32(b as i32)),
            ValueKind::Float => value
                .as_number()
                .and_then(|n| n.as_f64())
                .map(|f| Self::Float(f as f32)),
        }
    }
}

/// Arbitrary parameter value kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueKind {
    Bool,
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    Float,
}

impl TryFrom<&serde_json::Value> for ValueKind {
    type Error = ();

    fn try_from(value: &serde_json::Value) -> Result<Self, Self::Error> {
        let Some(string) = value.as_str() else {
            return Err(());
        };

        Ok(match string {
            "bool" => Self::Bool,
            "uint8" => Self::U8,
            "int8" => Self::I8,
            "uint16" => Self::U16,
            "int16" => Self::I16,
            "uint32" => Self::U32,
            "int32" => Self::I32,
            "float" => Self::Float,
            _ => return Err(()),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_to_bytes() {
        let value = Value::Float(1.234);
        assert_eq!(value.to_le_bytes(), [0xb6, 0xf3, 0x9d, 0x3f]);
    }

    #[test]
    #[cfg(feature = "flat-endpoints")]
    fn value_from_json() {
        Value::try_from_json(&serde_json::json!(true), ValueKind::Bool).unwrap();
        Value::try_from_json(&serde_json::json!(13), ValueKind::U8).unwrap();
        Value::try_from_json(&serde_json::json!(-13), ValueKind::I8).unwrap();
        Value::try_from_json(&serde_json::json!(13), ValueKind::U16).unwrap();
        Value::try_from_json(&serde_json::json!(-13), ValueKind::I16).unwrap();
        Value::try_from_json(&serde_json::json!(13), ValueKind::U32).unwrap();
        Value::try_from_json(&serde_json::json!(-13), ValueKind::I32).unwrap();
        Value::try_from_json(&serde_json::json!(0.0), ValueKind::Float).unwrap();
    }

    #[test]
    fn sdo_reply_float() {
        // watchdog_timeout = 0.1, as sent by an ODrive
        let reply = SdoReply::new(&[0x00, 0x0b, 0x01, 0x00, 0xcd, 0xcc, 0xcc, 0x3d]).unwrap();
        assert_eq!(reply.endpoint(), 267);
        assert!(matches!(reply.value(ValueKind::Float), Value::Float(v) if v == 0.1));
    }

    #[test]
    fn sdo_reply_short_bool() {
        // enable_watchdog = true: the ODrive sends a 5 byte reply for a bool
        let reply = SdoReply::new(&[0x00, 0x0c, 0x01, 0x00, 0x01]).unwrap();
        assert_eq!(reply.endpoint(), 268);
        assert!(matches!(reply.value(ValueKind::Bool), Value::Bool(true)));
    }

    #[test]
    fn sdo_reply_json_id() {
        // Reply to request_json_id from an ODrive Pro on firmware 0.6.12
        let reply = SdoReply::new(&[0x00, 0x00, 0x00, 0x00, 0x04, 0xd8, 0x33, 0x97]).unwrap();
        assert_eq!(reply.endpoint(), 0);
        assert!(matches!(
            reply.value(ValueKind::U32),
            Value::U32(0x9733_d804)
        ));
    }

    #[test]
    fn sdo_reply_invalid_length() {
        assert!(SdoReply::new(&[0x00, 0x0b, 0x01]).is_none());
        assert!(SdoReply::new(&[0; 9]).is_none());
    }

    #[test]
    fn feedback_decodes_short_sdo_reply() {
        let frame =
            CanFrame::new(Id::new(1, 0x05).unwrap(), &[0x00, 0x0c, 0x01, 0x00, 0x01]).unwrap();
        let (node, message) = feedback(&frame).unwrap();
        assert_eq!(node, 1);
        assert!(matches!(message, Message::SdoReply(reply) if reply.endpoint() == 268));
    }
}
