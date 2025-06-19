use std::io;

use odrive::can::ODrive;
use socketcan::tokio::CanSocket;

#[tokio::main]
async fn main() -> io::Result<()> {
    // Setup the ODrive connection
    let socket = CanSocket::open("can0")?;
    let odrive = ODrive::new(socket, 1);

    // Get the version information
    let version = odrive.get_version().await?;
    println!("{:?}", version);

    // Get any active errors
    let errors = odrive.get_error().await?;
    println!("{:?}", errors);

    // Get position estiimate
    let estimates = odrive.get_encoder_estimates().await?;
    println!("{:?}", estimates);

    // Get motor current
    let iq = odrive.get_iq().await?;
    println!("{:?}", iq);

    // Temperature readings
    let temp = odrive.get_temperature().await?;
    println!("{:?}", temp);

    // Bus voltage and current
    let bus = odrive.get_bus_voltage_current().await?;
    println!("{:?}", bus);

    // Torque values
    let torques = odrive.get_torques().await?;
    println!("{:?}", torques);

    Ok(())
}
