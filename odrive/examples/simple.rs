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

    Ok(())
}
