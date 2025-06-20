use std::io;

use odrive::can::{ODrive, ValueKind};
use socketcan::tokio::CanSocket;

#[tokio::main]
async fn main() -> io::Result<()> {
    // Setup the ODrive connection
    let socket = CanSocket::open("can0")?;
    let odrive = ODrive::new(socket, 1);

    // Read the bus voltage using an SDO read with an endpoint id of 1.
    let vbus = odrive.sdo_read(1, ValueKind::Float).await?;
    println!("VBUS: {:?}", vbus);

    // The given endpoint id for a parameter can be found in the
    // flat_endpoints.json file provided with each ODrive firmware release.

    Ok(())
}
