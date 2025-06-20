use odrive::{can::ODrive, flat_endpoints::FlatEndpoints};
use serde_json::json;
use socketcan::tokio::CanSocket;
use std::io;

#[tokio::main]
async fn main() -> io::Result<()> {
    // Setup the ODrive connection
    let socket = CanSocket::open("can0")?;
    let odrive = ODrive::new(socket, 1);

    // Get our endpoints reference
    let file = std::fs::File::open("examples/endpoints.json")?;
    let reader = io::BufReader::new(file);
    let endpoints = serde_json::from_reader(reader)?;
    let endpoints = FlatEndpoints::from_json(endpoints).unwrap();
    println!("Endpoints loaded");

    // Our configuration we want to apply
    let config = json!({
        "can.config.protocol": 1,
    });

    odrive.apply_configuration(&endpoints, &config).await?;
    println!("Configuration applied");

    Ok(())
}
