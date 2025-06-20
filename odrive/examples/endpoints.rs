use odrive::flat_endpoints::FlatEndpoints;
use std::io;

#[tokio::main]
async fn main() -> io::Result<()> {
    // endpoints.json is just a snippet of a full flat_endpoints.json file.
    println!("Reading endpoint file");
    let file = std::fs::File::open("examples/endpoints.json")?;
    let reader = io::BufReader::new(file);
    let endpoints = serde_json::from_reader(reader)?;
    println!("Parsing endpoint file");
    let flat_endpoints = FlatEndpoints::from_json(endpoints).unwrap();

    println!("Retrieving endpoint \"bootloader_version\"");
    let (id, kind) = flat_endpoints.get("bootloader_version").unwrap();
    println!("Got id: {}, kind: {:?}", id, kind);

    Ok(())
}
