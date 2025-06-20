# ODrive

This crate provides an interface that host systems can use to interface with
ODrive devices.

Currently, only the CAN interface is supported. Contributions for other
interfaces like USB are welcome.

## Features

- `can` enables the CAN interface using `socketcan` and `tokio`.
- `flat-endpoints` enables parsing a `flat_endpoints.json` file to discover
  arbitrary configuration endpoints.

## Examples

Connect to an ODrive using CAN bus and I node id of 1.

```rust no_run
let socket = socketcan::tokio::CanSocket::open("can0").unwrap();
let odrive = odrive::can::ODrive::new(socket, 1);
```

Show the bus voltage.

```rust no_run
# tokio_test::block_on(async {
# let socket = socketcan::tokio::CanSocket::open("can0").unwrap();
# let odrive = odrive::can::ODrive::new(socket, 1);
let bus = odrive.get_bus_voltage_current().await.unwrap();
let voltage = bus.voltage;
println!("Bus voltage is: {}V", voltage);
# });
```
