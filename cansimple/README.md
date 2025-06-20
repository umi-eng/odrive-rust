# CANSimple

An implementation of the CANSimple protocol mainly used by ODrive products.

## Features

- `defmt-1` enables [`defmt`](https://crates.io/crates/defmt) formatting on
  relevant types.

## Overview

The CANsimple protocol uses only 11-bit identifiers and features a `node` and
`command` components.

```text
| 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| Node ID                | Command ID        |
```

A full explanation can be found in the ODrive
[documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#overview).

## Examples

Identifiers can be created from either raw ids or from node and command ids.

```rust
// From components
let id = cansimple::Id::new(1, 15).unwrap();

// From raw id
let id = cansimple::Id::from_raw(0x029);
```

CANsimple identifers can be converted to and from
[`embedded_can`](https://crates.io/crates/embedded-can) identifers.

```rust
let id = cansimple::Id::new(1, 15).unwrap();
let embedded_id: embedded_can::Id = id.into();
```
