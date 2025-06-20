# CANSimple Protocol

An implementation of the CANSimple protocol mainly used by ODrive products.

## Features

- `defmt-1` enables `defmt` formatting on relevant types.

## Overview

The CANsimple protocol uses only 11-bit identifiers and features a `node` and
`command` components.

```
| 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |
| Node ID                     | Command ID    |
```

A full explanation can be found in the ODrive
[documentation](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#overview).
