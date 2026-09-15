# Changelog

## [Unreleased]

## v0.2.0

- Update bitflags to v2.13.2.
- Add missing units to doc comments.
- Add feddback message reader.

## v0.1.0

- Initial release.
- Use cansimple dependency from workspace.
- Only pin serde to minor version.
- Update bitflags to v2.13.0.
- Fix length of `set_control_mode` frame.
- Check frame length is correct for `sdo_read` inside response loop.
- Fix documented unit for `set_input_torque`.
- Check SDO write id will fit in message id size for `apply_configuration`.
- Fix typo in function name `set_lmits` -> `set_limits`.
