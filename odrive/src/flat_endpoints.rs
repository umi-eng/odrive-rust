//! # Flat Endpoints
//!
//! To allow reading and writing arbitrary configuration from the ODrive, there
//! exists a `flat_endpoints.json` file to accompany each firmware release that
//! maps a flattened tree of configuration endpoints to their respective
//! identifiers and value types.
//!
//! **The endpoint IDs are only valid for the firmware build and hardware the
//! file was generated for.** IDs move between releases as endpoints are added,
//! so a file for another firmware reads and writes the wrong parameters without
//! any error. Use the file from the exact release flashed on your ODrives, and
//! check it with [`FlatEndpoints::matches`] against the [`Version`] each ODrive
//! reports.
//!
//! The `flat_endpoints.json` files can be found on the firmware [downloads
//! page](https://docs.odriverobotics.com/releases/firmware).
//!
//! This module is enabled with the `flat-endpoints` feature which will also
//! bring in `serde_json` which is used to parse the endpoints file.

use crate::can::{ValueKind, Version};
use std::collections::HashMap;

/// Flattened endpoints store, with the firmware and hardware it was generated for.
#[derive(Debug, Clone)]
pub struct FlatEndpoints {
    endpoints: HashMap<String, (u64, ValueKind)>,
    fw_version: Option<String>,
    hw_version: Option<String>,
    crc: Option<u16>,
}

impl FlatEndpoints {
    pub fn from_json(input: serde_json::Value) -> Option<Self> {
        let endpoints = input.get("endpoints").and_then(|ep| ep.as_object())?;

        let mut map = HashMap::new();

        for (name, ep) in endpoints.iter() {
            let Some(kind) = ep.get("type") else {
                continue;
            };
            let Ok(kind) = ValueKind::try_from(kind) else {
                continue;
            };
            let Some(id) = ep.get("id").and_then(|i| i.as_u64()) else {
                continue;
            };

            map.insert(name.to_owned(), (id, kind));
        }

        let text = |key| input.get(key).and_then(|v| v.as_str()).map(str::to_owned);
        let crc = input
            .get("crc")
            .and_then(|v| v.as_u64())
            .and_then(|v| u16::try_from(v).ok());

        Some(Self {
            endpoints: map,
            fw_version: text("fw_version"),
            hw_version: text("hw_version"),
            crc,
        })
    }

    /// Get a flattened endpoint from its name.
    ///
    /// Returns (id, type).
    pub fn get(&self, name: &str) -> Option<(u64, ValueKind)> {
        self.endpoints.get(name).copied()
    }

    /// Access the map of endpoints.
    pub fn endpoints(&self) -> &HashMap<String, (u64, ValueKind)> {
        &self.endpoints
    }

    /// The firmware version the file was generated for, e.g. `0.6.12`.
    pub fn fw_version(&self) -> Option<&str> {
        self.fw_version.as_deref()
    }

    /// The hardware version the file was generated for, e.g. `4.4.58`.
    pub fn hw_version(&self) -> Option<&str> {
        self.hw_version.as_deref()
    }

    /// The CRC of the endpoint list.
    pub fn crc(&self) -> Option<u16> {
        self.crc
    }

    /// Whether this file was generated for the firmware and hardware an ODrive reports.
    ///
    /// Necessary but not sufficient: different builds of the same release (for example
    /// unreleased builds) can report the same version with different endpoint lists. A file
    /// without version information never matches.
    pub fn matches(&self, version: &Version) -> bool {
        let fw = format!(
            "{}.{}.{}",
            version.fw_version_major(),
            version.fw_version_minor(),
            version.fw_version_revision()
        );
        let hw = format!(
            "{}.{}.{}",
            version.hw_version_major(),
            version.hw_version_minor(),
            version.hw_version_variant()
        );
        // Release files can carry a packaging suffix, e.g. "0.6.12-1"
        let file_fw = self.fw_version().and_then(|v| v.split('-').next());

        file_fw == Some(fw.as_str()) && self.hw_version() == Some(hw.as_str())
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn parse_input() {
        let input = json!({"endpoints": {
        "vbus_voltage": {
          "id": 1,
          "type": "float",
          "access": "r"
        }}});

        let endpoints = FlatEndpoints::from_json(input).unwrap();

        assert_eq!(endpoints.get("vbus_voltage"), Some((1, ValueKind::Float)));
    }

    #[test]
    fn parse_version_information() {
        let input = json!({
            "fw_version": "0.6.12-1",
            "hw_version": "4.4.58",
            "crc": 38707,
            "endpoints": {}
        });

        let endpoints = FlatEndpoints::from_json(input).unwrap();

        assert_eq!(endpoints.fw_version(), Some("0.6.12-1"));
        assert_eq!(endpoints.hw_version(), Some("4.4.58"));
        assert_eq!(endpoints.crc(), Some(38707));
    }

    #[test]
    fn matches_version() {
        let file = FlatEndpoints::from_json(json!({
            "fw_version": "0.6.12-1",
            "hw_version": "4.4.58",
            "endpoints": {}
        }))
        .unwrap();

        // protocol, hw major, minor, variant, fw major, minor, revision, unreleased
        let matching = Version::new([2, 4, 4, 58, 0, 6, 12, 0]);
        let other_firmware = Version::new([2, 4, 4, 58, 0, 6, 11, 0]);
        let other_hardware = Version::new([2, 5, 2, 0, 0, 6, 12, 0]);

        assert!(file.matches(&matching));
        assert!(!file.matches(&other_firmware));
        assert!(!file.matches(&other_hardware));
    }

    #[test]
    fn no_version_information_never_matches() {
        let file = FlatEndpoints::from_json(json!({"endpoints": {}})).unwrap();
        assert!(!file.matches(&Version::new([2, 4, 4, 58, 0, 6, 12, 0])));
    }
}
