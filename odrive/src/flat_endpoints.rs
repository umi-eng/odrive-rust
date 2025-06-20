//! # Flat Endpoints
//!
//! To allow reading and writing arbitrary configuration from the ODrive, there
//! exists a `flat_endponts.json` file to acompany each firmware release that
//! maps a flattened tree of configuration endpoints to their respectivie
//! identifiers and value types.
//!
//! The `flat_endpoints.json` files can be found on the firmware [downloads
//! page](https://docs.odriverobotics.com/releases/firmware).
//!
//! This module is enabled with the `flat-endpoints` feature which will also
//! bring in `serde_json` which is used to parse the endpoints file.

use crate::can::ValueKind;
use std::collections::HashMap;

/// Flattened endpoints store.
#[derive(Debug, Clone)]
pub struct FlatEndpoints(HashMap<String, (u64, ValueKind)>);

impl FlatEndpoints {
    pub fn from_json(input: serde_json::Value) -> Option<Self> {
        let Some(endpoints) = input.get("endpoints").and_then(|ep| ep.as_object()) else {
            return None;
        };

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

        Some(Self(map))
    }

    /// Get a flattened endpoint from its name.
    ///
    /// Returns (id, type).
    pub fn get(&self, name: &str) -> Option<(u64, ValueKind)> {
        self.0.get(name).copied()
    }

    /// Access the map of endpoints.
    pub fn endpoints(&self) -> &HashMap<String, (u64, ValueKind)> {
        &self.0
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
}
