# Changelog
All notable changes to this project will be documented in this file.

## [0.2.0] - 2025-01-24
### Added
- Bus::get_eoi and Bus::clear_eoi functions
- Removed BusCommand::trace_message
- Xum1541Error::DeviceAccessKind::NoDevice, for other libaries to use when there is no Xum1541 connected to
- Added Xum1541Error::to_errno()
- Add serde serialization and deserialization or error types

### Changed

- Change Xum1541ErrorUsb to a serializable form

### Fixed
- Corrected bus state updates on Open - should be no state update

## [0.1.0] - 2025-01-22
### Added
- Initial release
