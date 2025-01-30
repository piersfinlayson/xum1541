# Changelog
All notable changes to this project will be documented in this file.

## [0.3.0] - 2025-??-??
### Added
- Added [`Device`] as a trait, to allow other device types and easier mocking.  As part of this changed the existing set of functions
- Similarly made [`BusBuilder`] a trait, to allow creation of [`Bus`] using other device types.

### Changed
- Handled xum1541 timeouts better
- Renamed [`Device`] to [`UsbDevice`]
- Renamed [`BusBuilder`] to [`UsbBusBuilder`]
- Removed [`Bus::device_context`]

## [0.2.0] - 2025-01-27
### Added

### Changed
- Downgraded error! to warn! logging - to avoid double error logging problems which the library user will also like log (and likely as error).  Reserve error! for unrecoverable error in xum1541.
- Changed Communication error to take CommunicationKind - to break out different sorts of errors, in particular the StatusValue, which can be used
on [`Bus::talk`] and [`Bus::Listen`] to infer the lack of a particular device.
- Change Xum1541Error to Error

## [0.1.1] - 2025-01-24
### Added
- Bus::get_eoi and Bus::clear_eoi functions
- Removed BusCommand::trace_message
- Xum1541Error::DeviceAccess::NoDevice, for other libaries to use when there is no Xum1541 connected to
- Added Xum1541Error::to_errno()
- Add serde serialization and deserialization or error types

### Changed

- Change Xum1541ErrorUsb to a serializable form

### Fixed
- Corrected bus state updates on Open - should be no state update

## [0.1.0] - 2025-01-22
### Added
- Initial release
