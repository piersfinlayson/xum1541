Restructure - commands and transfer not really used
See what other capabilities I need
Create more tests to make sure it actually works

When drive not turned on I get
[2025-01-20T21:03:22Z TRACE xum1541::device] Retrieved status 0000
[2025-01-20T21:03:22Z TRACE xum1541::bus] Entered Bus::talk
[2025-01-20T21:03:22Z DEBUG xum1541::device] Wait status
[2025-01-20T21:03:22Z TRACE xum1541::device] Read bulk endpoint 0x83
[2025-01-20T21:03:22Z DEBUG xum1541::device] Got status value 0x0000
[2025-01-20T21:03:22Z TRACE xum1541::device] Retrieved status 0000
[2025-01-20T21:03:22Z TRACE xum1541::bus] Entered Bus::read
[2025-01-20T21:03:22Z TRACE xum1541::bus] buf.len(): 1
[2025-01-20T21:03:22Z TRACE xum1541::bus] size: 1
[2025-01-20T21:03:23Z DEBUG xum1541::device] Read 0 bytes
[2025-01-20T21:03:23Z TRACE xum1541::bus] Entered Bus::untalk
[2025-01-20T21:03:23Z DEBUG xum1541::device] Wait status
[2025-01-20T21:03:23Z TRACE xum1541::device] Read bulk endpoint 0x83
[2025-01-20T21:03:23Z DEBUG xum1541::device] Got status value 0x0000
[2025-01-20T21:03:23Z TRACE xum1541::device] Retrieved status 0000
Drive type at device 8: Unknown: Unknown device: 0000
Error: DeviceError { device: 8, message: "Invalid status format: \0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0" }