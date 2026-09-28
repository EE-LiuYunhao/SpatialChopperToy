# AnyController upstream

This module vendors the reusable `anycontroller` Android library from the internal
`pico-xr-sdk/anyController` repository at commit
`591666d3ef7db8081aeea4ffebd3d9db8dff8e66`. The demo application from that
repository is intentionally not included.

The referenced upstream tree does not contain a standalone LICENSE or NOTICE file.

Local integration changes are limited to project build and quality-gate configuration, public API
documentation, and hiding all detected-plane wireframes after the user selects a controller
surface. The selected surface is still retained as an immutable calibration frame; only its
discovery outline and axis visualization are suppressed. A unit test protects this local
visibility policy.
