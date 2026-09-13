# pi-protocol (vendored, generated)

These four files are the **generated** pi-protocol sources that define the serial
wire format shared with the indiflight flight controller:

    pi-protocol.c / pi-protocol.h    framing / parser
    pi-messages.c  / pi-messages.h   message structs + (de)serialization

They are checked in so the companion-computer build stays a plain `make` with no
codegen or Python step on the target — the same approach used for the generated
CNN encoder in `src/cnn/`.

The library they are generated from is vendored in `ext/pi-protocol/` (source,
message definitions, templates and generator), so this repo does not depend on
the external relay project. `pi-protocol.c` here is a verbatim copy of
`ext/pi-protocol/src/pi-protocol.c`; the other three are produced from
`ext/pi-protocol/config.yaml` + `msgs/*.yaml` (protocol_version 2.0.0, global
mode RXTX).

Messages used by the on-board relay (see `src/relay/`): EKF_INPUTS (RX, for the
`time_us` timestamp), FAKE_GPS, EXTERNAL_POSE, POS_SETPOINT, KEYBOARD,
NN_INPUT_CHUNK (all TX to the FC).

## Regenerating

If the protocol changes, edit `ext/pi-protocol/` (or drop in a newer version)
and run:

    make regen-pi

which regenerates the three generated files here and re-copies `pi-protocol.c`
from the vendored source. **The `config.yaml` version and message set must match
the firmware** — a mismatch silently corrupts framing. Do not hand-edit the
generated files.
