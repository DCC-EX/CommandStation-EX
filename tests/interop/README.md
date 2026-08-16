# CommandStation protocol interoperability fixtures

This directory is the CommandStation-side contract for the DCC-EX protocol
interoperability suite. The fixtures describe traffic observed by a command
station: client input, command-station replies, and broadcasts delivered to
the clients that subscribed to them.

The fixture set deliberately does not prescribe a protocol-library API. The
sibling `DCCEXProtocol` task consumes these scenarios as a client and may
choose its own test harness.

## Running the contract check

The validator uses only the Python standard library:

```text
python tests/interop/validate_fixtures.py
```

This check is intentionally suitable for CI and review on hosts that cannot
compile Arduino firmware. Hardware-in-the-loop tests should replay the same
scenarios against a real CommandStation and compare normalized line endings.

Each scenario names its required capability and contains ordered client
inputs, per-client replies, and broadcasts. `broadcast_to` is either `all`,
the originating client id, or an explicit list of client ids. A reconnect
scenario starts a new client session and must not inherit the previous
session's pending input.
