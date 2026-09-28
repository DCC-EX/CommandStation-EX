# CommandStation metadata contract

CommandStation-EX exposes product, device, and release identity through this
newline-terminated information frame:

```
<iDCC-EX V-5.6.3 / ARDUINO_TYPE / MOTOR_SHIELD G-GITHUB_SHA>
```

Host tools may request it with the side-effect-free `<i>` command. The same
frame remains emitted during startup and as part of the existing `<s>` status
response, preserving compatibility with current clients.

Fields are positional and delimited by ` / `: product, release, device, and
build SHA. Hosts should tolerate future fields and use the release field for
display or compatibility-matrix lookup rather than assuming a fixed version.
