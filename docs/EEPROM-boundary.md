# Persistent settings boundary

`EEStore` owns the persistent-settings contract. Firmware code must use
`EEStore::read`, `EEStore::write`, and `EEStore::capacity`; it must not depend
on a platform's global `EEPROM` object. A future backend may implement those
operations with EEPROM, flash, emulated flash, or another persistent store.

The first record remains byte-for-byte compatible with existing installations:
`EEStoreData` starts with the `DCC++1` identifier followed by the three
16-bit object counts. The record stream following that header is unchanged in
this phase. A backend must provide Arduino `get`/`put` semantics, including
the existing address-based layout and capacity reporting.

This boundary deliberately does not redesign the settings protocol or erase
legacy data. Backend selection and migration can be added independently once a
target platform supplies a persistent implementation.
