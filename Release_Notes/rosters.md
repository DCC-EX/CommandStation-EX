# Roster

The Roster is a list of loco dcc addresses, with names for convenience and a list of the function keys that are supported by the loco decoder.
 
## How to create a Roster

There are three ways to create or maniplulate a roster

### Roster in EXRAIL (as per previosu versions)

Roster entries in EXRAIL are compiled into the command station and therefore afre backward compatible with previous versions. On startup the EXRAIL ROSTER entries are loaded into RAM before any locally saved roster entries are loaded from non-volatile storage on the command station.

   (TODO copy exrail roster doc here)

## Roster commands

There are a selection of roster commands which are normally used by throttles to extract roster information from the command station.

`<JR>` returns a list of the DCC loco addresses in the roster, e.g.
`<jR 3 5 100>`

`<JR 100>` Returns details for DCC address 100 e.g.
`<jR 100 "Thomas" "Lights/*Whistle/Squeal">`
The functions are separated by '/' and a '*' prefix indicates that the function is momentary rather than toggled.
A request for a roster that does not exist (eg `<JR 999>`) is returned as `<jR 999 X>`

New Roster commands for v5.9.13 and above:

`<JR id "name" "functions">`  adds or replaces a roster in RAM.

`<JR id DELETE>`  removes a roster entry from ram.

`<JR SAVE>`  saves the current rosters in ram. They will be reloaded again at startup.

## Roster editing with a browser

When using the browser dialog on the command station, the Config.. Roster Editor menu will show a roster editing dialog. This edits and saves roster entries in RAM.
