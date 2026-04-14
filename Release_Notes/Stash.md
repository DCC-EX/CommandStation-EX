# The STASH feature of exrail.

STASH is used for scenarios where it is helpful to relate a loco id to where it is parked. For example a fiddle yard may have 10 tracks and it's much easier for the operator to select a train to depart by using the track number, or pressing a button relating to that track, rather than by knowing the loco id which may be difficult to see.

Automated yard parking can use the stash to determine which tracks are empty without the need for block occupancy detectors.

Note that a negative locoid may be stashed to indicate that the loco will operate with inverted direction. For example a loco facing backwards, with the INVERT_DRECTION state may be stashed by exrail and the invert state will be restored along with the loco id when using the PICKUP_STASH.  CLEAR_ANY_STASH will clear all references to the loco regardless of direction. 

The following Stash  commands are available:
 | EXRAIL command | Serial protocol | function |
 | -------------- | --------------- | -------- | 
 | STASH(s) | `<JM s locoid>` | Save the current loco id in the stash array element s. |
 | CLEAR_STASH(s) | `<JM s 0>` | Sets stash array element s to zero. |  
 | CLEAR_ALL_STASH | `<JM CLEAR ALL>` | sets all stash entries to zero |
 | CLEAR_ANY_STASH | `<JM CLEAR ANY locoid>` | removes current loco from all stash elements | 
 | PICKUP_STASH(s) | N/A | sets current loco to stash element s |
 | IFSTASH(s)  | N/A | True if stash element s is not zero |
 | N/A | `<JM>`  | query all stashes (returns `<jM s loco>` where loco is not zero)
 | N/A | `<JM stash>` | Query loco in stash (returns `<jM s loco>`)  


