# Consist develeopment spec/checklist

## creating and managing

- ```<^ leadId followerId [id...] >```  creates a CS consist up to  (parser limit) locos  with first id being lead loco. [DONE]

- check each parameter is valid dcc range. [DONE]

- must catch same loco in list twice [DONE]

- negative loco ids in consist mean direction reversed so setting the flag in locoslot. [DONE]

- lead loco could be real or virtual, we couldnt tell but functions would be sent to lead loco address.[DONE]

- <^ leadid> means delete consist led by this loco

- If any follower is already in another consist, error.[DONE]

- all locos in consist are estopped during consist construction.

- <! forget command on any loco will delete a consist chain [DONE]

- `<^ locoid>`  on existing consist lead deletes the consist chain (doesnt create a 1 loco consist :) [DONE]

- ```<^>``` lists all consists [DONE]

- ```<D CABS>``` includes consist chains

- consist deletion will estop all locos in consist. 

- Locoslots contain nextConsist, leadConsist, consistInvertedDirection [DONE]

- lead loco has null leadConsist but !null nextConsist[DONE]

- Throttle commands to following locos are routed to the lead. [NO] becaused its impossible to know whether the sent command has allowed for the reversal of the addressed loco or not.

- Instead, throttle commands to the followers are ignored.[DONE]

- Throttle commands to lead loco are reflected directly to the targetSpeed of the followers. [DONE]

- Broadcast of lead loco broadcasts all followers [DONE]

- Multi loco broadcast should be transmission per throttle rather then one per loco per throttle.

- Following locos do NOT issue DCC speed packets.[DONE]

- Lead loco throttle commands / reminders etc queue DCC packets for all locos simultaneoulsy. [DONE]

- reminder loop ignores following loco speed reminders. [DONE]
(Lead reminder will be sent to all follower addresses, allowing for reversed locos.)

- momentum, speed limits etc will effectively come from the lead loco.[DONE]

- Functions to lead loco are not mapped to followers...[DONE] (Or do we want a way of configuring this)
