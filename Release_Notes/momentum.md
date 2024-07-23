New Momentum feature notes:

The command station can apply momentum to throttle movements in the same way that a standards compliant DCC decoder can be set to do. This momentum can be defaulted system wide and overridden on individual locos. It does not use or alter the loco CV values and so it also works when driving DC locos.
The momentum is applied regardless of the throttle type used (or even EXRAIL). 

Momentum is specified in mS / throttle_step.

There is a new command `<m cabid mS>`

For example: 
`<m 3 0>`   sets loco 3 to no momentum.
`<m 3 21>`   sets loco 3 to 21 mS/step.
`<m 0 21>`  sets the default momentum to 21mS/Step for all current and future locos that have not been specifically set.
`<m 3 -1>`   sets loco 3 to track the default momentum value. 

EXRAIL
  A new macro `MOMENTUM(mSecPerStep)` sets the momentum value of the current tasks loco. 

Note: Setting Momentum 7,14,21 etc is similar in effect to setting a decoder CV03 to 1,2,3. At present the same momentum value is used for acceleration and deceleration. The `<m>` command may be extended in future to separate these values. 

