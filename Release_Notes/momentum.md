New Momentum feature notes:

The command station can apply momentum to throttle movements in the same way that a standards compliant DCC decoder can be set to do. This momentum can be defaulted system wide and overridden on individual locos. It does not use or alter the loco CV values and so it also works when driving DC locos.
The momentum is applied regardless of the throttle type used (or even EXRAIL). 

Momentum is specified in mS / throttle_step.

There is a new command `<m cabid accelerating [brake]>`
where the brake value defaults to the accelerating value.

For example: 
`<m 3 0>`   sets loco 3 to no momentum.
`<m 3 21>`   sets loco 3 to 21 mS/step.
`<m 3 21 42>`   sets loco 3 to 21 mS/step accelerating and 42 mS/step when decelerating.

`<m 0 21>`  sets the default momentum to 21mS/Step for all current and future locos that have not been specifically set.
`<m 3 -1>`   sets loco 3 to track the default momentum value. 

EXRAIL
  A new macro `MOMENTUM(accel [, decel])` sets the momentum value of the current tasks loco ot the global default if loco=0. 

Note: Setting Momentum 7,14,21 etc is similar in effect to setting a decoder CV03/CV04 to 1,2,3.   

As an additional option, the momentum calculation is based on the 
difference in throttle setting and actual speed. For example, the time taken to reach speed 50 from a standing start would be less if the throttle were set to speed 100, thus increasing the acceleration. 

`<m LINEAR>` - acceleration is uniform up to selected throttle speed.
`<m POWER>`  - acceleration depends on difference between loco speed and selected throttle speed.
 