
BLINK(vpin, onMs,offMs)

which will start a vpin blinking until such time as it is SET, RESET or set by a signal operation such as RED, AMBER, GREEN. 

BLINK returns immediately, the blinking is autonomous. 

This means a signal that always blinks amber could be done like this:
```
SIGNAL(30,31,32)
ONAMBER(30) BLINK(31,500,500) DONE
```
The RED or GREEN calls will turn off the amber blink automatically.

Alternatively a signal that has normal AMBER and flashing AMBER could be like this:

#define FLASHAMBER(signal) \
                AMBER(signal) \
                BLINK(signal+1,500,500)
  
  (Caution: this issumes that the amber pin is redpin+1)

  ==

  FTOGGLE(function)
   Toggles the current loco function (see FON and FOFF)

  XFTOGGLE(loco,function)
     Toggles the function on given loco. (See XFON, XFOFF)

  TOGGLE_TURNOUT(id) 
      Toggles the turnout (see CLOSE, THROW)

  STEALTH_GLOBAL(code) 
      ADVANCED C++ users only.
      Inserts code such as static variables and functions that 
      may be utilised by multiple STEALTH operations.
       