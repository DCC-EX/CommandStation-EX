// Node traffic command parsing.
ZZBEGIN
ZZ(T,turnoutid,bit)  // Turnout throw/close (1=thrown, 0=closed)
// This is sent by a node wanting to change a turnout it doesnt own.
 auto t=Turnout::get(turnoutid);
 if (!t) return true; // not my turnout, ignore
 if (bit == t->isThrown()) return true; // no change, ignore
 t->setThrown(bit);
  // Considering the T command above...
  // If a Throw/close has originated on the 
ZZEND
