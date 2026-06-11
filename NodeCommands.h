// Node traffic command parsing.
ZZBEGIN
ZZ(T,turnoutid,bit)  // Turnout state change (1=thrown, 0=closed)
 auto t=Turnout::get(turnoutid);
 if (!t) return true; // not my turnout, ignore
 if (bit == t->isThrown()) return true; // no change, ignore
 t->setThrown(bit);
 
 
ZZEND
