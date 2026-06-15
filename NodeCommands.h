// Node traffic command parsing.
ZZBEGIN
ZZ(H,turnoutid,bit)  // Turnout throw/close (1=thrown, 0=closed)
  // This was sent by a node that changes a turnout state
  // It will not cause a rebroadcast
  Turnout::setClosed(turnoutid,bit==0,false);
ZZEND
