#!/bin/bash
# Files to be added to the CS as is
# Session.cpp
# Session.h
# Diag.cpp
# Queue.cpp
# Queue.h

# patches to apply
# file / "marker" / "replace with marker + target" / 0 marker before 1 marker after


patch2=(WifiInboundHandler.cpp "runningClientId);" "Connection::type = _WIFI; Connection::id = runningClientId;" 0)

#main include of the class for handling the CLI session
patch3=(DCCEX.h "#define DCCEX_h" "\n#include \"Session.h\"" 0)

#added testing if the motoshield has been started and thus the Waveform gen is running
patch7=(DCCWaveform.cpp "progTripValue=0;" "bool DCCWaveform::running=false;" 0)
patch13=(DCCWaveform.cpp "interruptHandler);" "\nrunning=true;" 0)
patch8=(DCCWaveform.h "public:" "\nstatic bool isRunning() { return running; }" 0)
patch9=(DCCWaveform.h "private:" "\nstatic bool running;" 0)

#definitions needed for handling latching i.e. sending the diag output to the currentmy 'active' connection
#prepared for WiFi but that is not implemented
patch10=(DIAG.h "StringFormatter::lcd" "\nenum Transport { _WIFI, _ETHERNET}; \
\nstruct Connection { static Transport type;  static byte id;}; \
\nstruct Latch { static Transport type;  static byte id;};" 0) 

#Ethernet Interface changes to get to the connection for sending information to the CLI 
patch11=(EthernetInterface.h "loop();" "\nbool isConnected() { return connected; };\
\nstatic EthernetInterface *get() { return singleton; };\
\nEthernetClient *getClient(int socket) { return \&clients[socket]; };" 0)
patch12=(EthernetInterface.cpp "socket,buffer);" "\nConnection::type = _ETHERNET; Connection::id = socket;" 0)

#Adding a) the LATCH diagnostic command to the parseD; allowing to send set the diag output to the active ethernet
#connection; DOes not work for WiFi and b) handling of the atCommandCallback piggy backing the + command so need check 
#if the Waveform gen has statred as otherwise we try to poweroff a non exisiting motorshield

patch4=(DCCEXParser.cpp "26133;" "\nconst int16_t HASH_KEYWORD_LATCH = 1618;" 0)
patch5=(DCCEXParser.cpp "(atCommandCallback) {" "\nif (DCCWaveform::isRunning()) {" 0)
patch6=(DCCEXParser.cpp "progTrack.setPowerMode(POWERMODE::OFF);" "\n}" 0)
patch14=(DCCEXParser.cpp "case HASH_KEYWORD_CABS:" "\n    case HASH_KEYWORD_LATCH:\
\n        Diag::LATCH = onOff; \
\n        Latch::type = Connection::type; \
\n        Latch::id = Connection::id; \
\n        return true; \n    " 1 )

#StringFormatter : adding things needed for Latching the Wifi or Ethernet connection to reciev the diag output
#remove a bracket on line 47 which will be added again in patch 17; If that is not done we end up with one
#bracket too much; Brittle and prone to issues as they change stuff but so far the best i can get
patch1=(StringFormatter.h "LCN;" "static bool LATCH;" 0)

sed -i -e '47d' StringFormatter.cpp

patch15=(StringFormatter.cpp "LCN=false;" "\nbool Diag::LATCH=false;" 0)
patch16=(StringFormatter.cpp "if (!diagSerial) return;" "\n#if ETHERNET_ON == true || WIFI_ON == true \
\n  auto t = diagSerial;\
\n  if (Diag::LATCH)\
\n  {\
\n    switch (Latch::type)\
\n    {\
\n    case _ETHERNET:\
\n    {\
\n#if ETHERNET_ON == true\
\n      auto i = EthernetInterface::get();\
\n      auto s = i->getClient(Latch::id); \
\n      if (s->connected())\
\n      { \
\n        diagSerial = s;\
\n      }\
\n#endif\
\n      break;\
\n    }\
\n    case _WIFI:\
\n    {\
\n      DIAG(F(\"Latch on Wifi is not possible for now ...\"));\
\n      break;\
\n    }\
\n    }\
\n  }\
\n#endif\n" 1)
patch17=(StringFormatter.cpp "void StringFormatter::lcd" "\n#if ETHERNET_ON == true || WIFI_ON == true \
\n  if (Diag::LATCH)\
\n  {\
\n    diagSerial = t;\
\n  }\
\n#endif\n}\n" 1)

patch=(patch1 patch2 patch3 patch4 patch5 patch6 patch7 patch8 patch9 patch10 patch11 patch12 patch13 patch14 patch15 patch16 patch17)

# patch=(patch17)

declare -n elmv1

for elmv1 in "${patch[@]}"; do
    file="${elmv1[0]}"
    marker="${elmv1[1]}"
    markerpos="${elmv1[3]}"

    if [ $markerpos = 1 ]
    then
        target="${elmv1[2]} $marker"
    else
        target="$marker ${elmv1[2]}"
    fi

    echo $marker
    # echo $target
    echo $file

    grep -q $marker $file
    if [ $? -eq 0 ]
    then
        echo "Patching $file with $marker --> $target ..."
        sed -i "s/$marker/$target/" $file
    else
        echo "Patching $file failed."
        exit 1
    fi

done