// setup for sensorCAM on an ESP32-CAM NUMDigitalPins <= 80
// assume 700 is first vpin (set with ...CREATE(700,80,0x11)
// the optional SETUP operations below initiate jmri monitoring of sensors for any change of state
// Mostly only useful during debug of initial system but load up CS with extra work. Use judiciously
//         id vPin	      	    
SETUP("<Z 100 700 0>");     // set up for control OUTPUT on vpin #00
// start of up to 80 sensors numbered bsNo's 100 to 197 (OCT) (0/0 to 9/7)
SETUP("<S 100 700 0>");     // first sensor (S00) (reference) 
SETUP("<S 101 701 0>");    
SETUP("<S 102 702 0>"); 
//        as many as you want.  You can add later manually with CS native commands       	 
SETUP("<S 107 707 0>");     
SETUP("<S 110 708 0>");     // Note: suggested id is b/s format (~OCT); vpin is DEC.
SETUP("<S 111 709 0>");     // myFilter.cpp REQUIRES this relationship for bsNo to vPin conversion
SETUP("<S 112 710 0>");
SETUP("<S 113 711 0>");
SETUP("<S 114 712 0>");
//etc.                      // can create a bulk set of sensors with c++ code so:
//for(uint16_t b=2; b<=9;b++) for(uint16_t s=0;s<8;s++) Sensor::create(100+b*10+s,700+b*8+s,1);             
//SETUP("<S 120 716 0>");
SETUP("<S 121 717 0>");
SETUP("<S 122 718 0>");
//SETUP("<S 123 719 0>");
//
SETUP("<S 181 765 0>");
SETUP("<S 191 773 0>");
