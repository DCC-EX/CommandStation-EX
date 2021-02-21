//SETUP("D WIFI ON");
//SETUP("D WIT ON");
Wire.begin();
Wire.setClock(400000);

LCD(3,F("192.168.4.1"));
LCD(4,F(__DATE__));
LCD(5,F(__TIME__));
