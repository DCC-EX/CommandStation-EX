// dummy LCD shim to keep linker happy
  LCDDisplay::LCDDisplay() {} 
  void LCDDisplay::interfake(int p1, int p2, int p3) {(void)p1; (void)p2; (void)p3;}   
  void LCDDisplay::setRowNative(byte row) { (void)row;} 
  void LCDDisplay::clearNative() {}
  void LCDDisplay::writeNative(char * b){ (void)b;} //  
  void LCDDisplay::displayNative(){}
  
