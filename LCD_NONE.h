// dummy LCD shim to keep linker happy
  LCDDisplay::LCDDisplay() {}  
  void LCDDisplay::setRow(byte row) { (void)row;} 
  void LCDDisplay::clear() {}
  size_t LCDDisplay::write(uint8_t b){ (void)b; return -1;} //  
  void LCDDisplay::display(){}
  
