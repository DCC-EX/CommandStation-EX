#ifndef LCDDisplay_h
#define LCDDisplay_h


// This class is created in LCDisplay_Implementation.h

class LCDDisplay : public Print {

  public:
    static LCDDisplay* lcdDisplay; 
    LCDDisplay();

    void clear();
    void display();
    void setRow(byte line); 
    virtual size_t write(uint8_t b);
    using Print::write;  
};

#endif
