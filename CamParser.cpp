#include "CamParser.h"
#include "FSH.h"
#include "IO_EXSensorCAM.h"


VPIN EXSensorCAM::CAMBaseVpin = 0;

bool CamParser::parseN(Print * stream, byte paramCount, int16_t p[])
{
  (void)stream; // probably unused parameter 
 	
  if (EXSensorCAM::CAMBaseVpin==0) return false; // no cam found
	if (paramCount == 0) return false; 
	VPIN vpin=EXSensorCAM::CAMBaseVpin;
  byte camop=p[0]; // cam oprerator (F is special) 
  if (camop!='F') camop=p[0]-0x20; // lower case the oprerator
  int16_t param1;
	int16_t param2;
  
  switch(paramCount) {    
    case 0: 
      return false;

    case 1:
      if (strchr_P((const char *)F("egrvwxFimt"),camop) == nullptr) return false;
      param1=0;
      param2=0;
      break;
    
    case 2:     //<N code val>  
      if(camop=='c'){ 
        EXSensorCAM::CAMBaseVpin=p[1];
        DIAG(F("CAM base vpin: %c %d "),p[0],p[1]);
        return true;
      }
      if (strchr_P((const char *)F("oalnrsuvimt"),camop) == nullptr) return false;
      
      param1 = p[1];
      param2 = 0;     
      break;
    
    case 3: //<N  vpin rowY colx >
      if (p[1]>236 || p[1]<0) return false;
      if (p[2]>316 || p[2]<0) return false;

      camop='A';  // sepcial case in IO_SensorCAM
      vpin = p[0];    
      param1 = p[1];  
      param2 = p[2];  
      break;
    
    case  4:        //<N a id row col> 
      if (camop!='a') return false;//must start with 'a' 
      
      if (p[1]>80  || p[1]<0) return false;
      if (p[2]>236 || p[2]<0) return false;
      if (p[3]>316 || p[3]<0) return false;

      camop=128+p[1];  // sensor id in camop
      param1=p[2];   // row
      param2=p[3];   // col
      break;

    default:
      return false;
    }
  DIAG(F("Cam: %d %d %c %d"),vpin,param1,camop,param2);
  IODevice::writeAnalogue(vpin,param1,camop,param2);
  return true;
}
