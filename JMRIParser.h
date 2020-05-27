#ifndef JMRIParser_h
#define JMRIParser_h
struct JMRIParser
{
    static void parse(Stream & stream,const char * command);

   private:
    static const int MAX_PARAMS=10; 
    static int p[MAX_PARAMS];
};
#endif
