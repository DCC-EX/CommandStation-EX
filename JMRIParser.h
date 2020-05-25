
struct JMRIParser
{
    static void parse(const char * command);

   private:
    static const int MAX_PARAMS=10; 
    static int p[MAX_PARAMS];
};
