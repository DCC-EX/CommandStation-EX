typedef enum _MDNSServiceProtocol_t 
{
  MDNSServiceTCP,
  MDNSServiceUDP
} MDNSServiceProtocol_t;

class MDNS {
public:
  MDNS(EthernetUDP& udp);
  ~MDNS();
  int begin(const IPAddress& ip,  char* name);
  int addServiceRecord(const char* name, uint16_t port, MDNSServiceProtocol_t proto);
  void run();
private:
  EthernetUDP *_udp;
  IPAddress _ipAddress;
  char* _name;
  char* _serviceName;
  int _servicePort;
};
