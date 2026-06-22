DCCEX protocol
===

We assume some things:

The CS did get an IP (for example via DHCP) and in these examples that
is 192.168.0.2.

The clients did get an IP (for example via DHCP) and in these examples
that is 192.168.0.22.

The client is sending commands like `<t>`, `<T>` and `<#>`

The server is sending answers like `<l>`, `<H>` and `<# 17>`

The CS does announce its IP and its services via mDNS.

We have the following services to announce:

TODO: define service names

* CS hostname
* withrottle port (server) over TCP [2560]
* dccex port (server) over TCP [2560]
* dccex port (server) over UDP [2560]
* dccex port (client) over UDP for answers [2561]

We may have to announce:

* the multicast addr for answers?
* the multicast addr for rebroadcasts?
* the multicast port for rebroadcasts?

Once we have all the IP and ports set, communication is like:

TCP (withrottle and dccex)
---

```
client                            server
192.168.0.22                     192.168.0.2
           10239 ---TCP--->  2560
           10239 <--TCP----  2560
```

10239 is some number the TCP stack does make up when opening the
connection. The TCP conection comes already as a bidirectional pair
when it is set up as a client server connection.


UDP (dccex)
---

Make an equivalent connection via UDP

```
client                            server
192.168.0.22                     192.168.0.2
         unknown ---UDP--->  2560
            2561 <--UDP----  unknown
239.255.254.254                  192.168.0.2
            2561 <--UDP----  unknown
```

There is no multicast traffic for commands. So this is NOT used:

```
client                            server
192.168.0.22                     239.255.254.254
         unknown ---UDP--->  2560
```
