#!/usr/bin/env python3
"""List DCC-EX throttle or node UDP datagrams without sending discovery probes."""

import argparse
import socket
import sys
from ipaddress import IPv4Address


THROTTLE_PORT = 2560
NODE_PORT = THROTTLE_PORT + 1


def sender_name(ip):
    try:
        return socket.gethostbyaddr(ip)[0]
    except socket.herror:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--station",
        help="command station IPv4 address or resolvable hostname (.local if supported); "
             "join its throttle multicast group at 239.255.255.<last IP octet>:2560",
    )
    mode.add_argument(
        "--node-group", type=int, choices=range(256), metavar="0-255",
        help="listen on 239.255.254.<group>:2561 (factory default group: 254)",
    )
    parser.add_argument(
        "--interface", type=IPv4Address, default=IPv4Address("0.0.0.0"),
        help="local IPv4 address for joining multicast (default: system-selected interface)",
    )
    parser.add_argument(
        "--resolve", action="store_true",
        help="try system reverse hostname lookups (including mDNS if configured; may delay output)",
    )
    args = parser.parse_args()
    if not args.station and args.node_group is None and int(args.interface):
        parser.error("--interface requires --station or --node-group")

    try:
        group = None
        listen_port = THROTTLE_PORT
        if args.node_group is not None:
            group = f"239.255.254.{args.node_group}"
            listen_port = NODE_PORT
        elif args.station:
            station_ip = socket.gethostbyname(args.station)
            group = f"239.255.255.{station_ip.rsplit('.', 1)[1]}"

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
            listener.bind(("", listen_port))
            if group:
                membership = socket.inet_aton(group) + args.interface.packed
                listener.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)

            mode = f"unicast and multicast {group}" if group else "unicast"
            print(f"Listening on UDP port {listen_port} ({mode})", file=sys.stderr, flush=True)
            print("Sender\tBytes\tHex payload", flush=True)
            while True:
                payload, (ip, port) = listener.recvfrom(65535)
                sender = f"{ip}:{port}"
                if args.resolve:
                    name = sender_name(ip)
                    if name:
                        sender += f" ({name})"
                print(f"{sender}\t{len(payload)}\t{payload}", flush=True)
    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        print(f"UDP listener: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
