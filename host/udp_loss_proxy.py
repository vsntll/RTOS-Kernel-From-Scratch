#!/usr/bin/env python3
"""Phase 7c QoS demo support: a UDP relay that randomly drops datagrams in
both directions between this project's client and a real MicroXRCEAgent,
so host/bench_qos.py can measure best-effort-vs-reliable delivery under
*induced*, known loss instead of whatever the host's loopback interface
happens to do (usually ~0%, which would prove nothing).

Same reasoning as pty_bridge.py (Phase 5): a real agent needs a real
client on the other end to test against, and the thing being measured
(loss/retry behavior) has to happen on the wire between them, not be
simulated inside either endpoint.

Usage:
    python3 udp_loss_proxy.py <listen_port> <agent_ip> <agent_port> <loss_pct> [-v]

Point this project's client at 127.0.0.1:<listen_port> instead of the real
agent; the proxy forwards to <agent_ip>:<agent_port> and back, dropping
each direction independently with probability <loss_pct> (0-100). Pass -v
for a per-packet FWD/DROP trace on stderr -- this is what actually found
the address-consistency and sequence-numbering bugs recorded in
xrce/docs/design.md's Phase 7c section; quiet by default since a real demo
run doesn't need it.
"""
import random
import socket
import sys


def main():
    if len(sys.argv) not in (5, 6):
        print(f"usage: {sys.argv[0]} <listen_port> <agent_ip> <agent_port> <loss_pct> [-v]",
              file=sys.stderr)
        return 2

    listen_port = int(sys.argv[1])
    agent_addr = (sys.argv[2], int(sys.argv[3]))
    loss_pct = float(sys.argv[4])
    verbose = len(sys.argv) == 6 and sys.argv[5] == "-v"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", listen_port))

    client_addr = None
    sent = 0
    dropped = 0

    print(f"udp_loss_proxy: listening on :{listen_port}, forwarding to {agent_addr}, "
          f"dropping {loss_pct}% each direction", file=sys.stderr)

    while True:
        data, src = sock.recvfrom(65535)
        if src == agent_addr:
            dst = client_addr
        else:
            client_addr = src
            dst = agent_addr

        if dst is None:
            continue  # a reply arrived before we ever saw a client packet

        sent += 1
        if random.uniform(0, 100) < loss_pct:
            dropped += 1
            if verbose:
                print(f"DROP {len(data)}B {src} -> {dst}", file=sys.stderr, flush=True)
            continue
        sock.sendto(data, dst)
        if verbose:
            print(f"FWD  {len(data)}B {src} -> {dst}", file=sys.stderr, flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
