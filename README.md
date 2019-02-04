# mavlink_udp_platform
A platform for using UDP/TCP with MAVLINK

MAVLink or Micro Air Vehicle Link is a protocol for communicating with small unmanned vehicle. It is designed as a header-only message marshaling library.

UDP (User Datagram Protocol) is a communications protocol which is used primarily for establishing low-latency and loss-tolerating connections between applications on the internet.

The platform's role is to depacketize motion capture data from optitrack to send the information through UDP in the form of MAVLINK Packets.
