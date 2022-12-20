#!/usr/bin/env python3

from scapy.all import *
from scapy.fields import Field, ByteEnumField, ByteField, ConditionalField, LEShortField, StrFixedLenField, \
    XByteField, XStrLenField
from scapy.layers.inet import TCP
from scapy.packet import Raw, bind_layers


class MAVLinkMessageID(Field):
    def __init__(self, name, default):
        Field.__init__(self, name, default, '<I')

    def addfield(self, pkt, s, val):
        value = []
        value[0] = int(val & 0xff)
        value[1] = int((val & 0xff00) / 0x0100)
        value[2] = int((val & 0xff0000) / 0x010000)
        return s + struct.pack('BBB', value[0], value[1], value[2])

    def getfield(self, pkt, s):
        return s[3:], self.m2i(pkt,
                               struct.unpack(self.fmt, s[:3] + b'\x00')[0])  # MAVLink 2.0 has 3-byte message ID


class MAVLink(Raw):
    name = 'MAVLink'
    fields_desc = [
        ByteEnumField('STX', None, {0xfd: '(0xfd) MAVLink v2.0'}),
        ByteField('LEN', None),
        ConditionalField(XByteField('INC_FLAGS', None), lambda pkt: pkt.STX == 0xfd),
        ConditionalField(XByteField('CMP_FLAGS', None), lambda pkt: pkt.STX == 0xfd),
        ByteField('SEQ', None),
        XByteField('SYS_ID', None),
        XByteField('COMP_ID', None),
        # ThreeBytesField('MSG_ID', None),
        MAVLinkMessageID('MSG_ID', None),
        XStrLenField('PAYLOAD', None, length_from=lambda pkt: pkt.LEN),
        LEShortField('CHECKSUM', None),
        ConditionalField(StrFixedLenField('SIGNATURE', None, length=13),
                         lambda pkt: pkt.STX == 0xfd and (pkt.INC_FLAGS & 0x01) > 0x00),
    ]

    def __init__(self, _pkt=b'', index=0, **kwargs):
        Raw.__init__(self, _pkt, index, kwargs)


bind_layers(TCP, MAVLink, sport=5760)
bind_layers(TCP, MAVLink, dport=5760)


def print_mavlink_message(pkt):
    if MAVLink in pkt:
        pkt.show()

# Sniff UDP query packets and invoke print_mavlink_message().
f = 'tcp and dst port 5760'
pkt = sniff(iface='lo', filter=f, prn=print_mavlink_message)
