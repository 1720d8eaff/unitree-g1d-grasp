"""Exercise the production framing protocol over real local stream sockets."""
import socket
import struct
import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'onboard/trajectory_bridge'))
from g1_d_trajectory_bridge.ipc_protocol import FramedSocket,ProtocolError,MAX_MESSAGE_BYTES


class IPCBehaviorTests(unittest.TestCase):
    def test_ipc_serialization_roundtrip_and_disconnect(self):
        a,b=socket.socketpair();a.settimeout(1);b.settimeout(1)
        sender,receiver=FramedSocket(a),FramedSocket(b)
        try:
            record={'type':'trajectory','joint_names':['left_elbow_joint'],'points':[[.2]],'timestamp':1.25}
            sender.send(record)
            assert receiver.recv()==record
            sender.close()
            with self.assertRaises(EOFError):receiver.recv()
        finally:sender.close();receiver.close()


    def test_ipc_oversize_header_rejected_before_allocation(self):
        a,b=socket.socketpair();b.settimeout(1)
        receiver=FramedSocket(b)
        try:
            a.sendall(struct.pack('!I',MAX_MESSAGE_BYTES+1))
            with self.assertRaises(ProtocolError):receiver.recv()
        finally:a.close();receiver.close()
