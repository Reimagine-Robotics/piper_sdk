import logging
import threading
from concurrent import futures
from types import SimpleNamespace

import pytest

from piper_sdk import C_PiperInterface_V2
from piper_sdk.piper_msgs.msg_v2 import ArmMsgType


def block(node):
    data = bytearray(80)
    data[:8] = b'H-V1.2.0'
    data[32:40] = b'JXB2_DRV'
    data[60:66] = b'V1.0.7'
    data[76:80] = str(node).encode().ljust(4, b'\x00')
    return data


@pytest.fixture
def interface():
    arm = object.__new__(C_PiperInterface_V2)
    arm.logger = logging.getLogger(__name__)
    arm._C_PiperInterface_V2__firmware_data_mtx = threading.Lock()
    arm._C_PiperInterface_V2__firmware_data = bytearray()
    arm._C_PiperInterface_V2__node_firmware_data_mtx = threading.Condition()
    arm._C_PiperInterface_V2__firmware_query_mtx = threading.RLock()
    arm._C_PiperInterface_V2__node_firmware_data = {}
    arm._C_PiperInterface_V2__arm_can = SimpleNamespace(
        CAN_STATUS=SimpleNamespace(SEND_MESSAGE_SUCCESS=None),
    )
    return arm


def receive(arm, node, data):
    arm._C_PiperInterface_V2__UpdatePiperFirmware(SimpleNamespace(
        type_=ArmMsgType.PiperMsgFirmwareRead, firmware_node=node,
        firmware_data=data,
    ))


def test_instant_sweep_and_repeated_master(interface):
    sent = []

    def send(can_id, data):
        sent.append((can_id, data[0]))
        if can_id == 0x422:
            return
        node = can_id - 0x4A0
        receive(interface, node, bytes(data))  # Request echo.
        receive(interface, node, b'\x91\x56')  # Late tail before the new header.
        payload = block(node)
        for offset in range(0, 80, 8):
            receive(interface, node, payload[offset:offset + 8])
        receive(interface, node, b'\x91\x56')  # Joint's short unknown tail.
        receive(interface, node, bytes(8))

    interface._C_PiperInterface_V2__arm_can.SendCanMessage = send
    assert interface.GetMotorDriverVersions((1, 2, 15), 0) == {
        1: 'V1.0.7', 2: 'V1.0.7', 15: 'V1.0.7',
    }
    assert sent == [(0x422, 2), (0x4A1, 1), (0x4A2, 1), (0x422, 0), (0x4AF, 1)]
    for _ in range(2):
        interface.SearchPiperFirmwareVersion()
        assert interface.GetNodeFirmwareInfo(15)['software_version'] == 'V1.0.7'
    assert all(len(data) == 80 for data in
               interface._C_PiperInterface_V2__node_firmware_data.values())


def test_wait_notification_and_timeout(interface):
    requested = threading.Event()
    sent = []

    def send(can_id, data):
        sent.append((can_id, data[0]))
        if can_id != 0x422:
            requested.set()

    interface._C_PiperInterface_V2__arm_can.SendCanMessage = send
    with futures.ThreadPoolExecutor() as executor:
        result = executor.submit(interface.GetMotorDriverVersions, (1,), 1)
        assert requested.wait(1)
        payload = block(1)
        for offset in range(0, 80, 8):
            receive(interface, 1, payload[offset:offset + 8])
        assert result.result(1) == {1: 'V1.0.7'}
    assert interface.GetMotorDriverVersions((1,), 0.01) == {}
    assert interface.GetNodeFirmwareInfo(1) is None
    assert sent == [(0x422, 2), (0x4A1, 1), (0x422, 0)] * 2


def test_scan_exit_on_send_error(interface):
    sent = []

    def send(can_id, data):
        sent.append((can_id, data[0]))
        if can_id != 0x422:
            raise RuntimeError('send failed')

    interface._C_PiperInterface_V2__arm_can.SendCanMessage = send
    with pytest.raises(RuntimeError, match='send failed'):
        interface.GetMotorDriverVersions((1,), 0)
    assert sent[-1] == (0x422, 0)
