import threading

from can import Message

from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2
from piper_sdk.piper_msgs.msg_v2 import (
    ArmMsgFeedbackStatusEnum,
    ArmMsgGripperCtrl,
    ArmMsgJointMitCtrl,
    ArmMsgMotionCtrl_2,
    ArmMsgType,
    PiperMessage,
)
from piper_sdk.protocol.protocol_v2 import C_PiperParserV2


class FakeCan:
    class CAN_STATUS:
        SEND_MESSAGE_SUCCESS = object()

    def __init__(self):
        self.sent_messages = []

    def SendCanMessage(self, arbitration_id, data):
        self.sent_messages.append((arbitration_id, data))
        return self.CAN_STATUS.SEND_MESSAGE_SUCCESS


def encode_joint_mit(torque_bits):
    parser = C_PiperParserV2()
    torque_ref = 0x7FF if torque_bits == 12 else 0x7F
    tx_can = Message()
    msg = PiperMessage(
        type_=ArmMsgType.PiperMsgJointMitCtrl_1,
        arm_joint_mit_ctrl=ArmMsgJointMitCtrl(
            pos_ref=0x7FFF,
            vel_ref=0x7FF,
            kp=0x051,
            kd=0x947,
            t_ref=torque_ref,
            torque_bits=torque_bits,
        ),
    )

    parser.EncodeMessage(msg, tx_can)

    return tx_can.data


def make_interface(firmware_data):
    interface = C_PiperInterface_V2.__new__(C_PiperInterface_V2)
    interface._C_PiperInterface_V2__parser = C_PiperParserV2()
    interface._C_PiperInterface_V2__arm_can = FakeCan()
    interface._C_PiperInterface_V2__firmware_data_mtx = threading.Lock()
    interface._C_PiperInterface_V2__firmware_data = bytearray(firmware_data)
    interface._C_PiperInterface_V2__firmware_version_tuple = None
    return interface


def test_arm_motion_ctrl_accepts_v188_mit_mode():
    msg = ArmMsgMotionCtrl_2(move_mode=0x06)

    assert msg.move_mode == 0x06


def test_joint_mit_12bit_encoding_uses_no_crc_layout():
    assert encode_joint_mit(12) == [
        0x7F,
        0xFF,
        0x7F,
        0xF0,
        0x51,
        0x94,
        0x77,
        0xFF,
    ]


def test_joint_mit_8bit_encoding_keeps_crc_layout():
    assert encode_joint_mit(8) == [
        0x7F,
        0xFF,
        0x7F,
        0xF0,
        0x51,
        0x94,
        0x77,
        0xFD,
    ]


def test_motion_ctrl_maps_mit_mode_for_v188_firmware():
    interface = make_interface(b"S-V1.8-8")

    interface.MotionCtrl_2(move_mode=0x04, is_mit_mode=0xAD)

    assert interface._C_PiperInterface_V2__arm_can.sent_messages == [
        (0x151, [0x01, 0x06, 50, 0xAD, 0x00, 0x00, 0x00, 0x00])
    ]


def test_firmware_parser_handles_two_digit_patch_versions():
    interface = make_interface(b"main=S-V1.8-10;motor=M-V1.2-3")

    assert interface.GetPiperFirmwareVersion() == "S-V1.8-10"


def test_feedback_status_accepts_v188_mit_mode():
    parser = C_PiperParserV2()
    message = PiperMessage()
    frame = Message(
        arbitration_id=0x2A1,
        data=[0x01, 0x00, 0x06, 0, 0, 0, 0, 0],
    )

    parser.DecodeMessage(frame, message)

    assert (
        message.arm_status_msgs.mode_feed
        == ArmMsgFeedbackStatusEnum.ModeFeed.MOVE_MIT
    )


def test_gripper_ctrl_accepts_angle_mode_status_codes():
    msg = ArmMsgGripperCtrl(status_code=0x05)

    assert msg.status_code == 0x05


def test_gripper_feedback_decodes_mode_byte():
    parser = C_PiperParserV2()
    message = PiperMessage()
    frame = Message(arbitration_id=0x2A8, data=[0, 0, 0, 0, 0, 0, 0, 0x01])

    parser.DecodeMessage(frame, message)

    assert message.gripper_feedback.mode == "angle"


def test_joint_mit_uses_12bit_torque_for_v188_firmware():
    interface = make_interface(b"S-V1.8-8")

    interface.JointMitCtrl(1, 0.0, 0.0, 10.0, 0.8, 0.0)

    assert interface._C_PiperInterface_V2__arm_can.sent_messages == [
        (0x15A, [0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x77, 0xFF])
    ]


def test_joint_mit_uses_8bit_torque_before_v188_firmware():
    interface = make_interface(b"S-V1.8-7")

    interface.JointMitCtrl(1, 0.0, 0.0, 10.0, 0.8, 0.0)

    assert interface._C_PiperInterface_V2__arm_can.sent_messages == [
        (0x15A, [0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x77, 0xFD])
    ]
