import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

FIRMWARE = Path(__file__).resolve().parents[1] / 'firmware'
sys.path.insert(0, str(FIRMWARE))
from frame_decoder import FrameDecoder


class ProtocolTests(unittest.TestCase):
    def test_fragmented_position_keeps_embedded_terminator(self):
        decoder = FrameDecoder()
        packet = bytes([2, 0x36, 1, 0, 0x6B, 0, 0, 0x6B])
        for byte in packet[:-1]:
            self.assertEqual(decoder.feed(bytes([byte])), [])
        self.assertEqual(decoder.feed(packet[-1:]), [(2, 0x36, packet[2:7])])

    def test_noise_and_multiple_replies(self):
        decoder = FrameDecoder(8)
        self.assertEqual(decoder.feed(b'\x00' * 2000), [])
        self.assertLessEqual(len(decoder.buffer), 8)
        data = bytes([2, 0x3A, 4, 0x6B, 2, 0xF3, 2, 0x6B])
        self.assertEqual(decoder.feed(data), [(2, 0x3A, b'\x04'), (2, 0xF3, b'\x02')])

    def test_bad_sign_and_terminator_recover(self):
        decoder = FrameDecoder()
        bad = bytes([2, 0x36, 9, 0, 0, 0, 0, 0x6B, 2, 0x3A, 4, 0])
        good = bytes([2, 0x3A, 8, 0x6B])
        self.assertEqual(decoder.feed(bad + good), [(2, 0x3A, b'\x08')])

    def load_driver(self):
        fake = types.ModuleType('maix')
        fake.uart = types.SimpleNamespace(UART=lambda *args: (_ for _ in ()).throw(AssertionError('hardware initialized on import')))
        fake.pinmap = types.SimpleNamespace(set_pin_function=lambda *args: 0)
        fake.err = types.SimpleNamespace(check_raise=lambda *args: None)
        fake.time = types.SimpleNamespace(ticks_ms=lambda: 100)
        spec = importlib.util.spec_from_file_location('tested_motor_driver', FIRMWARE / 'motor_driver.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'maix': fake}):
            spec.loader.exec_module(module)
        return module

    def test_import_is_passive_and_commands_keep_wire_format(self):
        driver = self.load_driver()
        self.assertIsNone(driver.serial)
        packets = []
        driver.serial = types.SimpleNamespace(write=packets.append)
        driver.motor_enable(2)
        driver.motor_move_to(2, 9)
        driver.motor_stop(2)
        self.assertEqual(packets[0], bytes([2, 0xF3, 0xAB, 1, 0, 0x6B]))
        self.assertEqual(packets[1], bytes([2, 0xFD, 0, 0, 80, 200, 0, 0, 0, 80, 1, 0, 0x6B]))
        self.assertEqual(packets[2], bytes([2, 0xFE, 0x98, 0, 0x6B]))

    def test_pid_and_control_angle_limits(self):
        driver = self.load_driver()
        driver.serial = types.SimpleNamespace(write=lambda _: None)
        control = driver.BallController()
        control.enabled = True
        for _ in range(20):
            previous = control.current_angle
            control.track(1000)
            self.assertLessEqual(abs(control.current_angle), control.MAX_ANGLE_T1)
            self.assertLessEqual(abs(control.current_angle - previous), control.MAX_DTHETA)
        control.track(0)
        self.assertEqual(control.current_angle, 0)


if __name__ == '__main__':
    unittest.main()
