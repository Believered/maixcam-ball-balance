"""Bounded incremental decoder for the archived Emm reply format.

0x36: addr, cmd, sign, magnitude[4], 0x6B (8 bytes)
0x3A: addr, cmd, status, 0x6B (4 bytes)
Acknowledgements use addr, cmd, result, 0x6B in the archive assumption.
The final 0x6B is a terminator, not a cryptographic/integrity checksum.
"""


class FrameDecoder:
    ACK_COMMANDS = (0xF3, 0xF6, 0xFD, 0xFE, 0x0A, 0x93, 0x9A, 0x46)

    def __init__(self, max_buffer=512):
        if not isinstance(max_buffer, int) or isinstance(max_buffer, bool) or max_buffer < 8:
            raise ValueError('max_buffer must be an integer of at least 8')
        self.max_buffer = max_buffer
        self.buffer = bytearray()

    def feed(self, data):
        frames = []
        # Process bytes incrementally, bounding memory even for long noisy input.
        for byte in data:
            self.buffer.append(byte)
            if len(self.buffer) > self.max_buffer:
                del self.buffer[:-self.max_buffer]
            while len(self.buffer) >= 2:
                address, command = self.buffer[0], self.buffer[1]
                size = 8 if command == 0x36 else 4
                if address == 0 or command not in (0x36, 0x3A) + self.ACK_COMMANDS:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < size:
                    break
                if self.buffer[size - 1] != 0x6B or (command == 0x36 and self.buffer[2] not in (0, 1)):
                    del self.buffer[0]
                    continue
                frames.append((address, command, bytes(self.buffer[2:size - 1])))
                del self.buffer[:size]
        return frames
