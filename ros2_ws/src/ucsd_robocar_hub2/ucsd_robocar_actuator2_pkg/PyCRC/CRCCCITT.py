from pycrc.algorithms import Crc


class CRCCCITT:
    """Compatibility shim for legacy pyvesc imports on modern pycrc."""

    def __init__(self):
        self._crc = Crc(
            width=16,
            poly=0x1021,
            reflect_in=False,
            xor_in=0x0000,
            reflect_out=False,
            xor_out=0x0000,
        )

    def calculate(self, payload):
        return self._crc.bit_by_bit_fast(payload)
