# type: ignore
from machine import mem32

# Define GPIO pad registers
PADS_BANK0_BASE = 0x4001C000
PAD_GPIO = PADS_BANK0_BASE + 0x04
PAD_GPIO_MPY = 4
PAD_DRIVE_BITS = 4

def set_pin_drive_strength(pin, mA):
    """Sets the drive strength of a GPIO pin."""
    addr = PAD_GPIO + PAD_GPIO_MPY * pin
    mem32[addr] &= 0xFFFFFFFF ^ (0b11 << PAD_DRIVE_BITS)

    if mA <= 2:
        mem32[addr] |= 0b00 << PAD_DRIVE_BITS
    elif mA <= 4:
        mem32[addr] |= 0b01 << PAD_DRIVE_BITS
    elif mA <= 8:
        mem32[addr] |= 0b10 << PAD_DRIVE_BITS
    else:
        mem32[addr] |= 0b11 << PAD_DRIVE_BITS