# Bambu-Bus
Tools &amp; Documentation relating to the Bambu Bus

## Introduction
The bambu-bus is a proprietary protocol, based on UART through a RS485 bus<br>
The UART is clocked at 1228800bps with 1 even parity bit and 1 stop bit

## Headers and Devices Adressing
The bambu bus is using 2 packet formats,<br>
The long header packet, which has data indication the master and slave between multiple avaible devices<br>
And The short header packet, which communicate between 1 preset master and slave

these packets are usually comprised of a start byte _3D_, a packet lenght indicator, and an 8 bit CRC for verifying the header integrity.<br>

### Long form header

| Byte number | Content                          | Bytes occupied |
|-------------|----------------------------------|----------------|
| 0           | 0x3D                             | 1              |
| 1           | Flag(less than 0x80)             | 1              |
| 2~3         | Packet sequence                  | 2              |
| 4~5         | Total packet length L            | 2              |
| 6           | CRC8 for all previous bytes [1]  | 1              |
| 7~8         | Target address                   | 2              |
| 9~10        | Source address                   | 2              |
| 11~(L-3)    | Packet content                   | L-13           |
| (L-2)~(L-1) | CRC16 for all previous bytes [2] | 2              |

[1] CRC8 generation polynomial: 0x39, initial value 0x66, no XOR and reverse.  
[2] CRC16 generation polynomial:0x1021, initial value 0x913D, no XOR and reverse, but low byte first in the array.

### adressing relations

| Address Number | Meaning                                                       |
|----------------|---------------------------------------------------------------|
| Address Number | Meaning                                                       |
| 0x03           | MC (motion controller)                                        |
| 0x06           | AP (The first type of upper computer, in the X series)        |
| 0x07           | AMS                                                           |
| 0x08           | TH                                                            |
| 0x09           | AP2(The second type of upper computer, in the P and A series) |
| 0x0E           | AHB                                                           |
| 0x0F           | EXT (May be external control board)                           |
| 0x12           | AMS lite                                                      |
| 0x13           | CTC                                                           |

### short form header

| Byte number | Content                              | Bytes occupied |
|-------------|--------------------------------------|----------------|
| 0           | 0x3D                                 | 1              |
| 1           | Flag and sequence (bigger than 0x80) | 1              |
| 2           | Total packet length L                | 1              |
| 3           | CRC8 for all previous bytes [1]      | 1              |
| 4           | Packet type                          | 1              |
| 5~(L-3)     | Packet content                       | L-7            |
| (L-2)~(L-1) | CRC16 for all previous bytes [2]     | 2              |

[1] CRC8 generation polynomial: 0x39, initial value 0x66, no XOR and reverse.<br>
[2] CRC16 generation polynomial: 0x1021, initial value 0x913D, no XOR and reverse, but low byte first in the array.<br>

The data type of the short frame header (collected in AMS lite, so the content listed is intended for AMS lite): 
| Value | The (possible) meaning                        |
|-------|-----------------------------------------------|
| 0x03  | Read the movement information of the filament |
| 0x04  | Read and change the AMS lite motion status    |
| 0x05  | Verify that the device is online              |
| 0x06  | Unknown                                       |
| 0x07  | Read the NFC information                      |
| 0x20  | Printer heartbeat packet                      |


# Tooling
## Parsers
- Protoparser.py -> packet parser, works best for the X1 serie packets
## Simulators
- AMCU -> AMS emulator written by researcher 4061N

## Research team:
- 4061N
- leonllrmc
