#!/usr/bin/env python3

from sys import argv
from crccheck.crc import Crc

DEVICES = {
    3: "MC",
    6: "AP",
    7: "AMS",
    8: "TH",
    9: "AP2",
    0xE: "AHB",
    0xF: "EXT",
    0x13: "CTC",
}

bambu_crc = Crc(width=16, poly=0x1021, initvalue=0x913D)

COMMANDS = {
    (1, 1): "ping",
    (1, 3): "get_version",
    (1, 4): "sync_timestamp",
    (1, 6): "mcu_upgrade",
    (1, 8): "mcu_hms",
    (1, 9): "factory_reset",
    (2, 5): "gcode_execute_state",
    (2, 6): "gcode_request", # no official name for this
    (2, 9): "mcu_display_message",
    (2, 10): "vosync",
    (2, 11): "gcode_ctx",
    (2, 12): "mc_state",
    (2, 15): "link_ams_report",
    (2, 17): "ack_tray_info",
    (2, 22): "gcode_line_handle",
    (2, 23): "ams_mapping",
    (2, 24): "ams_tray_info_write_ack",
    (2, 25): "ams_user_settings",
    (2, 27): "hw_info_voltage",
    (2, 28): "link_ams_tray_consumption_ack",
    (2, 29): "pack_get_part_info_ack",
    (2, 34): "extrusion_result_update",
    (2, 36): "fila_ams_get",
    (2, 37): "mc_get_skipped_obj_list",
    (3, 1): "M971",
    (3, 2): "M972",
    (3, 5): "M963",
    (3, 7): "M969",
    (3, 6): "M965_b",
    (3, 9): "M967",
    (3, 11): "M973",
    (3, 14): "M965",
    (3, 49): "M976",
    (3, 50): "M977",
    (3, 51): "M978",
    (3, 52): "M981",
    (3, 53): "M991",
    (3, 81): "M987",
    (3, 82): "SENSORCHECK",
    (4, 1): "set_module_sn",
    (4, 2): "get_module_sn",
    (4, 3): "inject_module_key",
    (4, 4): "get_inject_status",
    (4, 5): "mcu_reboot",
    (4, 6): "send_to_amt_core",
    (4, 7): "set_module_lifecycle",
    (4, 8): "get_module_lifecycle",
    (4, 10): "inject_productcode",
    (4, 11): "get_productcode",
}

def main():
    fh = open(argv[1], 'rb')
    data = fh.read()
    fh.close()

    first_byte = data.index(b'\x3d')
    data = data[first_byte:]

    while data:
        assert data[0] == 0x3d
        flags = data[1] # or flags?
        flags_is_initiator   = flags & 0x4
        flags_wants_response = flags & 0x1
        # i.e., flags == 4 is "posted message"; flags == 5 is "RPC wants response"; flags == 0 is "here is your response"
        sequence = data[2] | (data[3] << 8)
        packet_len = data[4] | (data[5] << 8)
        header_cksm = data[6]
        packet_data = data[7:packet_len-2]
        packet_cksm = (data[packet_len-1] << 8) | (data[packet_len-2] << 0)
        exp_cksm = bambu_crc.calc(data[:packet_len-2])
        
        data = data[packet_len:]
        
        # last two bytes appear to be a checksum?
        
        addr_to   = (packet_data[0] << 8) | (packet_data[1] << 0)
        addr_from = (packet_data[2] << 8) | (packet_data[3] << 0)
        
        cmd_id    = packet_data[4]
        cmd_set   = packet_data[5]
        packet_pl = packet_data[6:]
        
        addr_from = DEVICES.get(addr_from, f"unknown[{addr_from}]")
        addr_to   = DEVICES.get(addr_to  , f"unknown[{addr_to}]")
        
        print(f"{flags} {sequence}: {addr_from} -> {addr_to}: {COMMANDS.get((cmd_set, cmd_id), 'unknown cmd')} ({cmd_set}, {cmd_id}): {packet_pl} {'' if exp_cksm == packet_cksm else '[CRC16 FAILURE]'}")

        if data and data[0] != 0x3d:
            first_byte = data.index(b'\x3d')
            print(f"skipped {first_byte} bytes: {data[:first_byte]}")
            data = data[first_byte:]


if __name__ == '__main__':
    main()
