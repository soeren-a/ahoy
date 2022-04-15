#!/usr/bin/env python3X
"""
First attempt at providing basic 'master' ('DTU') functionality
for Hoymiles micro inverters.
Based in particular on demostrated first contact by 'of22'.
"""
import sys
import argparse
import time
import struct
import crcmod
import json
from datetime import datetime
from RF24 import RF24, RF24_PA_LOW, RF24_PA_MAX, RF24_250KBPS
import paho.mqtt.client
from configparser import RawConfigParser

cfg = RawConfigParser()
cfg.read('ahoy.conf')
mqtt_host = cfg.get('mqtt', 'host', fallback='')
mqtt_port = cfg.getint('mqtt', 'port', fallback=1883)
mqtt_user = cfg.get('mqtt', 'user', fallback='')
mqtt_password = cfg.get('mqtt', 'password', fallback='')

if not mqtt_host:
    raise RuntimeError('No MQTT host specified') 

radio = RF24(22, 0, 1000000)
mqtt_client = paho.mqtt.client.Client()
mqtt_client.username_pw_set(mqtt_user, mqtt_password)
mqtt_client.connect(mqtt_host, mqtt_port)
mqtt_client.loop_start()

# Master Address ('DTU')
dtu_ser = cfg.get('dtu', 'serial', fallback='99978563412')  # identical to fc22's

# inverter serial numbers
inv_ser = cfg.get('inverter', 'serial', fallback='') 
if not inv_ser:
    raise RuntimeError('No Inverter specified') 

f_crc_m = crcmod.predefined.mkPredefinedCrcFun('modbus')
f_crc8 = crcmod.mkCrcFun(0x101, initCrc=0, xorOut=0)


def ser_to_hm_addr(s):
    """
    Calculate the 4 bytes that the HM devices use in their internal messages to 
    address each other.
    """
    bcd = int(str(s)[-8:], base=16)
    return struct.pack('>L', bcd)


def ser_to_esb_addr(s):
    """
    Convert a Hoymiles inverter/DTU serial number into its
    corresponding NRF24 'enhanced shockburst' address byte sequence (5 bytes).

    The NRF library expects these in LSB to MSB order, even though the transceiver
    itself will then output them in MSB-to-LSB order over the air.
    
    The inverters use a BCD representation of the last 8
    digits of their serial number, in reverse byte order, 
    followed by \x01.
    """
    air_order = ser_to_hm_addr(s)[::-1] + b'\x01'
    return air_order[::-1]


def compose_0x80_msg(dst_ser_no=72220200, src_ser_no=72220200, ts=None):
    """
    Create a valid 0x80 request with the given parameters, and containing the 
    current system time.
    """

    if not ts:
        ts = 0x623C8ECF  # identical to fc22's for testing  # doc: 1644758171

    # "framing"
    p = b''
    p = p + b'\x15'
    p = p + ser_to_hm_addr(dst_ser_no)
    p = p + ser_to_hm_addr(src_ser_no)
    p = p + b'\x80'

    # encapsulated payload
    pp = b'\x0b\x00'
    pp = pp + struct.pack('>L', ts)  # big-endian: msb at low address
    #pp = pp + b'\x00' * 8    # of22 adds a \x05 at position 19

    pp = pp + b'\x00\x00\x00\x05\x00\x00\x00\x00'

    # CRC_M
    crc_m = f_crc_m(pp)

    p = p + pp
    p = p + struct.pack('>H', crc_m)

    crc8 = f_crc8(p)
    p = p + struct.pack('B', crc8)   
    return p


def print_addr(a):
    print(f"ser# {a} ", end='')
    print(f" -> HM  {' '.join([f'{x:02x}' for x in ser_to_hm_addr(a)])}", end='')
    print(f" -> ESB {' '.join([f'{x:02x}' for x in ser_to_esb_addr(a)])}")


def on_receive(p):
    """
    Callback: get's invoked whenever a packet has been received.
    :param p: Payload of the received packet.
    """

    d = {}

    ts = datetime.utcnow()
    ts_unixtime = ts.timestamp()
    #print(ts.isoformat(), end='Z ')

    # interpret content
    mid = p[0]
    d['mid'] = mid
    name = 'unknowndata'
 
    if mid == 0x95:
        src, dst, cmd = struct.unpack('>LLB', p[1:10])
        src_s = f'{src:08x}'
        dst_s = f'{dst:08x}'
        d['src'] = src_s
        d['dst'] = dst_s
        d['cmd'] = cmd

        if cmd==1:
            name = 'cmd1'
            uk1, uk2, uk3, uk4, uk5, uk6, uk7, uk8 = struct.unpack(
                '>HHHHHHHH', p[10:26])
            d['uk1'] = uk1
            d['uk2'] = uk2
            d['uk3'] = uk3
            d['uk4'] = uk4
            d['uk5'] = uk5
            d['uk6'] = uk6
            d['uk7'] = uk7
            d['uk8'] = uk8
            printValues(d)

        elif cmd == 2:
            name = 'cmd2'
            uk1, uk2, uk3, uk4, uk5, uk6, uk7, uk8 = struct.unpack(
                '>HHHHHHHH', p[10:26])
            d['uk1'] = uk1
            d['uk2'] = uk2
            d['uk3'] = uk3
            d['uk4'] = uk4
            d['dc_u3'] = uk5 / 10 # DC voltage Input 3
            d['dc_i3'] = uk6 / 100 # DC current Input 3
            d['uk7'] = uk7
            d['dc_p3'] = uk8 / 10 # DC power Input 3
            printValues(d)

        elif cmd == 3:
            name = 'cmd3'
            uk1, uk2, uk3, uk4, uk5, uk6, uk7, uk8 = struct.unpack(
                '>HHHHHHHH', p[10:26])
            d['uk1'] = uk1
            d['uk2'] = uk2
            d['kwh_total3'] = uk3 / 1000 # Power total Input 3
            d['uk4'] = uk4
            d['uk5'] = uk5
            d['wh_day3'] = uk6 # Power over Day Input 3
            d['uk7'] = uk7
            d['ac_u'] = uk8 / 10 # AC voltage
            printValues(d)

        elif cmd==129:
            name = 'error'
            print('Command error')

        elif cmd==131: #0x83
            name = 'cmd131'
            uk1, uk2, uk3, uk4, uk5, uk6, uk7, uk8 = struct.unpack('>HHHHHHHH', p[10:26])
           
            d['uk1'] = uk1
            d['uk2'] = uk2
            d['uk3'] = uk3
            d['uk4'] = uk4
            d['uk5'] = uk5
            d['uk6'] = uk6
            d['uk7'] = uk7
            d['uk8'] = uk8
            printValues(d)

        elif cmd==132: #0x84
            name = 'cmd132'
            uk1, uk2, uk3, uk4, uk5, uk6, uk7, uk8 = struct.unpack(
                '>HHHHHHHH', p[10:26])
            d['ac_f'] = uk1 / 100 # AC frequency
            d['ac_p'] = uk2 / 10 # Power overall (current)
            d['uk3'] = uk3
            d['ac_i'] = uk4 / 100 # Power Consumption
            d['thd_percent'] = uk5 / 100 # Total Harmonic Distortion
            d['temp_c'] = uk6 / 10 # inverter temperature
            d['uk7'] = uk7
            d['uk8'] = uk8
            printValues(d)

    else:
        print(f'unknown frame id {p[0]}')

    # output to MQTT
    if d:
        j = json.dumps({x: d[x] for x in d if x not in ["cmd", "src", "dst", "mid"] and not x.startswith('uk')})
        #print(j)
        """
        if d['cmd']==2:
            mqtt_client.publish(f'ahoy/{src}/emeter/0/voltage', d['u_V'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter/0/power', d['p_W'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter/0/total', d['wtot1_Wh'] or '0')
            mqtt_client.publish(f'ahoy/{src}/frequency', d['f_Hz'] or '0')
            mqtt_client.publish(f'ahoy/{src}/overall', j)
        if d['cmd']==1:
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/1/power', d['p1_W'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/1/voltage', d['u1_V'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/1/current', d['i1_A'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/2/power', d['p2_W'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/2/voltage', d['u2_V'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/2/current', d['i2_A'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/3/power', d['p3_W'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/3/voltage', d['u3_V'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/3/current', d['i3_A'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/4/power', d['p4_W'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/4/voltage', d['u4_V'] or '0')
            mqtt_client.publish(f'ahoy/{src}/emeter-dc/4/current', d['i4_A'] or '0')
        if d['cmd']==131:
            mqtt_client.publish(f'ahoy/{src}/temperature', d['t_C'])
        """
def printValues(d):
    print(f'cmd{d["cmd"]}:\n{json.dumps({x: d[x] for x in d if x not in ["cmd", "src", "dst", "mid"]}, indent = 3)}')

def main_loop():
    """
    Keep receiving on channel 3. Every once in a while, transmit a request
    to one of our inverters on channel 40.
    """

    print_addr(inv_ser)
    print_addr(dtu_ser)

    ctr = 1
    last_tx_message = ''

    rx_channels = [3,23,61,75]
    rx_channel_id = 0
    rx_channel = rx_channels[rx_channel_id]

    tx_channels = [40]
    tx_channel_id = 0
    tx_channel = tx_channels[tx_channel_id]

    while True:
        # Sweep receive start channel
        rx_channel_id = ctr % len(rx_channels)
        rx_channel = rx_channels[rx_channel_id]

        radio.setChannel(rx_channel)
        radio.enableDynamicPayloads()
        radio.setAutoAck(True)
        radio.setPALevel(RF24_PA_MAX)
        radio.setDataRate(RF24_250KBPS)
        radio.openWritingPipe(ser_to_esb_addr(inv_ser))
        radio.flush_rx()
        radio.flush_tx()
        radio.openReadingPipe(1,ser_to_esb_addr(dtu_ser))
        radio.startListening()

        t_end = time.monotonic_ns()+1e9
        while time.monotonic_ns() < t_end:
            has_payload, pipe_number = radio.available_pipe()
            if has_payload:
                size = radio.getDynamicPayloadSize()
                payload = radio.read(size)
                #print(last_tx_message, end='')
                last_tx_message = ''
                dt = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                #print(f"{dt} Received {size} bytes on channel {rx_channel} pipe {pipe_number}: " +
                #      " ".join([f"{b:02x}" for b in payload]))
                on_receive(payload)
            else:
                radio.stopListening()
                radio.setChannel(rx_channel)
                radio.startListening()
                rx_channel_id = rx_channel_id + 1
                if rx_channel_id >= len(rx_channels):
                    rx_channel_id = 0
                rx_channel = rx_channels[rx_channel_id]
                time.sleep(0.01)

        tx_channel_id = tx_channel_id + 1
        if tx_channel_id >= len(tx_channels):
            tx_channel_id = 0
        tx_channel = tx_channels[tx_channel_id]

        radio.stopListening()  # put radio in TX mode
        radio.setChannel(tx_channel)
        radio.openWritingPipe(ser_to_esb_addr(inv_ser))

        ts = int(time.time())
        payload = compose_0x80_msg(src_ser_no=dtu_ser, dst_ser_no=inv_ser, ts=ts)
        dt = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        last_tx_message = f"{dt} Transmit {ctr:5d}: channel={tx_channel} len={len(payload)} | " + \
            " ".join([f"{b:02x}" for b in payload]) + "\n"
        radio.write(payload)  # will always yield 'True' because auto-ack is disabled
        ctr = ctr + 1

        print(flush=True, end='')




if __name__ == "__main__":

    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")

    radio.setPALevel(RF24_PA_LOW)  # RF24_PA_MAX is default

    # radio.printDetails();  # (smaller) function that prints raw register values
    # radio.printPrettyDetails();  # (larger) function that prints human readable data

    try:
        main_loop()

    except KeyboardInterrupt:
        print(" Keyboard Interrupt detected. Exiting...")
        radio.powerDown()
        sys.exit()
