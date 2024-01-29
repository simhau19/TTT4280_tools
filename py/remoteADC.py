'''
Author: Simen Haug
Date: 26.01.2024
Version: 1.0

Greetings fellow elsysers!
Use this script to remotely start the ADCs and fetch data from the Raspberry Pi like a champ.
See usage example below.

Remember to define the adc pins in remote_adc_server.c
Compile and run remote_adc_server on the pi before using this script.
'''



import numpy as np
import sys
import socket
import struct


class remoteADC(socket.socket):

    def __init__(self):
        super().__init__(socket.AF_INET, socket.SOCK_STREAM)

    def recvall(self, count):
        buf = b''
        while(count > 0):
            chunk = self.recv(count)
            if not chunk:
                raise ConnectionResetError("Connection terminated unexpectedly")
            buf += chunk
            count -= len(chunk)
        return buf

    def read(self, sample_count = 31250, channel_count = 5):

        msg = struct.pack('<II', sample_count, channel_count)
        self.send(msg)

        #double (8 bytes) + uint16 (2 bytes) * sample_count * channel_count
        expected_size = 8 + 2*sample_count*channel_count

        raw_data = self.recvall(expected_size)

        sample_period = np.frombuffer(raw_data, count=1, dtype=float)[0]
        data = np.frombuffer(raw_data[8:], dtype='uint16').astype('float64')
        data = data.reshape((-1, channel_count))

        return sample_period * 1e-6, data



if __name__ == "__main__":

    #hostname or ip
    HOST = "raspberrypi.local"

    #must match the port in remote_adc_server.c
    PORT = 4280

    samples = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
    adc_channels = int(sys.argv[2]) if len(sys.argv) > 2 else 5


    with remoteADC() as adc:
        print(f"Connecting to {HOST}:{PORT}")
        adc.connect((HOST, PORT))
        print("Connected! Requesting data...")
        t, d = adc.read(samples, adc_channels)
        print("Data received!")
        print("Sampling period:", t)
        print("Data:")
        print(d)
        
