from crccheck.crc import Crc32, Crc32c
from crccheck.checksum import Checksum32
import os

PATH = '/home/jan/vivado-workspace/re7config/bitstreams/'
FILE = 'partial_green.bin'
TEST = 0

bytes = 0

file_size = os.path.getsize(PATH + FILE)
print(file_size)

bytes_num_to_append = 4 - file_size % 4
print(bytes_num_to_append)

# check if there's need to do zero-padding of the file
if TEST == 0:
    if bytes_num_to_append > 0 and bytes_num_to_append < 4:
        print('appending some bytes')
        with open(PATH + FILE, 'ab') as bitstream:
            bytes = bitstream.write((0x0).to_bytes(
                bytes_num_to_append, 'little'))

file_size = os.path.getsize(PATH + FILE)
print(file_size)

raw_file = 0
with open(PATH + FILE, 'rb') as bitstream:
    raw_file = bitstream.read(file_size)

crcinst = Crc32c()
crcinst.process(raw_file)
chksum = crcinst.final()
print(hex(chksum))

with open(PATH + FILE, 'ab') as bitstream:
    bytes = bitstream.write(chksum.to_bytes(4, 'little'))
