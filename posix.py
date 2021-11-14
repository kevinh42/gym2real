import posix_ipc
import posix 
import yaml
import struct

with open('statestore.yaml', 'r') as f:
    stores = yaml.load(f, Loader=yaml.Loader)['stores']

k = "controller_output"
fd = posix_ipc.SharedMemory(k,flags=0,size=stores['critical'][k])
val = posix.read(fd.fd,stores['critical'][k])
fval = struct.unpack('f',val[0:4])
print(fval)
fd.close_fd()


