import dxl
import time
ports = dxl.get_available_ports()
d = dxl.Dxl(ports[0])

print d.scan(10)
d.set_wheel_mode((4,))
d.set_moving_speed({4:0})

