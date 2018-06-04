import dxl
import time
ports = dxl.get_available_ports()
d = dxl.Dxl(ports[0])

ids = d.scan(10)
print ids
d.set_wheel_mode(ids)
d.enable_torque(ids)
d.set_moving_speed({1:0})
