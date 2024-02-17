from Aurora import Aurora, PortHandle
# from GalilRobot import GalilRobot
# from GalilRobot import RobotAxisID
import serial
import time
import binascii

PRINT_HANDLE = True

#tracker = Aurora(timeout=1, baud_rat=38400)
tracker = Aurora(baud_rat=9600)
# tracker = Aurora(baud_rat=921600)

if not tracker._isConnected:
    print('tracker is not connected!')
    tracker.connect()

time.sleep(.1)
if tracker._isConnected:
    print('tracker is connected!')

# aurora_device.init();
time.sleep(.1)
tracker.init()
print('tracker._serial_object.isOpen() is ' + str(tracker._serial_object.isOpen()))
print('tracker._device_init is ' + str(tracker._device_init))
# aurora_device.detectAndAssignPortHandles();
#ser.write('PHSR 00'.encode('utf-8'))
#read_val = ser.read(size=2)
#print(type(read_val))
# n_port_handles = int(ser.read(2), 16)
# print(n_port_handles)
#n_port_handles = 0

# Beeeeeep
#time.sleep(1)
#print('vor tracker._BEEP(2)')
#tracker._BEEP(2)

# time.sleep(1)
# print('vor tracker._APIREV()')
# tracker._APIREV()
# read_val = tracker._serial_object.readline()
# print(read_val)

# handle
# time.sleep(1)
# tracker.portHandles_detectAndAssign()
print('------------------------ setting ------------------------')
# tracker._port_handles[0]._printSensorStatus()
# print('tracker._n_port_handles is ' + str(tracker._n_port_handles))
# print('len(tracker._port_handles) is ' + str(len(tracker._port_handles)))
# print('tracker._port_handles[0].sensor_status is ' + str(tracker._port_handles[0].sensor_status))

time.sleep(0.1)
tracker.portHandles_detectAndAssign_FlowChart(printFeedback=True)
time.sleep(0.1)
#tracker._serial_object.flush()
tracker.portHandles_updateStatusAll()

# Tracking
print('------------------------ tracking ------------------------')
tracker.trackingStart()
time.sleep(3)
tracker.sensorData_write_FileName('/home/rgrassmann/PycharmProjects/Messungen/', 'data.txt')
tracker.sensorData_write_FileIni()
tic = time.time()
for n in range(10):
    tracker.sensorData_updateAll()
    tracker.sensorData_write(n)

tracker.sensorData_write_FileClose()
toc = time.time()

print(toc - tic, 'Sekunden sind vergangen.')
# print('reply')
# reply = tracker._serial_object.readline()
# print type(reply)

if PRINT_HANDLE:
    print('------------------------ handles ------------------------')
    for n in range(tracker._n_port_handles):
        print('-- tracker._n_port_handles[' + str(n) + ']')
        print('   portHandle_ID         -> ' + str(tracker._port_handles[n].portHandle_ID))
        print('   portHandle_status     -> ' + str(tracker._port_handles[n].portHandle_status))
        print('   occupied              -> ' + str(tracker._port_handles[n]._occupied))
        print('   initialized           -> ' + str(tracker._port_handles[n]._initialized))
        print('   gpio_line1_closed     -> ' + str(tracker._port_handles[n]._gpio_line1_closed))
        print('   gpio_line2_closed     -> ' + str(tracker._port_handles[n]._gpio_line2_closed))
        print('   gpio_line3_closed     -> ' + str(tracker._port_handles[n]._gpio_line3_closed))
        print('   out_of_volume         -> ' + str(tracker._port_handles[n]._out_of_volume))
        print('   partial_out_of_volume -> ' + str(tracker._port_handles[n]._partial_out_of_volume))
        print('   sensor_broken         -> ' + str(tracker._port_handles[n]._sensor_broken))
        print('   sensor_status         -> ' + str(tracker._port_handles[n]._sensor_status))
        print('   ------------------------------')
        print('   x  = ' + str(tracker._port_handles[n]._trans[0]))
        print('   y  = ' + str(tracker._port_handles[n]._trans[1]))
        print('   z  = ' + str(tracker._port_handles[n]._trans[2]))
        print('   q0 = ' + str(tracker._port_handles[n]._quaternion[0]))
        print('   qx = ' + str(tracker._port_handles[n]._quaternion[1]))
        print('   qy = ' + str(tracker._port_handles[n]._quaternion[2]))
        print('   qz = ' + str(tracker._port_handles[n]._quaternion[3]))
        print('   error = ' + str(tracker._port_handles[n]._error))
        print('   frame_number ' + str(tracker._port_handles[n]._frame_number))

time.sleep(1)
print('vor tracker._BEEP(2)')
tracker._BEEP(2)
tracker.trackingStop()


# Close and disconnect Aurora
print('------------------------ closing ------------------------')
if tracker._isConnected:
    print('Nach dem Beep wird Aurora geschlossen')
    tracker._BEEP(1)
    tracker.disconnect()
