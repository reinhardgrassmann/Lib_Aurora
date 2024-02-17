"""
This library definition based on MATLAB scripts by Andre Augusto Geraldes

The  PortHandle class represents simultaneously one port handle of the Aurora SCU and the tool connected to it.

Author: Reinhard Grassmann
Email: reinhard.grassmann@posteo.de
December 2017

Update February 2021:
- If you get the following "Permission denied: '/dev/ttyUSB0'" use $ sudo chmod a+rw /dev/ttyUSB0
- install pyserial instead of serial
- "ImportError: cannot import name IntEnum" install enum34 instead enum

Update March 2021:
- reset _port_measurments in sensorData_write_FileName to make overwriting of the file direction possible

Update December 2021:
- force write function to flush its buffer by using f.flush() and os.fsync
- this allows storing multiple measurement in the right order

Updated October 2022 (By Spencer Teetaert, spencer.teetaert@mail.utoronto.ca):
- Updated for Python 3 support 
"""



import math
import os

import numpy as np
import time
import serial
# from serial import serial
import struct

class Aurora:
    """
    This class offers a collection of tools used in the command and communication of NDI Medical electromagnetic tracking
    system.

    This class is based on AuroaDriver class created by Andre Augusto Geraldes. For more information vised github
    https://github.com/lara-unb/APIs-NDI-Digital
    """

    # Formats for sending commands (source: Aurora_API_Guide page 4)
    COMMAND_FORMAT_1 = 1
    COMMAND_FORMAT_2 = 2

    # Handle Status (source: Aurora_API_Guide page 13)
    SENSOR_STATUS_VALID    = '01'
    SENSOR_STATUS_MISSING  = '02'
    SENSOR_STATUS_DISABLED = '04'

    # Tool tracking priority codes (source: Aurora_API_Guide page 26)
    TT_PRIORITY_STATIC  = 'S'
    TT_PRIORITY_DYNAMIC = 'D'
    TT_PRIORITY_BUTTON  = 'B'

    def __init__(self, serial_port='/dev/ttyUSB0', timeout=1, baud_rat=9600, verbose=True):
        # depending on operating system.e.g. / dev / ttyUSB0 on GNU / Linux or COM3 on Windows.
        self._serial_port = serial_port  # serial(serial_port)
        self._baud_rat = baud_rat
        self._n_port_handles = 0
        self._port_handles = []
        self._port_measurements = []
        self._port_measurements_path = ''
        self._selected_command_format = self.COMMAND_FORMAT_2
        self._serial_object = serial.Serial(
            port=self._serial_port,
            timeout=timeout)
        self._isConnected = self._serial_object.isOpen()
        self._timeout = timeout
        self._device_init = False
        self._verbose = verbose

        self.setBaudRate(baud_rate=baud_rat)

    def __del__(self):
        # Possible commands are:
        # - PDIS for the Port Handles that have been enabled
        # - PHF for the Port Handles that have been initialized
        # - RESET
        #self._RESET('0')  # self.setBaudRate()
        self.disconnect()

    def _send_cmd(self, cmd, sec=None, get_replay=False):
        """
        This function receives a command as an entire formated string and sends it to the Aurora SCU.

        :param cmd: ascii string without CR (carriage return) at the end.
        :param sec: pause in second.
        :param get_replay: enable or disable reply.
        :return: None or read a '\n' terminated line of the serial port
        """
        # time.sleep(sec)
        # API commands need a CR character at the end of every message sent through the serial.
        # for that reason, the carriage return '\x0d' (or '\r') is added to the end of the commands.

        if sec is None:
            sec = {
                9600: 0.1,
                14400: 0.1,   # Not tested
                19200: 0.1,   # Not tested
                38400: 0.1,   # Not tested
                57600: 0.1,   # Not tested
                115200: 0.1,  # Not tested
                921600: 0.1,  # Not tested
                230400: 0.1   # Not tested
            }.get(self._baud_rat, 0.1)

        time.sleep(sec)
        self._serial_object.write(str.encode(cmd + '\r'))
        if get_replay:
            self._serial_object.flush()
            return self._serial_object.readline().decode()
        else:
            return None

    def disconnect(self):
        if self._port_measurements != []:
            self.sensorData_write_FileClose()

        if self._isConnected:
            for ith_port_handle in range(self._n_port_handles):
                self._PDIS(self._port_handles[ith_port_handle].portHandle_ID)
                self._PHF(self._port_handles[ith_port_handle].portHandle_ID)
            self._RESET('0')
            time.sleep(1)
            self._isConnected = False
            self._serial_object.close()

            print("\033[93m" +
                  'Closes connection to Aurora ( ' + str(self._serial_port) + ' ) ' +
                  " \033[0m")

    def connect(self, sec=1):
        if not self._isConnected:
            self._serial_object.open()
            time.sleep(sec)
            self._isConnected = self._serial_object.isOpen()

    def setBaudRate(self, baud_rate=9600, data_bits=8, parity='None', stop_bits=1, hardware_handshaking='Off'):
        """
        This function allows setting the Baud Rate of the serial port.

        For further information on the serial port parameters, refer to the Aurora_API_Guide page 16.

        :param baud_rate: The data transmission rate between the Aurora System and the host computer,
        in bits per second.
        :param data_bits: The data bits parameter must be set to 8 bits in order to use the BX command.
        :param parity:
        :param stop_bits:
        :param hardware_handshaking:
        :return:
        """

        # Baud rate options (source: Aurora_API_Guide page 16)
        baut_rate_code = {
            9600: '0',
            14400: '1',     # PySerial does not support this parameter
            19200: '2',
            38400: '3',
            57600: '4',
            115200: '5',
            921600: '6',
            230400: 'A'     # Not supported
        }.get(baud_rate)
        data_bits_code = {
            8: '0',
            7: '1'
        }.get(data_bits)
        parity_code = {
            'None': '0',
            'Odd': '1',
            'Even': '2'
        }.get(parity)
        stop_bits_code = {
            1: '0',
            2: '1'
        }.get(stop_bits)
        hardware_handshaking_code = {
            'Off': '0',
            'On': '1'
        }.get(hardware_handshaking)

        self._COMM(
            baut_rate_s=baut_rate_code,
            data_bits_s=data_bits_code,
            parity_s=parity_code,
            stop_bits_s=stop_bits_code,
            hardware_handshaking_s=hardware_handshaking_code
        )

        self._serial_object.baudrate = baud_rate
        self._serial_object.bytesize = {
            8: serial.EIGHTBITS,
            7: serial.SEVENBITS
        }.get(data_bits)
        self._serial_object.parity = {
            'None': serial.PARITY_NONE,
            'Odd': serial.PARITY_ODD,
            'Even': serial.PARITY_EVEN
        }.get(parity)
        self._serial_object.stopbits = {
            1: serial.STOPBITS_ONE,
            2: serial.STOPBITS_TWO
        }.get(stop_bits)

    def init(self):
        """
        Initiate the Aurora SCU. This needs to be performed right after
        opening the serial port, for enabling all other functions.

        :return:
        """
        self._INIT()
        self._device_init = True

    def trackingStart(self):
        """
        Puts the Aurora SCU into Tracking mode. This enables the position
        reading functions, but disables configuration functions. For
        further information, reffer to the Aurora_API_Guide page 3

        :return:
        """
        self._TSTART('80')  # 80 Rests the frame counter to zero

    def trackingStop(self):
        """
        Puts the Aurora SCU back into Setup mode

        :return:
        """
        self._TSTOP()

    def reply_withoutCR(self, msg):
        # delete last CR
        msg_split = msg.strip('\r').split('\r')

        msg_sought = False

        for msg_i in msg_split:
            if msg_sought:
                return msg_i

            if msg_i != '1D4C1':
                if len(msg_i) >= 4:
                    if msg_i[:4] == 'ERRO':
                        self.printErrorMessage(msg_i)
                    elif msg_i[:4] == 'OKAY':
                        msg_sought = True

        return msg_split[0]

    def systemAntwort(self, trials=50, sec=None):

        nth_trial = 0

        if sec is None:
            sec = {
                9600: 0.125,
                14400: 0.125,   # Not tested
                19200: 0.125,   # Not tested
                38400: 0.06,
                57600: 0.125,   # Not tested
                115200: 0.125,  # Not tested
                921600: 0.01,  # Not tested
                230400: 0.125   # Not tested
            }.get(self._baud_rat, 0.125)

        while True:
            #self._serial_object.flush()
            #reply = self._serial_object.readline().decode()
            #print repr(reply)

            bytes2read = self._serial_object.inWaiting()
            reply = self._serial_object.read(bytes2read)

            if nth_trial == trials:
                if self._verbose:
                    print("\033[93m" +
                        "Warring (systemAntwort): reply maybe wrong." +
                        " \033[0m")
                break
            else:
                nth_trial += 1

            if len(reply) >= 4:
                if reply[:4] == b'ERRO':
                    self.printErrorMessage(reply)
                    continue
                elif reply[:4] == b'OKAY':
                    continue

            if reply != b'':
                break

            time.sleep(sec)

        b_array = bytearray(reply)

        if len(b_array):
            return b_array
        else:
            return None

    def portHandles_detectAndAssign_FlowChart(self, sec=None, trials=30, printFeedback=False):
        # Retrieves the list of all available Port Handles, with their ID
        # and Status, and initialize the port_handles member variable

        NO_TOOLS_MSG = '001414'
        TRIALS = trials
        STEP_ONE = '01'
        STEP_TWO = '02'
        STEP_THREE = '03'

        step_current = STEP_ONE

        if sec is None:
            sec = {
                9600: 0.001,
                14400: 0.001,   # Not tested
                19200: 0.001,   # Not tested
                38400: 0.001,   # Not tested
                57600: 0.001,   # Not tested
                115200: 0.001,  # Not tested
                921600: 0.001,  # Not tested
                230400: 0.001   # Not tested
            }.get(self._baud_rat, 1)

        for step in range(TRIALS):
            if step_current == STEP_ONE:
                # First step: Are there port handles to be freed?
                reply = self._PHSR(reply_option=STEP_ONE)
                msg = self.reply_withoutCR(reply)
                if printFeedback:
                    print('PHRS ' + STEP_ONE + ' replies with ' + repr(msg))

                # if msg == '' and len(reply) >= 4:
                #     if reply[:4] == 'OKAY':
                #         continue

                if msg == NO_TOOLS_MSG:
                    # No port handles has to be freed
                    step_current = STEP_TWO
                else:
                    # Free port Handles (PHF)
                    portHandle = msg[2:4]
                    if self.portHandles_isPortHandleID(portHandle):
                        self._PHF(portHandle)

            elif step_current == STEP_TWO:
                # Second step: Get port handles that need to be initialized
                reply = self._PHSR(reply_option=STEP_TWO)
                msg = self.reply_withoutCR(reply)
                if printFeedback:
                    print('PHRS ' + STEP_TWO + ' replies with ' + repr(msg))

                # if msg == '' and len(reply) >= 4:
                #     if reply[:4] == 'OKAY':
                #         continue

                if msg == NO_TOOLS_MSG:
                    # No port handles has to be initialized
                    step_current = STEP_THREE
                else:
                    # Initialize port handle (PINIT)
                    portHandle = msg[2:4]
                    if self.portHandles_isPortHandleID(portHandle):
                        self._PINIT(portHandle)

            elif step_current == STEP_THREE:
                # Third step: Get port handles that need to be enabled
                reply = self._PHSR(reply_option=STEP_THREE)
                msg = self.reply_withoutCR(reply)
                if printFeedback:
                    print('PHRS ' + STEP_THREE + ' replies with ' + repr(msg))

                # if msg == '' and len(reply) >= 4:
                #     if reply[:4] == 'OKAY':
                #         continue

                if msg == NO_TOOLS_MSG:
                    # ready for start tracking
                    break
                else:
                    # Enable port handles (PENA)
                    portHandle = msg[2:4]
                    if self.portHandles_isPortHandleID(portHandle):
                        self.portHandles_dynamicsEnalbe(portHandle)

            time.sleep(sec)

        # detect and assign
        reply = self._PHSR(reply_option='00')
        msg = self.reply_withoutCR(reply)

        self._n_port_handles = int(msg[:2], 16)

        for idx in range(self._n_port_handles):
            s = 2 + 5*idx
            # if idx == 1:
            self._port_handles.append(PortHandle(
                portHandle_ID=msg[s:s+2],
                portHandle_status=msg[s+2:s+5]
            ))

    def portHandles_isPortHandleID(self, portHandle_ID):
        # portHandle_ID.strip('0x').strip('0X')
        hex_digits = set("0123456789abcdefABCDEF")
        for c in portHandle_ID:
            if not (c in hex_digits):
                return False
        return True

    def portHandles_updateStatusAll(self):
        reply = self._PHSR(reply_option='00')
        msg = self.reply_withoutCR(reply)

        found_n = int(msg[:2], 16)

        if self._n_port_handles is not found_n:
            if self._verbose:
                print("\033[93m" +
                    "Warring (portHandles_updateStatusAll): Number of tools has been changed." +
                    " \033[0m")

        for idx_found in range(found_n):
            s = 2 + 5*idx_found
            found_ID = reply[s:s+2]
            found_status = reply[s+2:s+5]

            for idx in range(self._n_port_handles):
                if found_ID == self._port_handles[idx].portHandle_ID:
                    self._port_handles[idx].updateStatusComplete(found_status)
                    break

    def portHandles_init(self, portHandle_id):
        """
        Initializes one port handle.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        self._PINIT(portHandle_id)

    def portHandles_initAll(self):
        """
        Initializes all port handles that habe already been detected and update their status.
        :return:
        """
        for handle in self._port_handles:
            self.portHandles_init(handle.portHandle_ID)

        self.portHandles_updateStatusAll()

    def portHandles_dynamicsEnalbe(self, portHandle_id):
        # Tool tracking priority codes (source: Aurora_API_Guide page 26)
        # TT_PRIORITY_STATIC = 'S'
        # TT_PRIORITY_DYNAMIC = 'D'
        # TT_PRIORITY_BUTTON = 'B'

        self._PENA(portHandle_id, 'D')

    def portHandles_dynamicsEnableAll(self):
        """
        Enable all Port Handles that have already been detected and update their status.

        :return:
        """
        for handle in self._port_handles:
            self.portHandles_dynamicsEnalbe(handle.portHandle_ID)

        self.portHandles_updateStatusAll()

    def unpackBytes(self, stringBytes, stringMsg = ''):
        if len(stringBytes) == 1:
            return stringBytes[0]
        elif len(stringBytes) == 2:
            return int("{0:b}".format(stringBytes[1]) + "{0:b}".format(stringBytes[0]), 2)
        else:
            if self._verbose:
                print("NIT", stringBytes)
                print("\033[93m" +
                    "Warring (unpackBytes): return is None " + stringMsg +
                    " \033[0m")
            return None

    def portHandles_int2hex(self, value_int):
        return format(value_int, '02X')

    def sensorData_updateAll(self):
        """
        Reads the current measurement of all sensors and update the corresponding Port Handle objects.

        :return:
        """
        if self._device_init:

            # Send a BX command for reading all sensors
            # reply = self._BX('0801', get_replay=True)
            # Sensor reading options (source: Aurora_API_Guide page 12)
            # READ_OUT_OF_VOLUME_NOT_ALLOWED = '0001'
            # READ_OUT_OF_VOLUME_ALLOWED     = '0800'  # '0801'

            # Error checking information
            # Unused

            # Number of available Port Handles (A = fread(fileID,sizeA,precision))

            self._BX('0801', get_replay=False)
            # self._serial_object.write('BX 0801\x0d')
            reply_ba = self.systemAntwort()
            if reply_ba is None: 
                return 

            # Start Sequence (A5C4) has 2 bytes. First two bytes.
            start_sequence = self.unpackBytes(reply_ba[0:2], 'start_sequence')

            # Reply Length has 2 bytes. Third and forth byte
            reply_length = self.unpackBytes(reply_ba[2:4], 'reply_length')

            # Header CRC has 16 bits (= 2 bytes). Fifth and sixth byte
            header_CRC = self.unpackBytes(reply_ba[4:6], 'header_CRC')

            # Number of Handles has 1 byte. Seventh byte
            num_handle_reads = self.unpackBytes(reply_ba[6:7], 'num_handle_reads')
            # print('num_handle_reads = ' + str(num_handle_reads))

            if num_handle_reads is None: # Handles None case 
                num_handle_reads = 0

            for idx_reads in range(num_handle_reads):
                # Get next position s in bytearray
                s = 8 + 42 * idx_reads

                # Get the Port Handle ID as a 2 character hexadecimal
                found_ID = self.portHandles_int2hex(self.unpackBytes(reply_ba[s-1:s], 'found_ID'))

                # Get the Port Handle status as a 2 character hexadecimal
                found_status = self.unpackBytes(reply_ba[s:s+1], 'found_status')

                if found_status == 2:
                    if self._verbose:
                        print("\033[93m Handle status of " + found_ID + " is 2: Missing \033[0m")
                    break
                elif found_status == 4:
                    if self._verbose:
                        print("\033[93m Handle status of " + found_ID + " is 2: Disabled \033[0m")
                    break

                # ########### -- Debug
                # print('found_ID is ' + found_ID)
                # print('idx_reads in range(num_handle_reads is ' + str(idx_reads))
                # print('self._n_port_handles = ' + str(self._n_port_handles))
                # print('reply_ba[s + 1:s + 5] = ' + reply_ba)
                # print('use unpack to the above reply results in ' + str(struct.unpack('f', str(reply_ba[s+17:s+21]))[0]))
                # # print('use calsize to the above reply results in ' + str(struct.calcsize(str(reply_ba[s+1:s+5]))))
                # # print('struct.calcsize = ' + str(struct.calcsize(str(reply_ba[s + 1:s + 5]))))
                # ###########

                for idx in range(self._n_port_handles):
                    if found_ID == self._port_handles[idx].portHandle_ID:
                        # Updates quaternion
                        self._port_handles[idx].updateRot(np.array([
                            struct.unpack('<f', reply_ba[s+1:s+5])[0],
                            struct.unpack('<f', reply_ba[s+5:s+9])[0],
                            struct.unpack('<f', reply_ba[s+9:s+13])[0],
                            struct.unpack('<f', reply_ba[s+13:s+17])[0]
                        ]))

                        # Updates position
                        self._port_handles[idx].updateTrans(np.array([
                            struct.unpack('<f', reply_ba[s + 17:s + 21])[0],
                            struct.unpack('<f', reply_ba[s + 21:s + 25])[0],
                            struct.unpack('<f', reply_ba[s + 25:s + 29])[0]
                        ]))

                        self._port_handles[idx].updateError(struct.unpack('f', reply_ba[s+29:s+33])[0])
                        sensorStatus = hex(struct.unpack('I', reply_ba[s + 33:s + 37])[0])
                        self._port_handles[idx].updateStatusComplete(sensorStatus)
                        self._port_handles[idx].updateSensorStatus(sensorStatus)
                        self._port_handles[idx].updateFrameNumber(struct.unpack('I', reply_ba[s+37:s+41])[0])
                        break

    def sensorData_write_FileName(self, path, name):
        self._port_measurements = []
        self._port_measurements_path = path + name

    def sensorData_write_FileIni(self):
        # k = 0
        for ith_port_handle in range(self._n_port_handles):
            # k = k + 1
            self._port_measurements.append(open(self._port_measurements_path, 'a'))
            # self._port_measurements.append(open(self._port_measurements_path + "_" + str(k), 'a'))
            # print("self._port_measurements_path")
            # print(self._port_measurements_path)

    def sensorData_write(self, n, data_wrt_Ref=False):
        # self.sensorData_write_FileClose()
        # self.sensorData_write_FileIni()

        for ith_port_handel in range(self._n_port_handles):
            # q0, q1, q2, q3, x, y, z, error, frame, port_handle, ldfNr, time
            if data_wrt_Ref:
                q, t = self.transformation_measurement2ref(ith_port_handel)
            else:
                q = self._port_handles[ith_port_handel].getRot()
                t = self._port_handles[ith_port_handel].getTrans()

            self._port_measurements[ith_port_handel].write(
                '%1.8f,\t' % q[0] +
                '%1.8f,\t' % q[1] +
                '%1.8f,\t' % q[2] +
                '%1.8f,\t' % q[3] +
                '%3.8f,\t' % t[0] +
                '%3.8f,\t' % t[1] +
                '%3.8f,\t' % t[2] +
                '%2.8f,\t' % self._port_handles[ith_port_handel].getError() +
                str(self._port_handles[ith_port_handel].getFrameNumber()) + ',\t' +
                str(self._port_handles[ith_port_handel].portHandle_ID) + ',\t' +
                str(n) + ',\t' +
                str(int(time.time())) + '\n'
            )
            # https://docs.python.org/2/library/os.html#os.fsync
            self._port_measurements[ith_port_handel].flush()
            os.fsync(self._port_measurements[ith_port_handel].fileno())
            # time.sleep(0.1)
            # print("\033[93m" + "the " + str(n) + "-th data of port_handle nb " + str(ith_port_handel) + " was stored" + " \033[0m")

    def sensorData_get(self, data_wrt_Ref=False):
        ret = []
        for ith_port_handel in range(self._n_port_handles):
            # q0, q1, q2, q3, x, y, z, error, frame, port_handle, ldfNr, time
            if data_wrt_Ref:
                q, t = self.transformation_measurement2ref(ith_port_handel)
            else:
                q = self._port_handles[ith_port_handel].getRot()
                t = self._port_handles[ith_port_handel].getTrans()

            ret += [[q, t]]
        return ret

    def sensorData_collectData(self, n_times=10):
        # Collect Data and afterwards compute average/least square

        n_port_handle = self._n_port_handles

        q0 = np.ndarray((n_times, n_port_handle), float)
        q1 = np.ndarray((n_times, n_port_handle), float)
        q2 = np.ndarray((n_times, n_port_handle), float)
        q3 = np.ndarray((n_times, n_port_handle), float)
        x = np.ndarray((n_times, n_port_handle), float)
        y = np.ndarray((n_times, n_port_handle), float)
        z = np.ndarray((n_times, n_port_handle), float)

        # x_mean = np.ndarray((1, n_port_handle), float)
        # y_mean = np.ndarray((1, n_port_handle), float)
        # z_mean = np.ndarray((1, n_port_handle), float)
        # xi0 = np.ndarray((1, n_port_handle), float)
        # xi1 = np.ndarray((1, n_port_handle), float)
        # xi2 = np.ndarray((1, n_port_handle), float)
        # xi3 = np.ndarray((1, n_port_handle), float)

        for n_th in range(n_times):
            self.sensorData_updateAll()

            for ith_port_handel in range(self._n_port_handles):
                # q0, q1, q2, q3, x, y, z, error, frame, port_handle, ldfNr, time
                q = self._port_handles[ith_port_handel].getRot()
                t = self._port_handles[ith_port_handel].getTrans()

                q0[n_th, ith_port_handel] = q[0]
                q1[n_th, ith_port_handel] = q[1]
                q2[n_th, ith_port_handel] = q[2]
                q3[n_th, ith_port_handel] = q[3]
                x[n_th, ith_port_handel] = t[0]
                y[n_th, ith_port_handel] = t[1]
                z[n_th, ith_port_handel] = t[2]

        for ith_port_handel in range(self._n_port_handles):
            # assuming error ellipsoid
            x_mean = np.mean(x[:, ith_port_handel])
            y_mean = np.mean(y[:, ith_port_handel])
            z_mean = np.mean(z[:, ith_port_handel])

            # assuming near zero error: 1 = xi0*q0 + xi1*q1 + xi2*q2 + xi3*q3
            Jr = np.transpose(np.array([
                q0[:, ith_port_handel],
                q1[:, ith_port_handel],
                q2[:, ith_port_handel],
                q3[:, ith_port_handel]
            ]))
            Jr_pinv = np.linalg.pinv(Jr)
            xi = Jr_pinv.dot(np.ones((n_times, 1)))
            xi_norm = np.sqrt(xi[0]**2 + xi[1]**2 + xi[2]**2 + xi[3]**2)

            xi0 = xi[0] / xi_norm
            xi1 = xi[1] / xi_norm
            xi2 = xi[2] / xi_norm
            xi3 = xi[3] / xi_norm

            self.sensorData_overwriteData(ith_port_handel, x_mean, y_mean, z_mean, xi0, xi1, xi2, xi3)

    def sensorData_overwriteData(self, handel, x, y, z, q0, q1, q2, q3):
        # Updates quaternion
        self._port_handles[handel].updateRot(np.array([q0, q1, q2, q3]))

        # Updates position
        self._port_handles[handel].updateTrans(np.array([x, y, z]))

    def sensorData_write_FileClose(self):
        for ith_port_handel in range(self._n_port_handles):
            self._port_measurements[ith_port_handel].close()

    def quaternion_conj(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def quaternion_mul(self, q1, q0):
        w0, x0, y0, z0 = q0
        w1, x1, y1, z1 = q1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                         x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                         -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                         x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0
                         ])

    def transformation_affine(self, q_ref, t_ref, q, t):

        t_quaternion = np.array([0.0, t[0] - t_ref[0], t[1] - t_ref[1], t[2] - t_ref[2]])

        t_inRef = self.quaternion_mul(self.quaternion_conj(q_ref), self.quaternion_mul(t_quaternion, q_ref))
        q_inRef = self.quaternion_mul(self.quaternion_conj(q_ref), q)

        if q_inRef[0] < 0:
            q_inRef = -q_inRef

        return q_inRef, np.array([t_inRef[1], t_inRef[2], t_inRef[3]])

    def transformation_measurement2ref(self, ith_port_handel=0):

        # q0, q1, q2, q3, x, y, and z of i-th port handel
        q = self._port_handles[ith_port_handel].getRot()
        t = self._port_handles[ith_port_handel].getTrans()

        # q0_ref, q1_ref, q2_ref, q3_ref, x_ref, y_ref, and z_ref of i-th port handel reference frame
        q_ref = self._port_handles[ith_port_handel].getRot_ref()
        t_ref = self._port_handles[ith_port_handel].getTrans_ref()

        q_inRef, t_inRef = self.transformation_affine(q_ref, t_ref, q, t)

        self.sensorData_overwriteData(
            ith_port_handel,
            t_inRef[0], t_inRef[1], t_inRef[2],
            q_inRef[0], q_inRef[1], q_inRef[2], q_inRef[3]
        )
        return q_inRef, t_inRef

    def portHandle_setRefFrame(self, q_ref=np.array([1.0, 0.0, 0.0, 0.0]), t_ref=np.array([0.0, 0.0, 0.0]), ith_port_handel=0):

        self._port_handles[ith_port_handel].updateTrans_ref(t_ref)
        self._port_handles[ith_port_handel].updateRot_ref(q_ref)

    def _APIREV(self):
        return self._send_cmd("APIREV ")

    def _BEEP(self, number):
        if number > 9:
            number = 9
        elif number < 1:
            number = 1
        return self._send_cmd("BEEP " + str(number))

    def _BX(self, reply_option='0001', get_replay=False):
        """
        Returns the latest tool transformations and system status information in binary format.

        :param reply_option:
        :param get_replay:
        :return:
        """
        # Sensor reading options (source: Aurora_API_Guide page 12)
        # READ_OUT_OF_VOLUME_NOT_ALLOWED = '0001'
        # READ_OUT_OF_VOLUME_ALLOWED     = '0800'  # '0801'

        return self._send_cmd(cmd="BX " + reply_option, sec=0.1, get_replay=get_replay)

    def _COMM(self, baut_rate_s='0', data_bits_s='0', parity_s='0', stop_bits_s='0', hardware_handshaking_s='0'):
        """
        Returns exactly what is sent with the command.

        For further information on the serial port parameters, refer to the Aurora_API_Guide page 16.

        :param baut_rate_s: The data transmission rate between the Aurora System and the host computer, in bits per
        second.
        :param data_bits_s: The data bits parameter must be set to 8 bits in order to use the BX command.
        :param parity_s:
        :param stop_bits_s:
        :param hardware_handshaking_s:
        :return:
        """

        return self._send_cmd(
            "COMM " + baut_rate_s + data_bits_s + parity_s + stop_bits_s + hardware_handshaking_s,
            get_replay=True
        )

    def _ECHO(self, msg):
        """
        Returns exactly what is sent with the command.

        :param msg: Four or more ASCII characters
        :return:
        """
        return self._send_cmd("ECHO " + msg)

    def _GET(self, user_parameter_name):
        """
        The GET command returns the values of user parameters.
        (The only user parameter values currently available are the timeouts for the API commands.)

        :param user_parameter_name: only Info.Timeout.PINIT currently available
        :return:
        """
        return self._send_cmd("GET " + user_parameter_name)

    def _INIT(self):
        """
        Initializes the system.

        :return:
        """

        return self._send_cmd("INIT ")

    def _LED(self, port_handle, led_number, state):
        """
        Changes the state of visible LEDs on a tool.

        For further information on the parameters, refer to the Aurora_API_Guide page 23.

        :param port_handle: 2 hexadecimal characters
        :param led_number: Specifies the LED. The LED number does not correspond to the GPIO line number. For
        example, LED 1 corresponds to the first LED on the tool, which may not be on GPIO line 1.
        :param state: Sets the state of the specified LED. B Blank (not on), F Flash, and S soldi on
        :return:
        """
        # function reply = LED(obj, port_handle, led_number, state)
        #     reply = obj.sendCommandAndGetReply(sprintf('LED %s%s%s', port_handle, led_number, state));
        # end
        return self._send_cmd("LED " + port_handle + str(led_number) + state)

    def _PDIS(self, port_handle):
        """
        Disables the reporting of transformations for a particular port handle.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        return self._send_cmd("PDIS " + port_handle)

    def _PENA(self, port_handle, tool_tracking_priority):
        """
        Enables the reporting of transformations for a particular port handle.

        For further information on the parameters, refer to the Aurora_API_Guide page 26.

        :param port_handle: 2 hexadecimal characters
        :param tool_tracking_priority: Describes the type of tool. S static, D dynamic, B button.
        :return:
        """
        return self._send_cmd("PENA " + port_handle + tool_tracking_priority)

    def _PHF(self, port_handle):
        """
        Releases system resources from an unused port handle.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        self._send_cmd("PHF " + port_handle, get_replay=False)

    def _PHINF(self, port_handle, reply_option='0001'):
        """
        Returns information about the tool associated with the port handle.

        0001 Tool information
        0004 Tool part number
        0008 Switch and visible LED information
        0020 Physical port location
        0040 GPIO line definitions

        For further information on the parameters, refer to the Aurora_API_Guide from page 28 to 32.

        :param port_handle: 2 hexadecimal characters
        :param reply_option:
        :return:
        """
        return self._send_cmd("PHINF " + port_handle + reply_option)

    def _PHSR(self, reply_option='00'):
        """
        Returns the number of assigned port handles and the port status for each one.
        Assigns a port handle to a tool.

        For further information on the parameters, refer to the Aurora_API_Guide from page 33 to 34.

        :param reply_option: Specifies which information will be returned.
        :return: Retrieves the list of all available Port Handles, with their ID and Status,
        and initialize the port_handles member variable
        """

        # PHSR Reply options (source: Aurora_API_Guide page 33)
        # PHSR_HANDLES_ALL = '00'
        # PHSR_HANDLES_TO_BE_FREED = '01'
        # PHSR_HANDLES_OCCUPIED = '02'
        # PHSR_HANDLES_OCCUPIED_AND_INITIALIZED = '03'
        # PHSR_HANDLES_ENABLED = '04'

        reply = self._send_cmd("PHSR " + reply_option, get_replay=True)
        self.printErrorMessage(reply)
        return reply

    def _PINIT(self, port_handle):
        """
        Initializes a port handle.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        return self._send_cmd("PINIT " + port_handle)

    def _PPRD(self, port_handle, srom_device_address):
        """
        Reads data from the SROM device of a tool.

        :param port_handle: 2 hexadecimal characters
        :param srom_device_address: 4 hexadecimal characters
        :return:
        """
        return self._send_cmd("PPRD " + port_handle + srom_device_address)

    def _PPWR(self, port_handle, srom_device_address, srom_device_data):
        """
        Writes data to the SROM device in a tool.

        For further information on the parameters, refer to the Aurora_API_Guide page 40.

        :param port_handle: 2 hexadecimal characters
        :param srom_device_address: 4 hexadecimal characters
        :param srom_device_data: 64 bytes (128 hexadecimal characters) of data
        :return:
        """
        return self._send_cmd("PPWR " + port_handle + srom_device_address + srom_device_data)

    def _PSEL(self, port_handle, tool_srom_device_id):
        """
        Selects an SROM device target for a tool.

        :param port_handle: 2 hexadecimal characters
        :param tool_srom_device_id: 16 characters
        :return:
        """
        return self._send_cmd("PSEL " + port_handle + tool_srom_device_id)

    def _PSOUT(self, port_handle, gpio_1_state, gpio_2_state, gpio_3_state):
        """
        Sets the states of the general purpose input/output (GPIO) lines in a tool.

        :param port_handle: 2 hexadecimal characters
        :param gpio_1_state: N no change, S solid on, P pule, and O off
        :param gpio_2_state: N no change, S solid on, P pule, and O off
        :param gpio_3_state: N no change, S solid on, P pule, and O off
        :return:
        """
        return self._send_cmd("PSOUT " + port_handle + gpio_1_state + gpio_2_state + gpio_3_state)

    def _PSRCH(self, port_handle):
        """
        Returns a list of valid SROM device IDs for a tool.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        return self._send_cmd("PSRCH " + port_handle)

    def _PURD(self, port_handle, user_srom_device_address):
        """
        Returns the tool information in the user section of the SROM device in a tool.

        For further information on the parameters, refer to the Aurora_API_Guide page 44.

        :param port_handle: 2 hexadecimal characters
        :param user_srom_device_address: 4 hexadecimal characters
        :return:
        """
        return self._send_cmd("PURD " + port_handle + user_srom_device_address)

    def _PUWR(self, port_handle, user_srom_device_address, user_srom_device_data):
        """
        Writes data to the user section of the SROM device in a tool.

        For further information on the parameters, refer to the Aurora_API_Guide page 48.

        :param port_handle: 2 hexadecimal characters
        :param user_srom_device_address: 4 hexadecimal characters
        :param user_srom_device_data: 64 bytes of data to write (128 hexadecimal characters)
        :return:
        """
        return self._send_cmd("PUWR " + port_handle + user_srom_device_address + user_srom_device_data)

    def _PVWR(self, port_handle, start_address, tool_definition_data):
        """
        Allows you to upload a tool definition file for a tool. The Aurora System will use the data in the
        uploaded tool definition file instead of the data in the tool's SROM device.

        :param port_handle: 2 hexadecimal characters
        :param start_address: 4 hexadecimal characters.
        Increment the start address by 64 bytes with each chunk of data sent
        for a particular port handle.
        :param tool_definition_data: 128 hexadecimal characters
        :return:
        """
        return self._send_cmd("PVWR " + port_handle + start_address + tool_definition_data)

    def _RESET(self, reset_option='0'):
        """
        Resets the system.

        RESET_SOFT: Generates a "soft" reset and resets the baud rate to 9600. System will not beep.
        RESET_HART: Same behaviour as resetting the system with a serial break

        :param reset_option: Specifies the type of reset.
        :return:
        """
        # Reset options
        # RESET_SOFT = '0'
        # RESET_HARD = '1'
        return self._send_cmd("RESET " + reset_option)

    def _SFLIST(self, reply_option):
        """
        Returns information about the supported features of the system.

        For further information on the parameters and the reply, refer to the Aurora_API_Guide form page 54 to 58.

        :param reply_option: Specifies which information will be returned.
        :return:
        """
        return self._send_cmd("SFLIST " + reply_option)

    def _TSTART(self, reply_option=None):
        """
        Starts Tracking mode.

        For further information on the parameter, refer to the Aurora_API_Guide page 59.

        :param reply_option: 40 starts tracking in faster acquisition mode, and 80 resets the frame counter to zero.
        :return:
        """
        # Tracking mode options (source: Aurora_API_Guide page 59)
        # TRACKING_OPTION_NONE = ''
        # TRACKING_OPTION_FAST_MODE = '40'
        # TRACKING_OPTION_RESET_COUNTER = '80'
        # TRACKING_OPTION_FAST_MODE_RESET_COUNTER = 'C0'

        if reply_option is None:
            return self._send_cmd("TSTART ")
        else:
            return self._send_cmd("TSTART " + reply_option)

    def _TSTOP(self):
        """
        Stops Tracking mode.

        :return:
        """
        return self._send_cmd("TSTOP ")

    def _TTCFG(self, port_handle):
        """
        Sets up a configuration for a tool, so that you can test the tool without using a tool definition file.

        For further information on the parameters, refer to the Aurora_API_Guide page 62.

        :param port_handle: 2 hexadecimal characters
        :return:
        """
        return self._send_cmd("TTCFG " + port_handle)

    def _TX(self, reply_option='0001'):
        """
        Returns the latest tool transformations and system status in text format.

        For further information on the parameters and the reply, refer to the Aurora_API_Guide form page 54 to 58.

        :param reply_option: 0001 Transformation data, 0800 Transformation not normally reported
        :return:
        """
        return self._send_cmd("TX " + reply_option)

    def _VER(self, reply_option):
        """
        Returns the firmware revision number of critical processors installed in the system.

        For further information on the parameters and the reply, refer to the Aurora_API_Guide form page 67 to 68.

        :param reply_option: Specifies which information will be returned.
        :return:
        """
        return self._send_cmd("VER " + reply_option)

    def _VSEL(self, volume_number):
        """
        Selects a characterized measurement volume.

        :param volume_number: 1 hexadecimal character. Valid values are 1 to the maximum returned by _SFLIST.
        :return:
        """
        return self._send_cmd("VSEL " + str(volume_number))

    def printErrorCode(self, errorCode):
        # (source: Aurora_API_Guide from page 70 to 72)
        errorCode_msg = {
            '01': 'Invalid command.',
            '02': 'Command too long.',
            '03': 'Command too short.',
            '04': 'Invalid CRC calculated for command; calculated CRC does not match the one sent.',
            '05': 'Time-out on command execution.',
            '06': 'Unable to set up new communication parameters. '
                  'This occurs if one of the communication parameters is out of range.',
            '07': 'Incorrect number of parameters.',
            '08': 'Invalid port handle selected.',
            '09': 'Invalid mode selected. Either the tool tracking priority is out of range, '
                  'or the tool has sensor coils defined and "button box" was selected.',
            '0A': 'Invalid LED selected. The LED selected is out of range.',
            '0B': 'Invalid LED state selected. The LED state selected is out of range.',
            '0C': 'Command is invalid while in the current operating mode.',
            '0D': 'No tool is assigned to the selected port handle.',
            '0E': 'Selected port handle not initialized. '
                  'The port handle needs to be initialized before the command is sent.',
            '0F': 'Selected port handle not enabled. '
                  'The port handle needs to be enabled before the command is sent.',
            '10': 'System not initialized. The system must be initialized before the command is sent.',
            '11': 'Unable to stop tracking. This occurs if there are hardware problems. Please contact NDI.',
            '12': 'Unable to start tracking. This occurs if there are hardware problems. Please contact NDI.',
            '13': 'Unable to initialize the port handle.',
            '14': 'Invalid Field Generator characterization parameters or incompatible hardware',
            '15': 'Unable to initialize the system. '
                  'This occurs if the system could not retrun to Setup mode or there are internal hardware problem. '
                  'Please contact NDI if hardware problem are detected.',
            '16': 'Unable to start Diagnostic mode. This occurs if there are hardware problems. Please contact NDI.',
            '17': 'Unable to stop Diagnostic mode. This occurs if there are hardware problems. Please contact NDI',
            '19': 'Unable to read devices firmware revision information. '
                  'This occurs if the processor selected is out of range or '
                  'the system is unable to inquire firmware revision information form a processor.',
            '1A': 'Internal system error. '
                  'This occurs when the system is unable to recover after a system processing exception',
            '1D': 'Unable to search for SROM device IDs.',
            '1E': 'Unable to read SROM device data, see Aurora_API_Guide on page 71',
            '1F': 'Unable to write SROM device data, see Aurora_API_Guide on page 71',
            '20': 'Unable to select SROM device for given port handle and SROM device ID.',
            '22': 'Enabled tools are not supported by selected volume parameters.',
            '23': 'Command parameter is out of range.',
            '24': 'Unable to select parameters by volume, see Aurora_API_Guide on page 71',
            '25': 'Unable to determine the systems supported features list. '
                  'This occurs if the system is unable to read all the hardware information.',
            '29': 'Main processor firmware is corrupt.',
            '2A': 'No memory is available for dynamic allocation (heap is full).',
            '2B': 'The requested port handle has not been allocated.',
            '2C': 'The requested port handle has become unoccupied.',
            '2D': 'All handles have been allocated.',
            '2E': 'Incompatible firmware versions, see Aurora_API_Guide on page 71',
            '31': 'Invalid input or output state.',
            '32': 'Invalid operation for the device associated with the specified port handle.',
            '33': 'Feature not available.',
            'C2': 'Command not supported by proxy.',
            'C3': 'Offline fit not possible, eg no or incompatible DLL.',
            'C4': 'Incompatible replies from two SCUs, eg different firmware revisions.',
            'F4': 'Unable to erase Flash SROM device.',
            'F5': 'Unable to write Flash SROM device',
            'F6': 'Unable to read Flash SROM device.'
        }.get(errorCode, 'Reserved')

        print("\033[93m" +
              #"Error Code " + errorCode + " correspondence to the following message: \n" +
              errorCode_msg +
              " \033[0m")

    def printErrorMessage(self, msg):
        msg_error = msg.split('\r')[0]
        if msg_error[:5] == 'ERROR':
            self.printErrorCode(msg_error[5:7])


class PortHandle:
    # Handle Status(source: Aurora_API_Guide page 13)
    SENSOR_STATUS_VALID = '01'
    SENSOR_STATUS_MISSING = '02'
    SENSOR_STATUS_DISABLED = '04'

    def __init__(self, portHandle_ID, portHandle_status):
        # Port Handle ID, given as a 2 character hex number(string e.g. '0A' and '0B')
        self.portHandle_ID = portHandle_ID
        self.portHandle_status = portHandle_status

        # Port Handle status variables (source: Aurora_API_Guide page 34)
        self._occupied = False
        self._initialized = False
        self._enabled = False
        self._gpio_line1_closed = False
        self._gpio_line2_closed = False
        self._gpio_line3_closed = False
        self._out_of_volume = False
        self._partial_out_of_volume = False
        self._sensor_broken = False

        # Handle Status (source: Aurora_API_Guide page 13)
        # '00000000' ist int(0)
        self._sensor_status = hex(0)

        # Last registered tool data
        self._trans = np.array([0.0, 0.0, 0.0])
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._error = 0.0
        self._frame_number = 0

        # referenz frame
        self._trans_ref = np.array([0.0, 0.0, 0.0])
        self._quaternion_ref = np.array([1.0, 0.0, 0.0, 0.0])


    # def __del__(self):
    #     pass

    def _get_bit(self, byte_val, idx):
        """
        Typically, the least-significant bit is bit index 0 and the most-significant bit is bit index 7. Using this
        terminology, we can determine if bit index k is set by taking the bitwise-and with 1 shifted to the left by k.
        If the bitwise and is non-zero, then that means that index k has a 1; otherwise, index k has a 0.

        :param byteval: a byte or a integer
        :param idx: a number
        :return: True or False
        """
        return (byte_val & (1 << idx)) != 0

    def _printSensorStatus(self):
        """
        Prints the status of the given sensor.

        bit 0:      System communication synchronization error
        bit 1:      Reserved
        bit 2:      System CRC error
        bit 3:      Recoverable system processing exception
        bit 4:      Hardware failure
        bit 5:      Hardware change.
                    This bit is set if the Field Generator is disconnected from the System Control Unit.
        bit 6:      Some port handle has become occupied
        bit 7:      Some port handle has become unoccupied
        bit 8-15:   Reserved

        :return:
        """
        status = int(self._sensor_status, 16)
        msg_0 = 'Sensor status: '
        msg = ''
        msg += msg_0

        if self._get_bit(status, 0):
            msg += 'System communication synchronization error\n'
        if self._get_bit(status, 2):
            msg += 'System CRC error\n'
        if self._get_bit(status, 3):
            msg += 'Recoverable system processing exception\n'
        if self._get_bit(status, 4):
            msg += 'Hardware failure\n'
        if self._get_bit(status, 5):
            msg += 'Hardware change. ' \
                 'This bit is set if the Field Generator is disconnected from the System Control Unit.\n'
        if self._get_bit(status, 6):
            msg += 'Some port handle has become occupied\n'
        if self._get_bit(status, 7):
            msg += 'Some port handle has become unoccupied\n'

        if msg_0 == msg:
            print("\033[93m" +
                  msg_0 + 'None'
                  " \033[0m")
        else:
            print("\033[93m" +
                  msg +
                  " \033[0m")

    def updateStatus(self, status_hex):
        status_int = int(status_hex, 16)
        self._occupied = self._get_bit(byte_val=status_int, idx=0)
        self._gpio_line1_closed = self._get_bit(byte_val=status_int, idx=1)
        self._gpio_line2_closed = self._get_bit(byte_val=status_int, idx=2)
        self._gpio_line3_closed = self._get_bit(byte_val=status_int, idx=3)
        self._initialized = self._get_bit(byte_val=status_int, idx=4)
        self._enabled = self._get_bit(byte_val=status_int, idx=5)

    def updateStatusComplete(self, status_hex):
        status_int = int(status_hex, 16)
        self._occupied = self._get_bit(byte_val=status_int, idx=0)
        self._gpio_line1_closed = self._get_bit(byte_val=status_int, idx=1)
        self._gpio_line2_closed = self._get_bit(byte_val=status_int, idx=2)
        self._gpio_line3_closed = self._get_bit(byte_val=status_int, idx=3)
        self._initialized = self._get_bit(byte_val=status_int, idx=4)
        self._enabled = self._get_bit(byte_val=status_int, idx=5)
        self._out_of_volume = self._get_bit(byte_val=status_int, idx=6)
        self._partial_out_of_volume = self._get_bit(byte_val=status_int, idx=7)
        self._sensor_broken = self._get_bit(byte_val=status_int, idx=8)

    def updateSensorStatus(self, sensor_status):
        self._sensor_status = sensor_status

    def updateTrans(self, trans):
        self._trans = trans

    def updateRot(self, quaternion):
        self._quaternion = quaternion

    def updateTrans_ref(self, trans):
        self._trans_ref = trans

    def updateRot_ref(self, quaternion):
        self._quaternion_ref = quaternion

    def updateError(self, error):
        self._error = error

    def updateFrameNumber(self, frame_number):
        self._frame_number = frame_number

    def getTrans(self):
        return self._trans

    def getRot(self):
        return self._quaternion

    def getTrans_ref(self):
        return self._trans_ref

    def getRot_ref(self):
        return self._quaternion_ref

    def getFrameNumber(self):
        return self._frame_number

    def getError(self):
        return self._error
