import struct
import codecs


RPLIDAR_SYNC_BYTE1 = b'\xA5'
RPLIDAR_SYNC_BYTE2 = b'\x5A'

RPLIDAR_DESCRIPTOR_LEN = 7

RPLIDAR_SEND_MODE_SINGLE_RES     = b'\x00'
RPLIDAR_SEND_MODE_MULTIPLE_RES   = b'\x01'

RPLIDAR_CMD_STOP            = b'\x25'
RPLIDAR_CMD_RESET           = b'\x40'
RPLIDAR_CMD_SCAN            = b'\x20'
RPLIDAR_CMD_EXPRESS_SCAN    = b'\x82'
RPLIDAR_CMD_FORCE_SCAN      = b'\x21'
RPLIDAR_CMD_GET_INFO        = b'\x50'
RPLIDAR_CMD_GET_HEALTH      = b'\x52'
RPLIDAR_CMD_GET_SAMPLERATE  = b'\x59'
RPLIDAR_CMD_GET_LIDAR_CONF  = b'\x84'

RPLIDAR_CMD_HQ_MOTOR_SPEED_CTRL = b'\xA8'
RPLIDAR_CMD_SET_MOTOR_PWM       = b'\xF0'
RPLIDAR_CMD_GET_ACC_BOARD_FLAG  = b'\xFF'


RPLIDAR_MAX_MOTOR_PWM       = 1023
RPLIDAR_DEFAULT_MOTOR_PWM   = 660


RPLIDAR_CONF_SCAN_MODE_COUNT            = 0x00000070
RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE    = 0x00000071
RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE     = 0x00000074
RPLIDAR_CONF_SCAN_MODE_ANS_TYPE         = 0x00000075
RPLIDAR_CONF_SCAN_MODE_TYPICAL          = 0x0000007C
RPLIDAR_CONF_SCAN_MODE_NAME             = 0x0000007F


RPLIDAR_STATUS = {
    0: "GOOD",
    1: "WARNING",
    2: "ERROR"
}

RPLIDAR_ANS_TYPE = {
    0x81: "NORMAL",
    0x82: "CAPSULED",
    0x83: "HQ",
    0x84: "ULTRA_CAPSULED",
    0x85: "DENSE_CAPSULED"
}


RPLIDAR_VARBITSCALE_X2_SRC_BIT = 9
RPLIDAR_VARBITSCALE_X4_SRC_BIT = 11
RPLIDAR_VARBITSCALE_X8_SRC_BIT = 12
RPLIDAR_VARBITSCALE_X16_SRC_BIT = 14

RPLIDAR_VARBITSCALE_X2_DEST_VAL = 512
RPLIDAR_VARBITSCALE_X4_DEST_VAL = 1280
RPLIDAR_VARBITSCALE_X8_DEST_VAL = 1792
RPLIDAR_VARBITSCALE_X16_DEST_VAL = 3328

VBS_SCALED_BASE = [RPLIDAR_VARBITSCALE_X16_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X8_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X4_DEST_VAL,
                   RPLIDAR_VARBITSCALE_X2_DEST_VAL,
                   0]

VBS_SCALED_LVL = [4, 3, 2, 1, 0]

VBS_TARGET_BASE = [(0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
                   (0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
                   0]



class PyRPlidarConnectionError(Exception):
    pass

class PyRPlidarProtocolError(Exception):
    pass



class PyRPlidarCommand:

    def __init__(self, cmd, payload=None):
        self.cmd = cmd
        self.payload = payload
        self.raw_bytes = RPLIDAR_SYNC_BYTE1 + cmd
        
        if payload is not None:
            size = struct.pack('B', len(payload))
            self.raw_bytes += size + payload
            self.raw_bytes += struct.pack('B', self.get_checksum(self.raw_bytes))

    def get_checksum(self, data):
        chksum = 0
        for value in data: chksum ^= value
        return chksum


class PyRPlidarResponse:
    
    def __init__(self, raw_bytes):
        self.sync_byte1 = raw_bytes[0]
        self.sync_byte2 = raw_bytes[1]
        size_q30_length_type = struct.unpack("<L", raw_bytes[2:6])[0]
        self.data_length = size_q30_length_type & 0x3FFFFFFF
        self.send_mode = size_q30_length_type >> 30
        self.data_type = raw_bytes[6]

    def __str__(self):
        data = {
            "sync_byte1" : hex(self.sync_byte1),
            "sync_byte2" : hex(self.sync_byte2),
            "data_length" : self.data_length,
            "send_mode" : self.send_mode,
            "data_type" : hex(self.data_type)
        }
        return str(data)






class PyRPlidarDeviceInfo:
    
    def __init__(self, raw_bytes):
        self.model = raw_bytes[0]
        self.firmware_minor = raw_bytes[1]
        self.firmware_major = raw_bytes[2]
        self.hardware = raw_bytes[3]
        self.serialnumber = codecs.encode(raw_bytes[4:], 'hex').upper()
        self.serialnumber = codecs.decode(self.serialnumber, 'ascii')

    def __str__(self):
        data = {
            "model" : self.model,
            "firmware_minor" : self.firmware_minor,
            "firmware_major" : self.firmware_major,
            "hardware" : self.hardware,
            "serialnumber" : self.serialnumber
        }
        return str(data)


class PyRPlidarHealth:
    
    def __init__(self, raw_bytes):
        self.status = raw_bytes[0]
        self.error_code = (raw_bytes[1] << 8) + raw_bytes[2]

    def __str__(self):
        data = {
            "status" : self.status,
            "error_code" : self.error_code
        }
        return str(data)


class PyRPlidarSamplerate:
    
    def __init__(self, raw_bytes):
        self.t_standard = raw_bytes[0] + (raw_bytes[1] << 8)
        self.t_express = raw_bytes[2] + (raw_bytes[3] << 8)
    
    def __str__(self):
        data = {
            "t_standard" : self.t_standard,
            "t_express" : self.t_express
        }
        return str(data)


class PyRPlidarScanMode:

    def __init__(self, data_name, data_max_distance, data_us_per_sample, data_ans_type):
        self.us_per_sample = struct.unpack("<I", data_us_per_sample[4:8])[0]
        self.max_distance = struct.unpack("<I", data_max_distance[4:8])[0]
        self.ans_type = struct.unpack("<B", data_ans_type[4:5])[0]
        self.name = codecs.decode(data_name[4:-1], 'ascii')
    
    def __str__(self):
        data = {
            "name" : self.name,
            "max_distance" : self.max_distance,
            "us_per_sample" : self.us_per_sample,
            "ans_type" : RPLIDAR_ANS_TYPE[self.ans_type]
        }
        return str(data)






class PyRPlidarMeasurement:

    def __init__(self, raw_bytes=None, measurement_hq=None):
        if raw_bytes is not None:
            self.start_flag = bool(raw_bytes[0] & 0x1)
            self.quality = raw_bytes[0] >> 2
            self.angle = ((raw_bytes[1] >> 1) + (raw_bytes[2] << 7)) / 64.0
            self.distance = (raw_bytes[3] + (raw_bytes[4] << 8)) / 4.0

        elif measurement_hq is not None:
            self.start_flag = True if measurement_hq.start_flag == 0x1 else False
            self.quality = measurement_hq.quality
            self.angle = ((measurement_hq.angle_z_q14 * 90) >> 8) / 64.0
            self.distance = (measurement_hq.dist_mm_q2) / 4.0

                
    def __str__(self):
        data = {
            "start_flag" : self.start_flag,
            "quality" : self.quality,
            "angle" : self.angle,
            "distance" : self.distance
        }
        return str(data)


class PyRPlidarMeasurementHQ:
    
    def __init__(self, syncBit, angle_q6, dist_q2):
        self.start_flag = syncBit | ((not syncBit) << 1)
        self.quality = (0x2f << 2) if dist_q2 else 0
        self.angle_z_q14 = (angle_q6 << 8) // 90
        self.dist_mm_q2 = dist_q2

    def get_angle(self):
        return self.angle_z_q14 * 90.0 / 16384.0
    
    def get_distance(self):
        return self.dist_mm_q2 / 4.0




class PyRPlidarCabin:
    
    def __init__(self, raw_bytes):
        self.distance1 = (raw_bytes[0] >> 2) + (raw_bytes[1] << 6)
        self.distance2 = (raw_bytes[2] >> 2) + (raw_bytes[3] << 6)
        self.d_theta1 = (raw_bytes[4] & 0x0F) + ((raw_bytes[0] & 0x03) << 4)
        self.d_theta2 = (raw_bytes[4] >> 4) + ((raw_bytes[2] & 0x03) << 4)

class PyRPlidarScanCapsule:
    
    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.cabins = list(map(
                               PyRPlidarCabin,
                               [raw_bytes[i:i+5] for i in range(4, len(raw_bytes), 5)]
                               ))

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):
        
        nodes = []
        
        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2
        
        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += (360 << 8)
        
        angleInc_q16 = (diffAngle_q8 << 3)
        currentAngle_raw_q16 = (prevStartAngle_q8 << 8)

        for pos in range(len(capsule_prev.cabins)):
            
            dist_q2 = [0] * 2
            angle_q6 = [0] * 2
            syncBit = [0] * 2
            
            dist_q2[0] = capsule_prev.cabins[pos].distance1 << 2
            dist_q2[1] = capsule_prev.cabins[pos].distance2 << 2
            
            angle_offset1_q3 = capsule_prev.cabins[pos].d_theta1
            angle_offset2_q3 = capsule_prev.cabins[pos].d_theta2
            
            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10)
            syncBit[0] = 1 if ((currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 else 0
            currentAngle_raw_q16 += angleInc_q16
            
            
            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10)
            syncBit[1] = 1 if ((currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 else 0
            currentAngle_raw_q16 += angleInc_q16

            
            for cpos in range(2):

                if angle_q6[cpos] < 0: angle_q6[cpos] += (360 << 6)
                if angle_q6[cpos] >= (360 << 6): angle_q6[cpos] -= (360 << 6)
                
                node = PyRPlidarMeasurementHQ(syncBit[cpos], angle_q6[cpos], dist_q2[cpos])
                nodes.append(node)

        return nodes






class PyRPlidarDenseCabin:
    
    def __init__(self, raw_bytes):
        self.distance = (raw_bytes[0] << 8) + raw_bytes[1]


class PyRPlidarScanDenseCapsule:

    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.cabins = list(map(
                               PyRPlidarDenseCabin,
                               [raw_bytes[i:i+2] for i in range(4, len(raw_bytes), 2)]
                               ))

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):
        
        nodes = []
        
        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2
        
        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += (360 << 8)
        
        angleInc_q16 = (diffAngle_q8 << 8) // 40
        currentAngle_raw_q16 = (prevStartAngle_q8 << 8)

        for pos in range(len(capsule_prev.cabins)):
            
            dist_q2 = 0
            angle_q6 = 0
            syncBit = 0

            syncBit = 1 if (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) else 0
            
            angle_q6 = (currentAngle_raw_q16 >> 10)
            if angle_q6 < 0: angle_q6 += (360 << 6)
            if angle_q6 >= (360 << 6): angle_q6 -= (360 << 6)
            currentAngle_raw_q16 += angleInc_q16
            
            dist_q2 = capsule_prev.cabins[pos].distance << 2

            node = PyRPlidarMeasurementHQ(syncBit, angle_q6, dist_q2)
            nodes.append(node)

        return nodes






class PyRPlidarUltraCabin:
    
    def __init__(self, raw_bytes):
        self.major = ((int(raw_bytes[1]) & 0xF) << 8) + int(raw_bytes[0])
        self.predict1 = ((int(raw_bytes[2]) & 0x3F) << 4) + ((int(raw_bytes[1]) >> 4) & 0xF)
        self.predict2 = ((int(raw_bytes[3]) & 0xFF) << 2) + ((int(raw_bytes[2]) >> 6) & 0x3)
    
        if self.predict1 & 0x200: self.predict1 |= 0xFFFFFC00
        if self.predict2 & 0x200: self.predict2 |= 0xFFFFFC00
    
    def __str__(self):
        data = {
            "major" : hex(self.major),
            "predict1" : hex(self.predict1),
            "predict2" : hex(self.predict2),
        }
        return str(data)

class PyRPlidarScanUltraCapsule:

    def __init__(self, raw_bytes):
        self.sync_byte1 = (raw_bytes[0] >> 4) & 0xF
        self.sync_byte2 = (raw_bytes[1] >> 4) & 0xF
        self.checksum = (raw_bytes[0] & 0xF) + ((raw_bytes[1] & 0xF) << 4)
        self.start_angle_q6 = raw_bytes[2] + ((raw_bytes[3] & 0x7F) << 8)
        self.start_flag = bool((raw_bytes[3] >> 7) & 0x1)
        self.ultra_cabins = list(map(
                                     PyRPlidarUltraCabin,
                                     [raw_bytes[i:i+4] for i in range(4, len(raw_bytes), 4)]
                                     ))
    

    def __str__(self):
        data = {
            "sync_byte1" : hex(self.sync_byte1),
            "sync_byte2" : hex(self.sync_byte2),
            "checksum" : hex(self.checksum),
            "start_angle_q6" : hex(self.start_angle_q6),
            "start_flag" : self.start_flag,
            "ultra_cabins" : [str(ultra_cabin) for ultra_cabin in self.ultra_cabins]
        }
        return str(data)

    @classmethod
    def _varbitscale_decode(self, scaled):
        
        scaleLevel = 0
        
        for i in range(len(VBS_SCALED_BASE)):
            
            remain = scaled - VBS_SCALED_BASE[i]
            if remain >= 0:
                scaleLevel = VBS_SCALED_LVL[i]
                return (VBS_TARGET_BASE[i] + (remain << scaleLevel), scaleLevel)
        
        return (0, scaleLevel)

    @classmethod
    def _parse_capsule(self, capsule_prev, capsule_current):
        
        nodes = []
        
        currentStartAngle_q8 = capsule_current.start_angle_q6 << 2
        prevStartAngle_q8 = capsule_prev.start_angle_q6 << 2
        
        diffAngle_q8 = (currentStartAngle_q8)-(prevStartAngle_q8)
        if prevStartAngle_q8 > currentStartAngle_q8:
            diffAngle_q8 += (360 << 8)

        angleInc_q16 = (diffAngle_q8 << 3) // 3
        currentAngle_raw_q16 = (prevStartAngle_q8 << 8)
        
        for pos in range(len(capsule_prev.ultra_cabins)):
            
            dist_q2 = [0] * 3
            angle_q6 = [0] * 3
            syncBit = [0] * 3
            
            dist_major = capsule_prev.ultra_cabins[pos].major
            
            # signed partical integer, using the magic shift here
            # DO NOT TOUCH
            
            dist_predict1 = capsule_prev.ultra_cabins[pos].predict1
            dist_predict2 = capsule_prev.ultra_cabins[pos].predict2
            
            dist_major2 = 0
            
            # prefetch next ...
            if pos == len(capsule_prev.ultra_cabins) - 1:
                dist_major2 = capsule_current.ultra_cabins[0].major
            else:
                dist_major2 = capsule_prev.ultra_cabins[pos + 1].major
            
            
            # decode with the var bit scale ...
            dist_major, scalelvl1 = PyRPlidarScanUltraCapsule._varbitscale_decode(dist_major)
            dist_major2, scalelvl2 = PyRPlidarScanUltraCapsule._varbitscale_decode(dist_major2)
            
            
            dist_base1 = dist_major
            dist_base2 = dist_major2
            
            if not(dist_major) and dist_major2:
                dist_base1 = dist_major2
                scalelvl1 = scalelvl2
            
            dist_q2[0] = (dist_major << 2)
            if (dist_predict1 == 0xFFFFFE00) or (dist_predict1 == 0x1FF):
                dist_q2[1] = 0
            else:
                dist_predict1 = (dist_predict1 << scalelvl1)
                dist_q2[1] = ((dist_predict1 + dist_base1) << 2) & 0xFFFFFFFF
            
            if (dist_predict2 == 0xFFFFFE00) or (dist_predict2 == 0x1FF):
                dist_q2[2] = 0
            else:
                dist_predict2 = (dist_predict2 << scalelvl2)
                dist_q2[2] = ((dist_predict2 + dist_base2) << 2) & 0xFFFFFFFF
        
        
            for cpos in range(3):
                
                syncBit[cpos] = 1 if (((currentAngle_raw_q16 + angleInc_q16) % (360 << 16)) < angleInc_q16) else 0
                
                offsetAngleMean_q16 = int(7.5 * 3.1415926535 * (1 << 16) / 180.0)
                
                if dist_q2[cpos] >= (50 * 4):
                    
                    k1 = 98361
                    k2 = int(k1 / dist_q2[cpos])
                    
                    offsetAngleMean_q16 = int(8 * 3.1415926535 * (1 << 16) / 180) - int(k2 << 6) - int((k2 * k2 * k2) / 98304)
                
                angle_q6[cpos] = (currentAngle_raw_q16 - int(offsetAngleMean_q16 * 180 / 3.14159265)) >> 10
                currentAngle_raw_q16 += angleInc_q16
                
                if angle_q6[cpos] < 0: angle_q6[cpos] += (360 << 6)
                if angle_q6[cpos] >= (360 << 6): angle_q6[cpos] -= (360 << 6)
                
                node = PyRPlidarMeasurementHQ(syncBit[cpos], angle_q6[cpos], dist_q2[cpos])
                nodes.append(node)

        return nodes
