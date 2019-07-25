from pyrplidar import PyRPlidar


def check_connection():

    lidar = PyRPlidar()
    lidar.connect(port="/dev/cu.SLAB_USBtoUART",
                  baudrate=256000,
                  timeout=5)

                  
    info = lidar.get_info()
    print("info :", info)
    
    health = lidar.get_health()
    print("health :", health)
    
    samplerate = lidar.get_samplerate()
    print("samplerate :", samplerate)
    
    
    scan_modes = lidar.get_scan_modes()
    print("scan modes :")
    for scan_mode in scan_modes:
        print(scan_mode)

    lidar.disconnect()



if __name__ == "__main__":
    check_connection()
