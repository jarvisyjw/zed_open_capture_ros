import pyzed.sl as sl

def main():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera: {}".format(err))
        exit(1)
    
    camera_info = zed.get_camera_infomation()
    transform = camera_info.sensors_configuration.camera_imu_transforms
    print("Left Camera Optical Frame to IMU Frame Transform.")
    print(transform.m)
    zed.close()

if __name__ == "__main__":
    main()