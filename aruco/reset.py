import pyrealsense2 as rs

def reset_camera_settings():
    # Create a context object. This object owns the handles to all connected realsense devices
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        # The following method is used to reset the hardware device
        print("Resetting device:", dev.get_info(rs.camera_info.name))
        dev.hardware_reset()

if __name__ == '__main__':
    reset_camera_settings()
