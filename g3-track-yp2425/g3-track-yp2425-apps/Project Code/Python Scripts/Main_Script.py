import CamModule as cam


test_photo_path=cam.test_camera_connection()

captured_photo_path=cam.capture_image()

# 5 is the duration in Sec
recorder_video_path=cam.record_video(5)
