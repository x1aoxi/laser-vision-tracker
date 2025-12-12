import cv2

# 初始化摄像头（使用V4L2接口）
cap = cv2.VideoCapture("/dev/video9", cv2.CAP_V4L2)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 可选：设置曝光值范围（自动模式下可能被忽略，视设备而定）
cap.set(cv2.CAP_PROP_EXPOSURE, 50)  # -1通常表示自动调整

while True:
    # 读取一帧图像
    ret, frame = cap.read()

    # 检查是否成功读取帧
    if not ret:
        print("无法获取图像")
        break

    # 显示帧
    cv2.imshow('Camera Feed (Auto Exposure)', frame)

    # 按 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源
cap.release()
# 关闭所有OpenCV窗口
cv2.destroyAllWindows()
    