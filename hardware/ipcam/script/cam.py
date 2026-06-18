import cv2
#from ultralytics import YOLO


ip_address = '192.168.0.115' #这是我们摄像头的ip地址
port = 80 #这是我们刚刚查看的端口号
username = 'admin' #这里输入你的用户名，如果你的摄像头打开需要登录的话
password = 'i2345678' #这里输入登录密码
#model = 'D:\\VScode_project\\v8_ocr_sort_diang\\ultralytics-main\\weights\\yolov8n.pt'
#rtsp_url = f'http://{username}:{password}@{ip_address}:{port}/livestream.cgi'
#rtsp_url = f'rtsp://{ip_address}:{port}/stream'
rtsp_url = f'rtsp://admin:i2345678@192.168.0.115/11'
# http://192.168.0.115:80/livestream.cgi?stream=11&action=play&media=video_audio_data HTTP/1.1

cap = cv2.VideoCapture(rtsp_url)
if not cap.isOpened():
    print("无法连接到摄像头！")
    exit()
while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()