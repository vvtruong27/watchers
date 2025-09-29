# FPT Hackathon 2025 - Team Watchers

## 🚀 Cách chạy chương trình

### Yêu cầu hệ thống
- Ubuntu 18.04/20.04/22.04
- ROS Melodic/Noetic
- Python 3.6+
- JetBot hardware với camera và LiDAR

### Cài đặt dependencies

```bash
# Cài đặt các package Python cần thiết
pip install -r requirements.txt

# Cài đặt ROS dependencies
sudo apt-get install ros-melodic-cv-bridge ros-melodic-sensor-msgs
```

### Chạy chương trình

1. **Khởi động ROS Master:**
```bash
roscore
```

2. **Khởi động JetBot hardware:**
```bash
# Terminal 1: Khởi động camera
roslaunch jetbot_pro csi_camera.launch

# Terminal 2: Khởi động LiDAR (nếu có)
roslaunch jetbot_pro lidar.launch
```

3. **Chạy Problem A:**
```bash
# Terminal 3: Chạy chương trình chính
cd watchers/solutions
python3 problem_a.py
```

### Quy trình hoạt động

1. **Khởi tạo**: Chương trình sẽ tự động lấy bản đồ từ API
2. **Kiểm tra giao lộ**: Sử dụng LiDAR để xác nhận vị trí khởi đầu
3. **Xác nhận người dùng**: Nhập 'y' để bắt đầu navigation
4. **Đọc QR**: Quét mã QR để xác nhận vị trí start
5. **Navigation**: Tự động điều hướng theo đường đã vạch
6. **Gửi API**: Tự động gửi dữ liệu QR lên server khi quét thành công

### Cấu trúc thư mục

```
watchers/
├── solutions/           # Code chính
│   ├── problem_a.py    # Chương trình Problem A
│   ├── map_api.py      # API lấy bản đồ
│   ├── edge_detection.py # Xử lý LiDAR
│   ├── jetbot_control.py # Điều khiển robot
│   └── config.py       # Cấu hình
├── sample_code/        # Code mẫu từ BTC
├── requirements.txt    # Dependencies Python
└── README.md          # File này
```

### Xử lý lỗi thường gặp

- **Lỗi camera**: Kiểm tra kết nối camera và topic `/csi_cam_0/image_raw`
- **Lỗi LiDAR**: Kiểm tra kết nối LiDAR và topic `/scan`
- **Lỗi API**: Kiểm tra kết nối internet và token trong `config.py`
- **Lỗi QR**: Đảm bảo QR code rõ nét và đủ ánh sáng

### Liên hệ
- Team: Watchers
- Email: [email của team]
- GitHub: [link repository]
