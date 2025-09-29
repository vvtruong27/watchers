#!/usr/bin/env python3
"""
Problem A - FPT Hackathon 2025  
Giải bài toán điều khiển JetBot đi từ start đến end theo đường đã vạch
Tích hợp: Map API + Edge Detection + JetBot Control + QR Scanner
Version: Cải tiến nhận diện line và luồng chương trình
"""

import rospy
import cv2
import numpy as np
import time
import math
import requests
import json
from enum import Enum
from pyzbar.pyzbar import decode

# Import các module đã tạo
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from map_api import get_and_process_map, PathFinder, MapAPIHandler
from config import MAP_TYPES, API_CONFIG
from edge_detection import EdgeDetector
from jetbot_control import JetBotController, RobotState

from jetbot import Robot
from sensor_msgs.msg import Image, LaserScan

class Direction(Enum):
    """Hướng đi"""
    NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

class ProblemAState(Enum):
    """Trạng thái cho Problem A"""
    INITIALIZING = 0        # Đang khởi tạo
    CHECK_START_INTERSECTION = 1  # Kiểm tra giao lộ khởi đầu
    WAIT_USER_INPUT = 2     # Chờ người dùng nhập y/N
    AT_INTERSECTION = 3     # Đang ở giao lộ
    READING_QR = 4          # Đang đọc QR
    ROTATING_TO_PATH = 5    # Đang quay về hướng đi tiếp theo
    FOLLOWING_LINE = 6      # Đang bám line đi đến giao lộ tiếp theo
    REACHED_DESTINATION = 7 # Đã đến đích
    ERROR_STOPPED = 8       # Lỗi - dừng xe

class ProblemASolver:
    """
    Solver cho Problem A - điều khiển JetBot đi theo đường đã vạch
    """
    
    def __init__(self):
        """Khởi tạo Problem A Solver"""
        rospy.loginfo(" KHỞI TẠO PROBLEM A SOLVER...")
        rospy.loginfo("="*60)
        
        # Khởi tạo các components
        self.setup_components()
        self.setup_parameters()
        
        # Trạng thái
        self.current_state = ProblemAState.INITIALIZING
        self.current_direction = Direction.EAST  # Ban đầu quay đầu về Đông
        self.current_angle = 90  # Góc hiện tại (ĐÔNG = 90°)
        
        # Navigation data
        self.processed_map = None
        self.planned_path = None
        self.current_node_index = 0  
        self.current_node_id = None
        self.target_node_id = None
        
        # LiDAR control
        self.lidar_enabled = False  
        self.left_intersection_time = None  
        self.lidar_delay = 1.5  # Delay 1s sau khi rời giao lộ theo yêu cầu
        
        # Initial check flags
        self.initial_intersection_checked = False
        self.user_confirmed = False
        
        # Backup tracking
        self.backup_attempts = 0
        self.max_backup_attempts = 3
        
        # *** THÊM: Tracking nodes đã gửi API để tránh trùng lặp ***
        self.submitted_nodes = set()  # Set các node đã gửi API
        
        # *** THÊM: Line lost handling ***
        self.line_lost_time = None  # Thời điểm bắt đầu mất line
        self.line_lost_continue_duration = 2.0  # Đi thẳng thêm 2 giây khi hết line
        self.is_going_straight_after_line_lost = False  # Flag đang đi thẳng sau khi hết line
        
        rospy.loginfo(" PROBLEM A SOLVER ĐÃ SẴN SÀNG!")
        rospy.loginfo("="*60)

    def setup_components(self):
        """Khởi tạo các components"""
        try:
            # Edge detector
            self.edge_detector = EdgeDetector()
            
            # JetBot controller với tham số nhận diện line nhạy hơn
            self.jetbot_controller = JetBotController()
            
            # Điều chỉnh tham số để nhận diện line nhạy hơn
            self.adjust_line_detection_sensitivity()
            
            # Robot hardware
            self.robot = Robot()
            
            # ROS subscribers
            rospy.Subscriber('/csi_cam_0/image_raw', Image, self.jetbot_controller.camera_callback)
            rospy.Subscriber('/scan', LaserScan, self.edge_detector.callback)
            
            rospy.loginfo(" Đã khởi tạo các components")
        except Exception as e:
            rospy.logerr(f" Lỗi khởi tạo components: {e}")
            raise

    def adjust_line_detection_sensitivity(self):
        """Điều chỉnh tham số để nhận diện line nhạy hơn và tập trung xuống phía dưới camera"""
        # Giảm ngưỡng pixel để dễ phát hiện line hơn  
        self.jetbot_controller.SCAN_PIXEL_THRESHOLD = 35  # Giảm từ 50 xuống 35 để nhạy hơn
        
        # *** ĐIỀU CHỈNH ROI XUỐNG PHÍA DƯỚI CAMERA ***
        # Thay vì dò ở 75-100% (xa), chuyển xuống dò ở 88-100% (gần chân robot)
        self.jetbot_controller.ROI_Y = int(self.jetbot_controller.HEIGHT * 0.88)  # Từ 0.75 lên 0.88
        self.jetbot_controller.ROI_H = int(self.jetbot_controller.HEIGHT * 0.12)  # Từ 0.25 xuống 0.12
        
        # Tăng độ rộng vùng trung tâm để tìm line dễ hơn
        self.jetbot_controller.ROI_CENTER_WIDTH_PERCENT = 0.8  # Từ 0.7 lên 0.8
        
        # Điều chỉnh màu sắc để phát hiện line tốt hơn (phạm vi rộng hơn)
        self.jetbot_controller.LINE_COLOR_LOWER = np.array([0, 0, 0])    # Đen hoàn toàn
        self.jetbot_controller.LINE_COLOR_UPPER = np.array([180, 255, 120])  # Tăng từ 100 lên 120
        
        # Giảm vùng an toàn để phản ứng nhanh hơn với line
        self.jetbot_controller.SAFE_ZONE_PERCENT = 0.15  # Từ 0.2 xuống 0.15
        
        # Tăng gain điều chỉnh để bám line mạnh hơn
        self.jetbot_controller.CORRECTION_GAIN = 0.7  # Từ 0.5 lên 0.65
        
        # Giảm tốc độ để ổn định hơn khi bám line
        self.jetbot_controller.BASE_SPEED = 0.19  # Từ 0.14 xuống 0.12
        
        # *** BỔ SUNG: THAM SỐ CHỐNG MISS LINE ***
        # Tăng số lần scan để tìm line kỹ hơn
        if hasattr(self.jetbot_controller, 'MAX_SCAN_ATTEMPTS'):
            self.jetbot_controller.MAX_SCAN_ATTEMPTS = 5  # Thêm tham số này nếu có
        
        # Giảm tolerance để bám line chặt hơn  
        if hasattr(self.jetbot_controller, 'LINE_CENTER_TOLERANCE'):
            self.jetbot_controller.LINE_CENTER_TOLERANCE = 15  # pixels
        
        rospy.loginfo(" Đã cải thiện tham số nhận diện line:")
        rospy.loginfo(f"   - Ngưỡng pixel: 35 (nhạy hơn, ít miss)")
        rospy.loginfo(f"   - Vùng ROI: 88-100% (TẬP TRUNG PHÍA DƯỚI CAMERA)")
        rospy.loginfo(f"   - Vùng trung tâm: 80% (rộng hơn, khó miss)")
        rospy.loginfo(f"   - Gain điều chỉnh: 0.65 (bám chặt hơn)")  
        rospy.loginfo(f"   - Tốc độ: 0.12 (chậm hơn, ổn định hơn)")
        rospy.loginfo(" Cải thiện chống miss line và tập trung xuống chân camera!")

    def setup_parameters(self):
        """Thiết lập tham số"""
        # Direction mapping
        self.DIRECTIONS = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        self.LABEL_TO_DIRECTION = {'N': Direction.NORTH, 'E': Direction.EAST, 'S': Direction.SOUTH, 'W': Direction.WEST}
        self.DIRECTION_TO_LABEL = {v: k for k, v in self.LABEL_TO_DIRECTION.items()}
        
        # Angle mapping
        self.DIRECTION_TO_ANGLE = {
            Direction.NORTH: 0,     # Bắc
            Direction.EAST: 90,     # Đông  
            Direction.SOUTH: 180,   # Nam
            Direction.WEST: 270     # Tây
        }
        
        # Turn parameters
        self.TURN_DURATION_90_DEG = 0.8
        self.TURN_SPEED = 0.2

    def load_map_and_plan_path(self):
        """Lấy bản đồ từ API và lập kế hoạch đường đi"""
        rospy.loginfo("="*60)
        rospy.loginfo(" BƯỚC 1: LẤY BẢN ĐỒ VÀ TÌM ĐƯỜNG")
        rospy.loginfo("="*60)
        
        # Lấy bản đồ
        rospy.loginfo(" Đang kết nối API lấy bản đồ...")
        # PROBLEM_A, PROBLEM_B, SAMPLE
        map_type = MAP_TYPES["SAMPLE"]  
        self.processed_map, path_finder = get_and_process_map(map_type)
        
        if not self.processed_map or not path_finder:
            rospy.logerr(" LẤY BẢN ĐỒ THẤT BẠI!")
            return False
        
        # Log thông tin bản đồ
        map_info = self.processed_map.get('map_info', {})
        rospy.loginfo(" LẤY BẢN ĐỒ THÀNH CÔNG!")
        rospy.loginfo(f"    Tên bản đồ: {map_info.get('name', 'Unknown')}")
        rospy.loginfo(f"    Kích thước: {map_info.get('dimensions', {}).get('width')}x{map_info.get('dimensions', {}).get('height')}")
        rospy.loginfo(f"    Start node: {self.get_node_name(map_info.get('start_node'))} (ID: {map_info.get('start_node')})")
        rospy.loginfo(f"    End nodes: {[self.get_node_name(n) for n in map_info.get('end_nodes', [])]}")
        
        # Tìm đường đi ngắn nhất
        rospy.loginfo(" Đang tìm đường đi ngắn nhất...")
        self.planned_path = path_finder.find_shortest_path()
        
        if not self.planned_path:
            rospy.logerr(" KHÔNG TÌM THẤY ĐƯỜNG ĐI!")
            return False
        
        # Thiết lập navigation
        self.current_node_index = 0
        self.current_node_id = self.planned_path[0]
        self.target_node_id = self.planned_path[1] if len(self.planned_path) > 1 else None
        
        # In đường đi chi tiết
        rospy.loginfo(" TÌM ĐƯỜNG THÀNH CÔNG!")
        rospy.loginfo(f"    Độ dài đường đi: {len(self.planned_path)} nodes")
        rospy.loginfo("="*60)
        rospy.loginfo(" ĐƯỜNG ĐI NGẮN NHẤT (với hướng di chuyển):")
        
        directions_list = []
        for i in range(len(self.planned_path)):
            node_id = self.planned_path[i]
            node_name = self.get_node_name(node_id)
            
            # Lấy thông tin node
            node_info = None
            for node in self.processed_map['nodes']:
                if node['id'] == node_id:
                    node_info = node
                    break
            
            if i < len(self.planned_path) - 1:
                next_node_id = self.planned_path[i + 1]
                direction_label = "?"
                
                # Tìm edge và hướng
                for edge in self.processed_map['edges']:
                    if edge['source'] == node_id and edge['target'] == next_node_id:
                        direction_label = edge['label']
                        directions_list.append(direction_label)
                        break
                
                # Map hướng sang tiếng Việt
                direction_vn = {"N": "Bắc", "E": "Đông", "S": "Nam", "W": "Tây"}.get(direction_label, direction_label)
                
                rospy.loginfo(f"   Bước {i+1}: {node_name}({node_id}) "
                            f"→ [{direction_label}({direction_vn})] → "
                            f"{self.get_node_name(next_node_id)}({next_node_id})")
            else:
                node_type = node_info['type'] if node_info else ""
                rospy.loginfo(f"   Bước {i+1}: {node_name}({node_id}) - ĐÍCH [{node_type}]")
        
        # Thống kê hướng đi
        if directions_list:
            rospy.loginfo(" THỐNG KÊ HƯỚNG:")
            for direction in ['N', 'E', 'S', 'W']:
                count = directions_list.count(direction)
                if count > 0:
                    direction_vn = {"N": "Bắc", "E": "Đông", "S": "Nam", "W": "Tây"}[direction]
                    rospy.loginfo(f"   {direction}({direction_vn}): {count} lần")
        
        rospy.loginfo("="*60)
        return True

    def get_node_name(self, node_id):
        """Lấy tên node từ ID"""
        if not self.processed_map or not node_id:
            return "?"
        for node in self.processed_map['nodes']:
            if node['id'] == node_id:
                return node['name']
        return f"Node_{node_id}"

    def ensure_lidar_active(self):
        """Đảm bảo LiDAR được bật khi cần thiết"""
        # Nếu đang đi đến node tiếp theo mà LiDAR chưa bật
        if (self.current_state == ProblemAState.FOLLOWING_LINE and 
            not self.lidar_enabled and 
            self.left_intersection_time and
            rospy.get_time() - self.left_intersection_time > self.lidar_delay):
            
            rospy.logwarn(" FORCE BẬT LIDAR - Phát hiện LiDAR chưa được bật!")
            self.edge_detector.start_scanning()
            self.lidar_enabled = True
            self.left_intersection_time = None
    
    def log_lidar_status(self):
        """Log trạng thái LiDAR chi tiết"""
        if self.lidar_enabled:
            detection_info = self.edge_detector.get_detection_info()
            if detection_info:
                total_objects = detection_info['total_objects']
                opposite_pairs = detection_info['opposite_pairs']
                
                # Log chi tiết như sample_code
                rospy.loginfo_throttle(3, 
                    f"📡 LiDAR ACTIVE: Quét #{self.edge_detector.scan_count} | "
                    f"Vật thể: {total_objects} | Cặp đối: {opposite_pairs} | "
                    f"Tổng phát hiện: {self.edge_detector.detection_count}")
        else:
            # Log khi LiDAR tắt và thời gian còn lại
            if self.left_intersection_time:
                time_since_left = rospy.get_time() - self.left_intersection_time
                time_remaining = self.lidar_delay - time_since_left
                if time_remaining > 0:
                    rospy.loginfo_throttle(2, f" LiDAR: TẮT - Còn {time_remaining:.1f}s để bật")
                else:
                    rospy.logwarn_throttle(1, " LiDAR: SHOULD BE ON - Kiểm tra logic!")
            else:
                rospy.logdebug(" LiDAR: TẮT")

    def check_initial_intersection(self):
        """Kiểm tra xem xe đã ở giao lộ khởi đầu chưa"""
        rospy.loginfo("="*60)
        rospy.loginfo(" BƯỚC 2: KIỂM TRA GIAO LỘ KHỞI ĐẦU")
        rospy.loginfo("="*60)
        
        rospy.loginfo(" Bật LiDAR kiểm tra giao lộ...")
        self.edge_detector.start_scanning()
        time.sleep(1.0)  # Đợi LiDAR ổn định
        
        max_attempts = 10
        for attempt in range(1, max_attempts + 1):
            rospy.loginfo(f" Đang quét LiDAR... (lần {attempt}/{max_attempts})")
            
            if self.edge_detector.detect_edge():
                rospy.loginfo(" LiDAR: XÁC NHẬN XE ĐANG Ở GIAO LỘ!")
                rospy.loginfo(" Tắt LiDAR tạm thời")
                self.edge_detector.stop_scanning()
                self.lidar_enabled = False
                return True
            
            time.sleep(0.5)
        
        rospy.logwarn(" LiDAR: KHÔNG PHÁT HIỆN ĐƯỢC GIAO LỘ KHỞI ĐẦU")
        rospy.loginfo(" Tắt LiDAR, tiếp tục chương trình")
        self.edge_detector.stop_scanning()
        self.lidar_enabled = False
        return False

    def wait_user_confirmation(self):
        """Chờ xác nhận từ người dùng"""
        rospy.loginfo("="*60)
        rospy.loginfo(" BƯỚC 3: CHỜ XÁC NHẬN TỪ NGƯỜI DÙNG")
        rospy.loginfo("="*60)
        rospy.loginfo(" Vị trí hiện tại: %s", self.get_node_name(self.current_node_id))
        rospy.loginfo(" Hướng hiện tại: %s (%d°)", 
                     self.DIRECTION_TO_LABEL[self.current_direction], 
                     self.current_angle)
        rospy.loginfo("="*60)
        
        while True:
            try:
                user_input = input("\n Bắt đầu navigation? (y/N): ").strip().lower()
                if user_input in ['y', 'yes']:
                    rospy.loginfo(" NGƯỜI DÙNG XÁC NHẬN: BẮT ĐẦU!")
                    return True
                elif user_input in ['n', 'no', '']:
                    rospy.loginfo(" NGƯỜI DÙNG HỦY CHƯƠNG TRÌNH!")
                    return False
                else:
                    print(" Vui lòng nhập 'y' để bắt đầu hoặc 'n' để hủy")
            except KeyboardInterrupt:
                rospy.loginfo("\n Keyboard interrupt - dừng chương trình")
                return False

    def handle_line_lost_backup(self):
        """Xử lý backup khi hết line - quay NE kiểm tra QR"""
        rospy.logwarn(" BACKUP: Hết line, kiểm tra QR tại góc NE")
        
        # Lưu hướng hiện tại
        original_angle = self.current_angle
        original_direction = self.current_direction
        
        # Quay sang NE (45°)
        rospy.logwarn(f" Lưu hướng gốc: {self.DIRECTION_TO_LABEL[original_direction]} ({original_angle}°)")
        rospy.logwarn(" Quay sang NE (45°) kiểm tra QR backup")
        
        ne_angle = 45
        angle_diff = ne_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        self.rotate_robot(angle_diff)
        time.sleep(1.0)
        
        # Kiểm tra QR
        rospy.logwarn(" Đang kiểm tra QR backup...")
        qr_attempts = 15
        qr_found = None
        
        for attempt in range(qr_attempts):
            qr_result = self.scan_qr_code()
            if qr_result:
                qr_found = qr_result
                rospy.logwarn(f" TÌM THẤY QR BACKUP: '{qr_result}'")
                break
            time.sleep(0.3)
        
        if qr_found:
            # Có QR - xử lý như giao lộ bình thường
            rospy.logwarn(f" XÁC NHẬN BACKUP TẠI NODE: {qr_found}")
            rospy.logwarn(" BACKUP QR THÀNH CÔNG - CHUẨN BỊ TIẾP TỤC NAVIGATION")
            
            # Gửi dữ liệu QR backup lên server
            rospy.logwarn(" Gửi QR backup data lên server...")
            api_success = self.submit_to_server(
                text=qr_found,
                node_id=self.current_node_id,
                map_type=self.processed_map['map_info']['mapType']
            )
            if api_success:
                rospy.logwarn(" Đã gửi QR backup data lên server thành công!")
            else:
                rospy.logwarn(" Không thể gửi QR backup data lên server, tiếp tục chương trình...")
            
            return qr_found
        else:
            # Không có QR - quay về hướng cũ và dừng
            rospy.logwarn(" Không tìm thấy QR backup - Quay về hướng cũ")
            
            # Quay về hướng cũ
            angle_back = original_angle - self.current_angle
            while angle_back > 180:
                angle_back -= 360
            while angle_back < -180:
                angle_back += 360
            
            if abs(angle_back) > 5:
                self.rotate_robot(angle_back)
            
            rospy.logerr(" KHÔNG CÓ QR BACKUP - DỪNG ROBOT!")
            return None

    def try_line_recovery(self):
        """Thử khôi phục line khi tạm mất - quay nhẹ trái/phải tìm lại"""
        rospy.logwarn(" BẮT ĐẦU LINE RECOVERY...")
        
        # Lưu trạng thái ban đầu
        original_angle = self.current_angle
        recovery_attempts = [
            ("TRÁI", -8),    # Quay trái 8°
            ("PHẢI", 16),    # Quay phải 16° (8° + 8° bù trừ)
            ("TRÁI", -24),   # Quay trái 24° (16° + 8° thêm) 
            ("PHẢI", 16),    # Quay phải về gần vị trí ban đầu
        ]
        
        for attempt, (direction, angle) in enumerate(recovery_attempts):
            rospy.logwarn(f" Recovery {attempt+1}/4: Quay {direction} {abs(angle)}°")
            
            # Quay nhẹ
            if abs(angle) >= 5:  # Chỉ quay nếu góc đủ lớn
                if angle > 0:  # Quay trái
                    self.robot.set_motors(0.15, -0.15)
                else:  # Quay phải  
                    self.robot.set_motors(-0.15, 0.15)
                
                duration = abs(angle) / 90.0 * self.TURN_DURATION_90_DEG
                time.sleep(duration)
                self.robot.stop()
                time.sleep(0.3)
                
                # Cập nhật angle ước tính
                self.current_angle += angle
                while self.current_angle < 0:
                    self.current_angle += 360
                while self.current_angle >= 360:
                    self.current_angle -= 360
            
            # Thử tìm line ở vị trí mới
            rospy.logwarn(f" Tìm line ở góc {self.current_angle:.1f}°...")
            for scan in range(3):  # Thử 3 lần ở mỗi góc
                line_result = self.jetbot_controller.follow_line_continuous()
                if line_result['status'] == 'following_line':
                    rospy.logwarn(f" TÌM LẠI LINE THÀNH CÔNG ở lần thử {attempt+1}!")
                    return True
                time.sleep(0.2)
        
        # Không tìm được - quay về góc ban đầu  
        rospy.logwarn(" Line recovery thất bại - quay về góc ban đầu")
        angle_back = original_angle - self.current_angle
        while angle_back > 180:
            angle_back -= 360  
        while angle_back < -180:
            angle_back += 360
            
        if abs(angle_back) >= 5:
            self.rotate_robot(angle_back)
            
        return False

    def align_line_center_improved(self):
        """Căn chỉnh line về giữa xe - đi thẳng giữ vị trí"""
        rospy.loginfo(" Căn chỉnh line - đi thẳng giữ vị trí...")
        # Sử dụng hàm align_to_line_center từ jetbot_controller
        return self.jetbot_controller.align_to_line_center(max_attempts=8)
        rospy.loginfo(" Bắt đầu căn chỉnh line về giữa...")
        
        max_align_attempts = 8  # Tối đa 8 lần thử
        image_center = self.jetbot_controller.WIDTH // 2
        tolerance = 20  # pixels - cho phép sai lệch 20px
        
        for attempt in range(max_align_attempts):
            # Lấy thông tin line hiện tại
            line_result = self.jetbot_controller.follow_line_continuous()
            
            if line_result['status'] != 'following_line':
                rospy.logwarn(f" Căn chỉnh {attempt+1}: Không thấy line")
                time.sleep(0.2)
                continue
                
            line_pos = line_result.get('line_position', image_center)
            deviation = line_pos - image_center
            
            rospy.loginfo(f" Căn chỉnh {attempt+1}: Line ở {line_pos}px, lệch {deviation}px")
            
            # Nếu đã đủ chính xác
            if abs(deviation) <= tolerance:
                rospy.loginfo(f" Căn chỉnh thành công! Lệch chỉ {abs(deviation)}px")
                break
                
            # Điều chỉnh nhẹ theo độ lệch
            adjust_duration = min(abs(deviation) / 100.0 * 0.3, 0.2)  # Tối đa 0.2s
            adjust_speed = 0.1
            
            if deviation > 0:  # Line lệch phải, quay trái
                rospy.loginfo(f"   ↶ Line lệch phải {deviation}px, quay trái {adjust_duration:.2f}s")
                self.robot.set_motors(adjust_speed, -adjust_speed)
            else:  # Line lệch trái, quay phải  
                rospy.loginfo(f"   ↷ Line lệch trái {abs(deviation)}px, quay phải {adjust_duration:.2f}s")
                self.robot.set_motors(-adjust_speed, adjust_speed)
                
            time.sleep(adjust_duration)
            self.robot.stop()
            time.sleep(0.3)  # Đợi ổn định
            
        rospy.loginfo(" Hoàn thành căn chỉnh line!")

    def handle_lidar_false_positive(self, original_direction):
        """Xử lý trường hợp LiDAR nhầm - quay về hướng cũ"""
        rospy.logwarn(" LiDAR FALSE POSITIVE: Không có QR, quay về hướng cũ")
        
        # Tính góc quay về
        target_angle = self.DIRECTION_TO_ANGLE[original_direction]
        angle_diff = target_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        if abs(angle_diff) > 5:
            rospy.logwarn(f" Quay về hướng cũ: {self.DIRECTION_TO_LABEL[original_direction]} ({target_angle}°)")
            self.rotate_robot(angle_diff)
        
        # Tiếp tục đi theo hướng cũ
        rospy.logwarn(" Tiếp tục bám line theo hướng ban đầu")
        return True

    def scan_qr_code(self):
        """Quét mã QR"""
        if not hasattr(self.jetbot_controller, 'latest_image') or self.jetbot_controller.latest_image is None:
            return None
            
        try:
            from pyzbar.pyzbar import decode
            decoded_objects = decode(self.jetbot_controller.latest_image)
            
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8').strip()
                return qr_data
            return None
                
        except Exception as e:
            rospy.logerr(f"❌ Lỗi quét QR: {e}")
            return None

    def rotate_robot(self, angle_degrees):
        """Quay robot một góc nhất định"""
        if abs(angle_degrees) < 5:  
            rospy.loginfo(f"↻ Góc quay {angle_degrees}° quá nhỏ, bỏ qua")
            return
        
        rospy.loginfo(f" QUAY ROBOT {angle_degrees}°")
        
        duration = abs(angle_degrees) / 90.0 * self.TURN_DURATION_90_DEG
        
        if angle_degrees > 0:  # Quay trái
            rospy.loginfo(f"   ↶ Quay TRÁI {abs(angle_degrees)}°")
            self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)
        else:  # Quay phải
            rospy.loginfo(f"   ↷ Quay PHẢI {abs(angle_degrees)}°")
            self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)
        
        time.sleep(duration)
        self.robot.stop()
        time.sleep(0.5)
        
        # Cập nhật góc và hướng
        self.current_angle += angle_degrees
        while self.current_angle < 0:
            self.current_angle += 360
        while self.current_angle >= 360:
            self.current_angle -= 360
        
        # Cập nhật hướng
        old_direction = self.current_direction
        min_diff = float('inf')
        for direction, angle in self.DIRECTION_TO_ANGLE.items():
            diff = abs(self.current_angle - angle)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                self.current_direction = direction
        
        rospy.loginfo(f"    Đã quay xong: {self.DIRECTION_TO_LABEL[old_direction]} → {self.DIRECTION_TO_LABEL[self.current_direction]} ({self.current_angle}°)")

    def get_direction_to_target(self):
        """Lấy hướng đi đến target node"""
        if not self.processed_map or not self.current_node_id or not self.target_node_id:
            return None
            
        for edge in self.processed_map['edges']:
            if edge['source'] == self.current_node_id and edge['target'] == self.target_node_id:
                direction_label = edge['label']
                return self.LABEL_TO_DIRECTION.get(direction_label)
        
        return None

    def handle_start_position(self):
        """Xử lý vị trí khởi đầu: Quay sang NE đọc QR"""
        rospy.loginfo("="*60)
        rospy.loginfo(" BƯỚC 4: ĐỌC QR XÁC NHẬN VỊ TRÍ START")
        rospy.loginfo("="*60)
        
        # Quay sang NE (45°) để đọc QR
        rospy.loginfo(f" Hướng hiện tại: {self.DIRECTION_TO_LABEL[self.current_direction]} ({self.current_angle}°)")
        rospy.loginfo(" Hướng sắp quay: NE (45°) để đọc QR")
        
        ne_angle = 45
        angle_diff = ne_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        self.rotate_robot(angle_diff)
        time.sleep(1.0)
        
        # Đọc QR
        rospy.loginfo(" Đang quét mã QR...")
        max_attempts = 30  # 30 * 0.5s = 15s
        
        for attempt in range(max_attempts):
            qr_result = self.scan_qr_code()
            
            if qr_result:
                expected_name = self.get_node_name(self.current_node_id)
                
                rospy.loginfo(f" QR đọc được: '{qr_result}'")
                rospy.loginfo(f" Node mong đợi: '{expected_name}'")
                
                if qr_result == expected_name:
                    rospy.loginfo(" QR KHỚP VỚI BẢN ĐỒ!")
                    
                    # Gửi dữ liệu QR lên server
                    rospy.loginfo(" Gửi QR data lên server...")
                    api_success = self.submit_to_server(
                        text=qr_result,
                        node_id=self.current_node_id,
                        map_type=self.processed_map['map_info']['mapType']
                    )
                    if api_success:
                        rospy.loginfo(" Đã gửi QR data lên server thành công!")
                    else:
                        rospy.logwarn(" Không thể gửi QR data lên server, tiếp tục chương trình...")
                
                    return True
                else:
                    rospy.logerr(f" QR KHÔNG KHỚP! (Đọc: '{qr_result}' ≠ Mong đợi: '{expected_name}')")
                    return False
            
            if attempt % 5 == 0:
                rospy.loginfo(f"   ... đang quét QR (lần {attempt+1}/{max_attempts})")
            
            time.sleep(0.5)
        
        rospy.logerr(" Không thể đọc QR sau 15 giây!")
        return False

    def navigate_to_next_node(self):
        """Điều hướng đến node tiếp theo"""
        if self.target_node_id is None:
            rospy.loginfo(" Đã đến node cuối cùng!")
            return False
        
        # Lấy hướng cần đi
        target_direction = self.get_direction_to_target()
        if not target_direction:
            rospy.logerr(" Không tìm thấy hướng đi đến node tiếp theo!")
            return False
        
        rospy.loginfo("="*60)
        rospy.loginfo(" BẮT ĐẦU DI CHUYỂN")
        rospy.loginfo("="*60)
        rospy.loginfo(f" Từ: {self.get_node_name(self.current_node_id)}")
        rospy.loginfo(f" Đến: {self.get_node_name(self.target_node_id)}")
        rospy.loginfo(f" Hướng hiện tại: {self.DIRECTION_TO_LABEL[self.current_direction]} ({self.current_angle}°)")
        rospy.loginfo(f" Hướng cần đi: {self.DIRECTION_TO_LABEL[target_direction]}")
        
        # Quay về hướng cần đi
        target_angle = self.DIRECTION_TO_ANGLE[target_direction]
        angle_diff = target_angle - self.current_angle
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        if abs(angle_diff) > 5:
            self.rotate_robot(angle_diff)
            # *** LOẠI BỎ: Không còn căn chỉnh line - CHỈ ĐI THẲNG ***
            rospy.loginfo(" BỎ QUA CĂNG CHỈNH LINE - CHỈ ĐI THẲNG KHÔNG RẼ")
        
        rospy.loginfo(" LĂN BÁNH!")
        
        # Ghi nhận thời điểm rời giao lộ và BẬT LIDAR NGAY SAU DELAY NGẮN
        self.left_intersection_time = rospy.get_time()
        self.lidar_enabled = False
        
        rospy.logwarn(f" RỜI GIAO LỘ: {self.get_node_name(self.current_node_id)}")
        rospy.logwarn(f" HƯỚNG ĐẾN: {self.get_node_name(self.target_node_id)}")
        rospy.logwarn(" LiDAR sẽ bật sau ĐÚNG 1 GIÂY để tìm giao lộ tiếp theo")
        rospy.logwarn(" XE ĐI THẲNG GIỮ VỊ TRÍ - KHÔNG RẺ THEO LINE CONG!")
        
        return True

    def run_navigation(self):
        """Vòng lặp navigation chính"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🚦 BẮT ĐẦU CHƯƠNG TRÌNH NAVIGATION")
        rospy.loginfo("="*60)
        
        # 1. Lấy bản đồ và tìm đường
        if not self.load_map_and_plan_path():
            return False
        
        # 2. Kiểm tra giao lộ khởi đầu
        self.initial_intersection_checked = self.check_initial_intersection()
        
        # 3. Chờ xác nhận người dùng
        if not self.wait_user_confirmation():
            self.cleanup()
            return False
        
        # 4. Xử lý vị trí START
        if not self.handle_start_position():
            rospy.logerr(" Không thể xác nhận vị trí START!")
            self.cleanup()
            return False
        
        # 5. Bắt đầu di chuyển
        if not self.navigate_to_next_node():
            rospy.logerr(" Không thể bắt đầu di chuyển!")
            self.cleanup()
            return False
        
        self.current_state = ProblemAState.FOLLOWING_LINE
        
        # Vòng lặp chính - LOGIC ĐƠN GIẢN
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            try:
                if self.current_state == ProblemAState.FOLLOWING_LINE:
                    # Quản lý LiDAR - bật ngay sau khi rời giao lộ (delay ngắn tránh phát hiện lại)
                    if not self.lidar_enabled and self.left_intersection_time:
                        time_since_left = rospy.get_time() - self.left_intersection_time
                        if time_since_left >= self.lidar_delay:
                            self.edge_detector.start_scanning()
                            self.lidar_enabled = True
                            rospy.logwarn(" BẬT LIDAR - LIÊN TỤC TÌM GIAO LỘ TIẾP THEO")
                            rospy.logwarn(f"    Đã chờ ĐÚNG {time_since_left:.1f}s sau khi rời giao lộ")
                            rospy.logwarn("    LIDAR ĐANG TÌM KIẾM CỘT NE-SW TẠI GIAO LỘ")
                            self.left_intersection_time = None
                    
                    # Kiểm tra giao lộ bằng LiDAR TRƯỚC KHI bám line
                    intersection_detected = False
                    if self.lidar_enabled and self.edge_detector.detect_edge():
                        rospy.logwarn(" LIDAR: PHÁT HIỆN GIAO LỘ!")
                        rospy.logwarn(" DỪNG ROBOT VÀ XỬ LÝ GIAO LỘ")\

                        self.jetbot_controller.stop_robot()
                        self.edge_detector.stop_scanning()
                        self.lidar_enabled = False
                        time.sleep(1.0)
                        intersection_detected = True
                    
                    if intersection_detected:
                        
                        # Cập nhật vị trí
                        self.current_node_index += 1
                        self.current_node_id = self.target_node_id
                        
                        rospy.logwarn(f" ĐÃ ĐẾN GIAO LỘ: {self.get_node_name(self.current_node_id)}")
                        rospy.logwarn(f" Trạng thái: ĐANG Ở GIAO LỘ")
                        
                        # Kiểm tra đã đến đích chưa
                        map_info = self.processed_map['map_info']
                        if self.current_node_id in map_info.get('end_nodes', []):
                            rospy.logwarn(" ĐÃ ĐẾN ĐÍCH!")
                            
                            # Gửi thông báo đến đích lên server (không cần QR)
                            rospy.logwarn(" Gửi thông báo đến đích lên server...")
                            destination_text = f"{self.get_node_name(self.current_node_id)}"
                            api_success = self.submit_to_server(
                                text=destination_text,
                                node_id=self.current_node_id,
                                map_type=self.processed_map['map_info']['mapType']
                            )
                            if api_success:
                                rospy.logwarn(" Đã gửi thông báo đến đích lên server thành công!")
                            else:
                                rospy.logwarn(" Không thể gửi thông báo đến đích lên server, tiếp tục chương trình...")
                            
                            self.current_state = ProblemAState.REACHED_DESTINATION
                            continue
                        
                        # Quay sang NE đọc QR
                        rospy.logwarn(" BƯỚC 1: Quay sang NE (45°) để quét QR xác nhận")
                        ne_angle = 45
                        angle_diff = ne_angle - self.current_angle
                        while angle_diff > 180:
                            angle_diff -= 360
                        while angle_diff < -180:
                            angle_diff += 360
                        self.rotate_robot(angle_diff)
                        
                        # Đọc QR xác nhận
                        rospy.logwarn(" BƯỚC 2: Đang đọc QR xác nhận tên giao lộ...")
                        qr_attempts = 20
                        qr_success = False
                        
                        for i in range(qr_attempts):
                            qr_result = self.scan_qr_code()
                            if qr_result:
                                expected = self.get_node_name(self.current_node_id)
                                rospy.logwarn(f" QR đọc được: '{qr_result}'")
                                rospy.logwarn(f" Giao lộ mong đợi: '{expected}'")
                                
                                if qr_result == expected:
                                    rospy.logwarn(f" QR XÁC NHẬN ĐÚNG: {qr_result}")
                                    
                                    # Gửi dữ liệu QR lên server
                                    rospy.logwarn(" Gửi QR data lên server...")
                                    api_success = self.submit_to_server(
                                        text=qr_result,
                                        node_id=self.current_node_id,
                                        map_type=self.processed_map['map_info']['mapType']
                                    )
                                    if api_success:
                                        rospy.logwarn(" Đã gửi QR data lên server thành công!")
                                    else:
                                        rospy.logwarn(" Không thể gửi QR data lên server, tiếp tục chương trình...")
                                    
                                    qr_success = True
                                    break
                                else:
                                    rospy.logerr(f" QR SAI: {qr_result} ≠ {expected}")
                                    break
                            
                            if i % 5 == 0:
                                rospy.loginfo(f"   ... đang quét QR ({i+1}/{qr_attempts})")
                            time.sleep(0.5)
                        
                        if not qr_success:
                            rospy.logerr(" KHÔNG TÌM THẤY QR - LIDAR FALSE POSITIVE!")
                            
                            # Quay về hướng cũ và tiếp tục đi
                            if self.handle_lidar_false_positive(self.current_direction):
                                rospy.logwarn(" Tiếp tục bám line theo hướng cũ")
                                # Reset LiDAR delay để bật lại sau
                                self.left_intersection_time = rospy.get_time()
                                self.lidar_enabled = False
                                continue
                            else:
                                rospy.logerr(" Không thể khôi phục - dừng!")
                                self.current_state = ProblemAState.ERROR_STOPPED
                                continue
                        
                        # Chuẩn bị đi tiếp
                        if self.current_node_index < len(self.planned_path) - 1:
                            self.target_node_id = self.planned_path[self.current_node_index + 1]
                            rospy.logwarn(f" BƯỚC 3: Chuẩn bị đi đến node tiếp theo: {self.get_node_name(self.target_node_id)}")
                            rospy.logwarn(f" Trạng thái: RỜI GIAO LỘ - LIDAR SẼ BẬT SAU 0.5s")
                            
                            self.navigate_to_next_node()
                            self.current_state = ProblemAState.FOLLOWING_LINE
                            
                            # ĐẢM BẢO LIDAR ĐƯỢC BẬT
                            rospy.logwarn(" Đảm bảo LiDAR sẽ hoạt động để tìm giao lộ tiếp theo...")
                        else:
                            rospy.logwarn(" ĐÃ ĐẾN NODE CUỐI CÙNG!")
                            self.current_state = ProblemAState.REACHED_DESTINATION
                        
                        continue
                    
                    else:
                        # KHÔNG có giao lộ - tiếp tục bám line với cải thiện chống miss
                        line_result = self.jetbot_controller.follow_line_continuous()
                        
                        if line_result['status'] == 'following_line':
                            # *** RESET flag khi tìm lại line thành công ***
                            if self.is_going_straight_after_line_lost:
                                rospy.loginfo(" TÌM LẠI LINE - RESET line lost flags")
                                self.is_going_straight_after_line_lost = False
                                self.line_lost_time = None
                            
                            # Đang bám line thành công
                            lidar_status = "ON" if self.lidar_enabled else "WAIT"
                            
                            # *** CẢI TIẾN: Sử dụng bám đường nhẹ để giảm sai số ***
                            line_quality = "GOOD"
                            if 'line_position' in line_result:
                                line_pos = line_result['line_position']
                                image_center = self.jetbot_controller.WIDTH // 2
                                deviation = abs(line_pos - image_center)
                                
                                if deviation > 50:
                                    line_quality = "POOR"
                                elif deviation > 30:
                                    line_quality = "OK"
                            
                            rospy.loginfo_throttle(3, 
                                f" ĐI THẲNG GIỮ VỊ TRÍ - KHÔNG RẺ THEO LINE | "
                                f"Từ: {self.get_node_name(self.current_node_id)} | "
                                f"Đến: {self.get_node_name(self.target_node_id)} | "
                                f"Hướng: {self.DIRECTION_TO_LABEL[self.current_direction]} | "
                                f"LiDAR: {lidar_status} | "
                                f"Line: {line_quality} | "
                                f" BÁM ĐƯỜNG NHẸ!"
                            )
                            
                            # Log trạng thái LiDAR chi tiết
                            self.log_lidar_status()
                            
                            # Đảm bảo LiDAR luôn hoạt động khi cần
                            self.ensure_lidar_active()
                            
                        elif line_result['status'] == 'line_lost_temporary':
                            # *** CẢI THIỆN: Xử lý tạm mất line (chưa cần backup) ***
                            rospy.logwarn_throttle(1, " TẠM MẤT LINE - THỬ TÌM LẠI...")
                            
                            # Thử tìm lại line bằng cách quay nhẹ trái/phải
                            recovery_success = self.try_line_recovery()
                            if not recovery_success:
                                rospy.logwarn(" Không tìm lại được line - chuyển sang mode đi thẳng 2s")
                                line_result['status'] = 'line_lost'
                                line_result['need_backup'] = True
                            
                        elif line_result['status'] == 'line_lost' and line_result.get('need_backup'):
                            # *** SỬA ĐỔI: Hết line - đi thẳng 2 giây trước khi backup ***
                            if not self.is_going_straight_after_line_lost:
                                # Lần đầu phát hiện hết line - bắt đầu đi thẳng 2s
                                self.line_lost_time = rospy.get_time()
                                self.is_going_straight_after_line_lost = True
                                rospy.logwarn(" HẾT LINE - BẮT ĐẦU ĐI THẲNG THÊM 2 GIÂY!")
                                rospy.logwarn(f" Thời gian bắt đầu: {self.line_lost_time}")
                                
                            # Kiểm tra đã đi thẳng đủ 2 giây chưa
                            time_since_line_lost = rospy.get_time() - self.line_lost_time
                            
                            if time_since_line_lost < self.line_lost_continue_duration:
                                # Vẫn chưa đủ 2 giây - tiếp tục đi thẳng
                                remaining_time = self.line_lost_continue_duration - time_since_line_lost
                                rospy.logwarn_throttle(0.5, 
                                    f" ĐI THẲNG SAU KHI HẾT LINE | "
                                    f"Đã đi: {time_since_line_lost:.1f}s | "
                                    f"Còn lại: {remaining_time:.1f}s"
                                )
                                
                                # Đi thẳng với tốc độ chậm
                                self.robot.set_motors(0.1, 0.1)  # Đi thẳng chậm
                                continue
                            
                            else:
                                # Đã đi thẳng đủ 2 giây - chuyển sang backup mode
                                rospy.logwarn(" ĐÃ ĐI THẲNG ĐỦ 2 GIÂY - CHUYỂN SANG BACKUP!")
                                self.robot.stop()  # Dừng xe trước khi backup
                                
                                # Reset flag
                                self.is_going_straight_after_line_lost = False
                                self.line_lost_time = None
                                
                                # Tăng số lần backup
                                self.backup_attempts += 1
                                if self.backup_attempts > self.max_backup_attempts:
                                    rospy.logerr(f" QUÁ NHIỀU LẦN BACKUP ({self.backup_attempts}) - DỪNG!")
                                    self.current_state = ProblemAState.ERROR_STOPPED
                                    continue
                                
                                rospy.logwarn(f" BẮT ĐẦU BACKUP QR (lần {self.backup_attempts}/{self.max_backup_attempts})!")
                                backup_qr = self.handle_line_lost_backup()
                            
                                if backup_qr:
                                    try:
                                        # Tìm thấy QR backup - XỬ LÝ NHỮ GIAO LỘ THẬT
                                        rospy.logwarn(f" BACKUP THÀNH CÔNG: Tìm thấy node {backup_qr}")
                                        
                                        # *** RESET line lost flags khi backup thành công ***
                                        self.is_going_straight_after_line_lost = False
                                        self.line_lost_time = None
                                                
                                        # Cập nhật vị trí hiện tại
                                        rospy.logwarn(f" CẬP NHẬT VỊ TRÍ: Đã đến {backup_qr}")
                                                
                                        # Tìm node ID từ tên
                                        backup_node_id = None
                                        for node in self.processed_map['nodes']:
                                            if node['name'] == backup_qr:
                                                backup_node_id = node['id']
                                                rospy.logwarn(f"📍 Tìm thấy node ID: {backup_node_id}")
                                                break
                                        
                                        if backup_node_id:
                                            # Cập nhật navigation
                                            self.current_node_id = backup_node_id
                                            rospy.logwarn(f" Cập nhật current_node_id = {self.current_node_id}")
                                            
                                            # Kiểm tra đã đến đích chưa
                                            map_info = self.processed_map['map_info']
                                            end_nodes = map_info.get('end_nodes', [])
                                            rospy.logwarn(f" End nodes: {end_nodes}")
                                            
                                            if self.current_node_id in end_nodes:
                                                rospy.logwarn(" BACKUP THÀNH CÔNG - ĐÃ ĐẾN ĐÍCH!")
                                                
                                                # Gửi thông báo đến đích qua backup lên server
                                                rospy.logwarn(" Gửi thông báo đến đích (backup) lên server...")
                                                destination_text = self.get_node_name(self.current_node_id)
                                                api_success = self.submit_to_server(
                                                    text=destination_text,
                                                    node_id=self.current_node_id,
                                                    map_type=self.processed_map['map_info']['mapType']
                                                )
                                                if api_success:
                                                    rospy.logwarn(" Đã gửi thông báo đến đích (backup) lên server thành công!")
                                                else:
                                                    rospy.logwarn(" Không thể gửi thông báo đến đích (backup) lên server, tiếp tục chương trình...")
                                                
                                                self.current_state = ProblemAState.REACHED_DESTINATION
                                                continue
                                            
                                            # Tìm node tiếp theo trong planned_path
                                            rospy.logwarn(f" Tìm vị trí trong planned_path: {self.planned_path}")
                                            found_next = False
                                            
                                            for i, node_id in enumerate(self.planned_path):
                                                if node_id == self.current_node_id and i < len(self.planned_path) - 1:
                                                    self.current_node_index = i
                                                    self.target_node_id = self.planned_path[i + 1]
                                                    
                                                    rospy.logwarn(f" TIẾP TỤC ĐẾN: {self.get_node_name(self.target_node_id)} (index: {i+1})")
                                                    
                                                    # Bắt đầu di chuyển đến node tiếp theo
                                                    if self.navigate_to_next_node():
                                                        rospy.logwarn(" Navigation setup thành công!")
                                                        self.current_state = ProblemAState.FOLLOWING_LINE
                                                        found_next = True
                                                    else:
                                                        rospy.logerr(" Không thể tiếp tục navigation!")
                                                        self.current_state = ProblemAState.ERROR_STOPPED
                                                    break
                                            
                                            if not found_next:
                                                rospy.logerr(f" Không tìm thấy node {backup_node_id} trong planned_path!")
                                                rospy.logerr(f"   Planned path: {self.planned_path}")
                                                self.current_state = ProblemAState.ERROR_STOPPED
                                        else:
                                            rospy.logerr(f" Không tìm thấy node ID cho '{backup_qr}'!")
                                            rospy.logerr("   Available nodes:")
                                            for node in self.processed_map['nodes'][:5]:  # Log 5 nodes đầu
                                                rospy.logerr(f"     {node['id']}: {node['name']}")
                                            self.current_state = ProblemAState.ERROR_STOPPED
                                
                                    except Exception as e:
                                        rospy.logerr(f" LỖI XỬ LÝ BACKUP: {e}")
                                        import traceback
                                        rospy.logerr(f"   Traceback: {traceback.format_exc()}")
                                        self.current_state = ProblemAState.ERROR_STOPPED
                                else:
                                    # Không tìm thấy QR backup - dừng hẳn
                                    rospy.logerr(" BACKUP THẤT BẠI - DỪNG CHƯƠNG TRÌNH!")
                                    self.current_state = ProblemAState.ERROR_STOPPED
                        else:
                            # Lỗi camera hoặc trường hợp khác
                            rospy.logwarn(" Vấn đề camera - tiếp tục thử...")
                            continue

                elif self.current_state == ProblemAState.REACHED_DESTINATION:
                    rospy.loginfo("="*60)
                    rospy.loginfo(" HOÀN THÀNH PROBLEM A THÀNH CÔNG!")
                    rospy.loginfo("="*60)
                    
                    # Gửi thông báo hoàn thành cuối cùng lên server
                    rospy.loginfo(" Gửi thông báo hoàn thành cuối cùng lên server...")
                 
                  
                    self.robot.stop()
                    break
                    
                elif self.current_state == ProblemAState.ERROR_STOPPED:
                    rospy.logerr(" Problem A dừng do lỗi!")
                    self.robot.stop()
                    break
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f" Lỗi trong vòng lặp: {e}")
                self.robot.stop()
                break
        
        self.cleanup()
        return self.current_state == ProblemAState.REACHED_DESTINATION

    def submit_to_server(self, text, node_id, map_type=None):
        """
        Gửi dữ liệu lên server theo API specification
        
        Args:
            text (str): Nội dung Symbol đã nhận diện (QR code)
            node_id (str): ID của node/vị trí phát hiện Symbol
            map_type (str): Map đang chạy (map_a, map_b, map_c, map_z)
        
        Returns:
            bool: True nếu gửi thành công, False nếu thất bại
        """
        try:
            # *** KIỂM TRA TRÙNG LẶP: Tránh gửi cùng node nhiều lần ***
            node_key = f"{node_id}_{text}"
            if node_key in self.submitted_nodes:
                rospy.logwarn(f" ĐÃ GỬI API CHO NODE {node_id} VỚI TEXT '{text}' - BỎ QUA!")
                return True  # Coi như thành công vì đã gửi rồi
            
            # Đánh dấu node này đã gửi
            self.submitted_nodes.add(node_key)
            rospy.loginfo(f" Đánh dấu node {node_id} với text '{text}' đã gửi API")
            # Xác định map_type nếu không được cung cấp
            if map_type is None:
                # Tìm map_type từ processed_map
                if self.processed_map and 'map_info' in self.processed_map:
                    map_name = self.processed_map['map_info'].get('name', '')
                    # Map từ tên sang map_type
                    if 'map_z' in map_name.lower():
                        map_type = 'map_z'
                    elif 'map_a' in map_name.lower():
                        map_type = 'map_a'
                    elif 'map_b' in map_name.lower():
                        map_type = 'map_b'
                    else:
                        map_type = 'map_z'  # Default
                else:
                    map_type = 'map_z'  # Default fallback
            
            # Chuẩn bị request body
            request_data = {
                "text": text,
                "node_id": str(node_id),
                "token": API_CONFIG["token"],
                "map_type": map_type
            }
            
            # URL endpoint
            url = API_CONFIG["base_url"] + API_CONFIG["endpoints"]["submit"]
            
            rospy.loginfo(" GỬI DỮ LIỆU LÊN SERVER:")
            rospy.loginfo(f"   URL: {url}")
            rospy.loginfo(f"   Text: {text}")
            rospy.loginfo(f"   Node ID: {node_id}")
            rospy.loginfo(f"   Map Type: {map_type}")
            
            # Gửi POST request
            response = requests.post(
                url,
                json=request_data,
                headers={'Content-Type': 'application/json'},
                timeout=API_CONFIG["timeout"]
            )
            
            # Kiểm tra response (200 và 201 đều là thành công)
            if response.status_code in [200, 201]:
                rospy.loginfo(" GỬI DỮ LIỆU THÀNH CÔNG!")
                try:
                    response_data = response.json()
                    rospy.loginfo(f"   Response: {response_data}")
                except:
                    rospy.loginfo(f"   Response: {response.text}")
                return True
            elif response.status_code == 400:
                # Xử lý lỗi 400 - Maximum submissions reached
                try:
                    error_data = response.json()
                    if "Maximum number of submissions reached" in error_data.get("error", ""):
                        rospy.logwarn(" ĐÃ GỬI QUÁ NHIỀU LẦN CHO NODE NÀY - BỎ QUA!")
                        rospy.logwarn(f"   Node: {node_id}, Text: {text}")
                        return True  # Coi như thành công vì đã gửi rồi
                except:
                    pass
                
                rospy.logerr(f" LỖI 400 - BAD REQUEST!")
                rospy.logerr(f"   Response: {response.text}")
                return False
            else:
                rospy.logerr(f" GỬI DỮ LIỆU THẤT BẠI!")
                rospy.logerr(f"   Status Code: {response.status_code}")
                rospy.logerr(f"   Response: {response.text}")
                return False
                
        except requests.exceptions.Timeout:
            rospy.logerr(" TIMEOUT khi gửi dữ liệu lên server!")
            return False
        except requests.exceptions.ConnectionError:
            rospy.logerr(" LỖI KẾT NỐI khi gửi dữ liệu lên server!")
            return False
        except Exception as e:
            rospy.logerr(f" LỖI KHÔNG MONG MUỐN khi gửi dữ liệu: {e}")
            return False

    def cleanup(self):
        """Dọn dẹp tài nguyên"""
        rospy.loginfo(" Đang dọn dẹp...")
        
        if hasattr(self, 'robot'):
            self.robot.stop()
        
        if hasattr(self, 'edge_detector'):
            self.edge_detector.stop_scanning()
        
        rospy.loginfo(" Đã dọn dẹp xong")

def main():
    """Main function"""
    try:
        rospy.init_node('problem_a_solver', anonymous=True)
        
        print("\n" + "="*60)
        print(" FPT HACKATHON 2025 - PROBLEM A")
        print("="*60)
        print(" KHỞI TẠO HỆ THỐNG...")
        
        solver = ProblemASolver()
        success = solver.run_navigation()
        
        if success:
            print(" PROBLEM A HOÀN THÀNH THÀNH CÔNG!")
            return True
        else:
            print(" PROBLEM A THẤT BẠI!")
            return False
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Problem A bị ngắt bởi ROS")
        return False
    except KeyboardInterrupt:
        rospy.loginfo("Problem A bị ngắt bởi Ctrl+C")
        return False
    except Exception as e:
        rospy.logerr(f"Lỗi không mong muốn: {e}")
        return False

if __name__ == "__main__":
    main()