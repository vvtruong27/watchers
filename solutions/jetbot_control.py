#!/usr/bin/env python3
"""
JetBot Control Functions - PHIÊN BẢ        # Tham số cho ĐI THẲNG GIỮ VỊ TRÍ - TĂNG ĐỘ BÁM LINE
        self.SAFE_ZONE_PERCENT = 0.45            # 45% vùng an toàn - tăng độ bám line
        self.CORRECTION_GAIN = 0.22              # Gain tăng nhẹ - bám line tốt hơn
        self.MAX_CORRECTION_ADJ = 0.06           # Giới hạn tăng nhẹ - vẫn tránh rẽ mạnhI TIẾN BÁM ĐƯỜNG NHẸ
Các hàm điều khiển JetBot với khả năng bám line nhẹ nhàng

CHỨC NĂNG:
- JetBotController: Class điều khiển robot với bám đường nhẹ
- correct_course(): Bám đường nhẹ dựa trên sample_code
- Tránh bẻ lái quá gắt, chỉ điều chỉnh khi cần thiết
"""

import rospy
import cv2
import numpy as np
import time
from enum import Enum

from jetbot import Robot
from sensor_msgs.msg import Image

class RobotState(Enum):
    """Trạng thái robot đơn giản"""
    WAITING_FOR_LINE = 0
    DRIVING_STRAIGHT = 1  
    STOPPED = 2

class JetBotController:
    """
    Controller với khả năng bám đường nhẹ
    """
    
    def __init__(self):
        rospy.loginfo("Khởi tạo JetBot Controller với bám đường nhẹ...")
        self.setup_parameters()
        self.initialize_hardware()
        
        # Trạng thái
        self.current_state = RobotState.WAITING_FOR_LINE
        self.latest_image = None
        self.state_change_time = rospy.get_time()
        
        # ROS subscribers
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        rospy.loginfo("Đã đăng ký camera topic /csi_cam_0/image_raw")
        
        rospy.loginfo("Controller với bám đường nhẹ đã sẵn sàng!")

    def setup_parameters(self):
        """Thiết lập tham số"""
        self.WIDTH, self.HEIGHT = 300, 300
        self.BASE_SPEED = 0.3
        
        # ROI parameters
        self.ROI_Y = int(self.HEIGHT * 0.75)      
        self.ROI_H = int(self.HEIGHT * 0.25)        
        self.ROI_CENTER_WIDTH_PERCENT = 0.6       
        
        # Line detection
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])     
        self.LINE_COLOR_UPPER = np.array([180, 255, 75]) 
        self.SCAN_PIXEL_THRESHOLD = 100          
        
        # Tham số cho ĐI THẲNG GIỮ VỊ TRÍ - TĂNG ĐỘ BÁM LINE
        self.SAFE_ZONE_PERCENT = 0.45            # 45% vùng an toàn - tăng độ bám line
        self.CORRECTION_GAIN = 0.6              # Gain tăng nhẹ - bám line tốt hơn
        self.MAX_CORRECTION_ADJ = 0.06           # Giới hạn tăng nhẹ - vẫn tránh rẽ mạnh
        
        rospy.loginfo("✅ Đã thiết lập tham số đi thẳng - TĂNG ĐỘ BÁM LINE")

    def initialize_hardware(self):
        """Khởi tạo hardware"""
        try:
            self.robot = Robot()
            rospy.loginfo("Khởi tạo JetBot hardware thành công")
        except Exception as e:
            rospy.logwarn(f"Lỗi khởi tạo hardware: {e}")
            from unittest.mock import Mock
            self.robot = Mock()

    def camera_callback(self, image_msg):
        """Camera callback"""
        try:
            if image_msg.encoding.endswith('compressed'):
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
            if 'rgb' in image_msg.encoding: 
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.latest_image = cv2.resize(cv_image, (self.WIDTH, self.HEIGHT))
        except Exception as e: 
            rospy.logerr(f"Lỗi camera callback: {e}")
    
    def _get_line_center(self, image, roi_y, roi_h):
        """
        Phát hiện line center - copy từ sample_code
        """
        if image is None: 
            return None
        roi = image[roi_y : roi_y + roi_h, :]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Bước 1: Tạo mặt nạ màu sắc
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Bước 2: Tạo mặt nạ tập trung
        focus_mask = np.zeros_like(color_mask)
        roi_height, roi_width = focus_mask.shape
        
        center_width = int(roi_width * self.ROI_CENTER_WIDTH_PERCENT)
        start_x = (roi_width - center_width) // 2
        end_x = start_x + center_width
        
        cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
        
        # Bước 3: Kết hợp hai mặt nạ
        final_mask = cv2.bitwise_and(color_mask, focus_mask)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None
            
        c = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(c) < self.SCAN_PIXEL_THRESHOLD:
            return None

        M = cv2.moments(c)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
        return None
    
    def correct_course(self, line_center_x):
        """
        BÁM LINE TRÊN ĐƯỜNG THẲNG - Tăng độ bám nhưng không rẽ theo line cong
        Điều chỉnh vừa phải để bám line tốt trên đường thẳng, tránh rẽ theo line cong
        """
        error = line_center_x - (self.WIDTH / 2)
        
        # VÙNG AN TOÀN VỪA PHẢI - tăng độ bám line nhưng tránh rẽ theo line cong
        # Giảm vùng an toàn từ 60% xuống 45% để tăng độ bám
        safe_zone_pixels = (self.WIDTH / 2) * 0.45  # 45% vùng an toàn
        if abs(error) < safe_zone_pixels:
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            rospy.logdebug(f"🎯 Line trong vùng an toàn ({error:.0f}px) - đi thẳng")
            return

        # ĐIỀU CHỈNH VỪA PHẢI - tăng độ bám line nhưng không rẽ theo line cong
        # Tăng gain từ 0.15 lên 0.22 để bám line tốt hơn
        correction_gain = 0.6  # Vừa phải
        adj = (error / (self.WIDTH / 2)) * correction_gain
        
        # GIỚI HẠN TỐI ĐA VỪa PHẢI - tăng độ bám nhưng tránh rẽ mạnh
        # Tăng từ 0.04 lên 0.06 để bám line tốt hơn
        max_adjustment = 0.06  # Vừa phải
        adj = np.clip(adj, -max_adjustment, max_adjustment)
        
        # Áp dụng điều chỉnh rất nhẹ
        left_motor = self.BASE_SPEED + adj
        right_motor = self.BASE_SPEED - adj
        self.robot.set_motors(left_motor, right_motor)
        
        # Log để theo dõi
        direction = "trái" if adj > 0 else "phải"
        rospy.logdebug(f"🔧 Bám line {direction}: {abs(adj):.3f} (lỗi: {error:.0f}px - TĂNG ĐỘ BÁM trên đường thẳng)")

    def follow_line_continuous(self):
        """
        Bám line liên tục với khả năng điều chỉnh nhẹ
        """
        if self.latest_image is None:
            rospy.logwarn_throttle(5, "⚠️ Đang chờ dữ liệu camera...")
            self.robot.stop()
            return {'status': 'no_camera'}
        
        # Phát hiện line
        line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
        
        if line_center is not None:
            # Có line - bám đường nhẹ với correct_course
            if self.current_state != RobotState.DRIVING_STRAIGHT:
                self._set_state(RobotState.DRIVING_STRAIGHT)
            
            # Sử dụng correct_course để bám đường nhẹ
            self.correct_course(line_center)
            return {'status': 'following_line', 'line_position': line_center}
        else:
            # Mất line - TRẢ VỀ SIGNAL CẦN BACKUP
            if self.current_state != RobotState.STOPPED:
                rospy.logwarn("⚠️ HẾT LINE - CẦN KIỂM TRA BACKUP!")
                self._set_state(RobotState.STOPPED)
            
            self.robot.stop()
            return {'status': 'line_lost', 'need_backup': True}

    def scan_qr_at_current_position(self):
        """Quét QR tại vị trí hiện tại để backup khi hết line"""
        if self.latest_image is None:
            return None
            
        try:
            from pyzbar.pyzbar import decode
            decoded_objects = decode(self.latest_image)
            
            if decoded_objects:
                qr_data = decoded_objects[0].data.decode('utf-8').strip()
                rospy.logwarn(f"📱 BACKUP QR: Tìm thấy QR '{qr_data}' tại vị trí hiện tại")
                return qr_data
            return None
        except Exception as e:
            rospy.logerr(f"❌ Lỗi quét QR backup: {e}")
            return None

    def align_to_line_center(self, max_attempts=10):
        """
        Căn chỉnh line về giữa - SỬ DỤNG CORRECT_COURSE
        """
        rospy.loginfo("🎯 Bắt đầu căn chỉnh line về giữa với bám đường nhẹ...")
        
        for attempt in range(max_attempts):
            if self.latest_image is None:
                rospy.logwarn("⚠️ Không có ảnh để căn chỉnh")
                time.sleep(0.1)
                continue
            
            # Kiểm tra vị trí line hiện tại
            line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
            
            if line_center is None:
                rospy.logwarn(f"⚠️ Không thấy line (lần {attempt+1}/{max_attempts})")
                time.sleep(0.1)
                continue
            
            # Sử dụng correct_course để căn chỉnh
            self.correct_course(line_center)
            time.sleep(0.3)  # Cho robot thời gian di chuyển
            
            # Kiểm tra lại
            line_center_after = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
            if line_center_after is not None:
                error = abs(line_center_after - (self.WIDTH / 2))
                safe_zone = (self.WIDTH / 2) * self.SAFE_ZONE_PERCENT
                
                if error < safe_zone:
                    rospy.loginfo("✅ Line đã ở giữa!")
                    self.robot.stop()
                    return True
        
        rospy.logwarn(f"⚠️ Căn chỉnh chưa hoàn hảo sau {max_attempts} lần thử")
        return False

    def handle_line_lost_backup(self, current_direction):
        """Xử lý backup khi hết line - quay NE kiểm tra QR"""
        rospy.logwarn("🔄 BACKUP: Hết line, quay NE kiểm tra QR...")
        
        return {
            'action': 'check_qr_backup',
            'original_direction': current_direction,
            'target_angle': 45  # NE
        }

    def _set_state(self, new_state):
        """Helper để chuyển trạng thái"""
        if self.current_state != new_state:
            rospy.loginfo(f"Chuyển trạng thái: {self.current_state.name} -> {new_state.name}")
            self.current_state = new_state
            self.state_change_time = rospy.get_time()

    def stop_robot(self):
        """Dừng robot ngay lập tức"""
        rospy.loginfo("⛔ DỪNG ROBOT")
        self.robot.stop()
        self._set_state(RobotState.STOPPED)

    def cleanup(self):
        """Dọn dẹp"""
        rospy.loginfo("🧹 Dừng robot và dọn dẹp...")
        if hasattr(self, 'robot') and self.robot is not None:
            self.robot.stop()
        rospy.loginfo("✅ Đã dọn dẹp JetBot Controller")

    def run(self):
        """
        Vòng lặp chính - BÁM ĐƯỜNG NHẸ
        """
        rospy.loginfo("🚀 Bắt đầu Line Follower - ĐI THẲNG TĂNG ĐỘ BÁM")
        rospy.loginfo("   📋 Logic: Có line -> Bám line trên đường thẳng, Mất line -> Dừng")
        rospy.loginfo("   🚨 TĂNG ĐỘ BÁM nhưng KHÔNG RẼ THEO LINE CONG")
        time.sleep(3)  # Đợi camera ổn định
        
        rate = rospy.Rate(20)  # 20Hz
        consecutive_line_lost = 0
        
        while not rospy.is_shutdown():
            if self.latest_image is None:
                rospy.logwarn_throttle(5, "⚠️ Đang chờ dữ liệu camera...")
                self.robot.stop()
                rate.sleep()
                continue
            
            # Phát hiện line
            line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
            
            if line_center is not None:
                # Có line - reset counter và bám theo
                consecutive_line_lost = 0
                if self.current_state != RobotState.DRIVING_STRAIGHT:
                    rospy.loginfo("✅ Tìm thấy line - bắt đầu bám line trên đường thẳng")
                    self._set_state(RobotState.DRIVING_STRAIGHT)
                
                self.correct_course(line_center)
                rospy.loginfo_throttle(3, f"🛣️ BÁM LINE TRÊN ĐƯỜNG THẲNG - TĂNG ĐỘ BÁM (center: {line_center})")
            
            else:
                # Mất line
                consecutive_line_lost += 1
                
                if consecutive_line_lost < 5:  # Cho phép mất line tạm thời
                    rospy.logwarn_throttle(1, 
                        f"⚠️ Mất line tạm thời ({consecutive_line_lost}/5) - Tiếp tục đi chậm")
                    # Tiếp tục đi thẳng với tốc độ rất chậm
                    self.robot.set_motors(0.08, 0.08)
                else:
                    # Mất line hoàn toàn - DỪNG
                    rospy.logerr("❌ MẤT LINE HOÀN TOÀN - DỪNG ROBOT!")
                    if self.current_state != RobotState.STOPPED:
                        self._set_state(RobotState.STOPPED)
                    self.robot.stop()
                    break

            rate.sleep()
        
        self.cleanup()

# LOGIC ĐI THẲNG GIỮ VỊ TRÍ - TĂNG ĐỘ BÁM LINE:
# - Vùng an toàn vừa phải (45% width) - tăng độ bám line trên đường thẳng
# - Điều chỉnh khi line lệch - gain tăng lên 0.22 để bám tốt hơn  
# - Giới hạn tối đa vừa phải 0.06 - bám line tốt nhưng vẫn tránh rẽ mạnh
# - MỤC TIÊU: Bám line ổn định trên đường thẳng, không bị cuốn theo line cong/rẽ