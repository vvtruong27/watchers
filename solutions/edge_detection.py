#!/usr/bin/env python3
"""
Edge (Intersection) Detection using LiDAR
Phát hiện đỉnh (giao lộ) bằng cách tìm 2 cột đối nhau tại hướng NE và SW
Bám sát sample_code/opposite_detector.py

CHỨC NĂNG:
- EdgeDetector: Class phát hiện giao lộ bằng LiDAR
- start_scanning()/stop_scanning(): Bật/tắt quét LiDAR
- lidar_callback(): Nhận dữ liệu từ /scan topic
- find_objects_in_directions(): Tìm vật thể ở hướng NE (45°) và SW (225°)
- check_intersection(): Kiểm tra xem có phải giao lộ không
- detect_edge(): Hàm chính để phát hiện đỉnh
- get_detection_info(): Lấy thông tin chi tiết

NGUYÊN LÝ:
- Tại mỗi giao lộ có 2 cột đặt tại hướng Đông Bắc (NE-45°) và Tây Nam (SW-225°)
- LiDAR quét 360° tìm vật thể trong khoảng cách 0.25-0.35m
- Nếu phát hiện đồng thời 2 vật thể đối nhau ở NE và SW → Giao lộ

SỬ DỤNG:
- Import class này vào problem_a.py
- Khởi tạo detector và gọi detect_edge() để kiểm tra
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import time
import math

class EdgeDetector:
    """
    Phát hiện giao lộ (đỉnh) bằng LiDAR - tìm 2 cột đối nhau tại NE và SW
    Bám sát SimpleOppositeDetector từ sample_code
    """
    
    def __init__(self):
        """Khởi tạo Edge Detector - copy từ sample_code với logging chi tiết"""
        # Tham số phát hiện vật thể - copy từ sample_code line 14-21
        self.min_distance = 0.25                # Khoảng cách tối thiểu
        self.max_distance = 0.35                # Khoảng cách tối đa  
        self.object_min_points = 15             # Số điểm tối thiểu của vật thể
        self.distance_threshold = 0.10          # Ngưỡng khoảng cách giữa các điểm
        self.angle_range = 20.0                 # Phạm vi góc quét
        self.detection_interval = 2.0           # Khoảng thời gian giữa các lần phát hiện
        
        # Tham số phát hiện đối nhau - copy từ sample_code line 19-20
        self.opposite_tolerance = 5.0           # Dung sai góc đối nhau (±5°)
        self.min_opposite_distance = 45.0       # Khoảng cách góc tối thiểu
        
        # Tham số riêng cho NE-SW
        self.ne_angle_center = 45.0             # Hướng Đông Bắc (NE) - 45°
        self.sw_angle_center = 225.0            # Hướng Tây Nam (SW) - 225°  
        self.angle_tolerance = 30.0             # Dung sai ±30° cho mỗi hướng
        
        # Biến trạng thái - copy từ sample_code
        self.scanning_active = False
        self.subscriber = None
        self.last_detection_time = 0
        self.latest_scan = None
        
        # Logging chi tiết
        self.scan_count = 0                     # Đếm số lần quét
        self.detection_count = 0                # Đếm số lần phát hiện 
        self.last_log_time = 0                  # Thời gian log cuối
        
        rospy.loginfo("🔍 Edge Detector initialized với tham số:")
        rospy.loginfo(f"   📏 Khoảng cách: {self.min_distance}m - {self.max_distance}m")
        rospy.loginfo(f"   🎯 Hướng phát hiện: NE({self.ne_angle_center}°) ± {self.angle_tolerance}°")
        rospy.loginfo(f"   🎯 Hướng phát hiện: SW({self.sw_angle_center}°) ± {self.angle_tolerance}°")
    
    def start_scanning(self):
        """
        Bắt đầu quét LiDAR - copy từ sample_code line 44-56 với logging
        """
        try:
            if not self.scanning_active:
                self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
                self.scanning_active = True
                self.scan_count = 0
                self.detection_count = 0
                self.last_log_time = time.time()
                rospy.loginfo("🟢 Edge Detection: BẮT ĐẦU QUÉT LIDAR")
                rospy.loginfo(f"   📡 Đang lắng nghe topic: /scan")
                rospy.loginfo(f"   🎯 Tìm kiếm cột NE-SW để phát hiện giao lộ")
                return True
            else:
                rospy.logdebug("🟡 LiDAR đã đang quét")
                return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to start edge scanning: {e}")
            return False
    
    def stop_scanning(self):
        """
        Dừng quét LiDAR - copy từ sample_code line 58-71 với logging 
        """
        try:
            if self.scanning_active:
                if self.subscriber:
                    self.subscriber.unregister()
                    self.subscriber = None
                self.scanning_active = False
                self.latest_scan = None
                rospy.loginfo("🔴 Edge Detection: DỪNG QUÉT LIDAR")
                rospy.loginfo(f"   📊 Tổng cộng: {self.scan_count} lần quét, {self.detection_count} lần phát hiện")
                return True
            else:
                rospy.logdebug("🟡 LiDAR đã dừng")
                return True
        except Exception as e:
            rospy.logerr(f"❌ Failed to stop edge scanning: {e}")
            return False

    def callback(self, scan):
        """
        Callback xử lý dữ liệu LiDAR - copy CHÍNH XÁC từ sample_code line 78-83
        """
        if not self.scanning_active: 
            return
        
        self.latest_scan = scan
        self.scan_count += 1
        current_time = time.time()
        
        # Log trạng thái LiDAR định kỳ
        if current_time - self.last_log_time >= 5.0:  # Log mỗi 5s
            rospy.loginfo(f"📡 LiDAR Status: Quét #{self.scan_count}, "
                         f"Phát hiện: {self.detection_count}, "
                         f"Range: {len(scan.ranges)} điểm")
            self.last_log_time = current_time
        
        # XỬ LÝ DETECTION THEO INTERVAL như sample_code
        if current_time - self.last_detection_time >= self.detection_interval:
            try:
                detected = self.process_detection()
                if detected:
                    rospy.logwarn(f"🚨 CALLBACK: GIAO LỘ ĐƯỢC PHÁT HIỆN!")
            except Exception as e:
                rospy.logerr(f"❌ Lỗi trong process_detection: {e}")
            
            self.last_detection_time = current_time
    
    def index_to_angle(self, index, scan):
        """
        Chuyển đổi index thành góc - copy từ sample_code line 80-82
        """
        angle_rad = scan.angle_min + (index * scan.angle_increment)
        return math.degrees(angle_rad)
    
    def normalize_angle(self, angle):
        """
        Chuẩn hóa góc về [0, 360)
        """
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle
    
    def get_angle_difference(self, angle1, angle2):
        """
        Tính khoảng cách góc - copy từ sample_code line 84-86
        """
        diff = abs(angle1 - angle2)
        return 360 - diff if diff > 180 else diff
    
    def are_opposite(self, angle1, angle2):
        """
        Kiểm tra 2 góc có đối nhau không - copy từ sample_code line 88-89
        """
        return abs(self.get_angle_difference(angle1, angle2) - 180.0) <= self.opposite_tolerance
    
    def is_in_direction_range(self, angle, target_angle, tolerance):
        """
        Kiểm tra góc có nằm trong phạm vi hướng mong muốn không
        """
        angle = self.normalize_angle(angle)
        target_angle = self.normalize_angle(target_angle)
        
        diff = self.get_angle_difference(angle, target_angle)
        return diff <= tolerance
    
    def detect_object_in_zone(self, zone_ranges, zone_name):
        """
        Phát hiện vật thể trong một vùng - copy từ sample_code line 151-174
        """
        if len(zone_ranges) == 0: 
            return None
            
        valid_mask = (zone_ranges >= self.min_distance) & (zone_ranges <= self.max_distance) & np.isfinite(zone_ranges)
        if np.sum(valid_mask) < self.object_min_points: 
            return None
            
        valid_ranges = zone_ranges[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        
        # Gom các điểm gần nhau thành clusters
        clusters, current_cluster = [], [0]
        for i in range(1, len(valid_ranges)):
            if (valid_indices[i] - valid_indices[current_cluster[-1]] <= 2 and 
                abs(valid_ranges[i] - valid_ranges[current_cluster[-1]]) <= self.distance_threshold):
                current_cluster.append(i)
            else:
                if len(current_cluster) >= self.object_min_points: 
                    clusters.append(current_cluster)
                current_cluster = [i]
                
        if len(current_cluster) >= self.object_min_points: 
            clusters.append(current_cluster)
        if not clusters: 
            return None
            
        # Lấy cluster lớn nhất
        largest_cluster = max(clusters, key=len)
        cluster_distances = [valid_ranges[i] for i in largest_cluster]
        
        return {
            'distance': np.mean(cluster_distances), 
            'point_count': len(largest_cluster), 
            'zone': zone_name
        }
    
    def find_all_objects(self, scan):
        """
        Tìm TẤT CẢ vật thể - copy CHÍNH XÁC từ sample_code line 97-112
        """
        ranges = np.array(scan.ranges)
        n = len(ranges)
        angle_increment_deg = math.degrees(scan.angle_increment)
        points_per_range = int(self.angle_range / angle_increment_deg)
        objects = []
        
        for start_idx in range(0, n, points_per_range // 2):
            end_idx = min(start_idx + points_per_range, n)
            if end_idx - start_idx < points_per_range // 2: 
                continue
                
            zone_ranges = ranges[start_idx:end_idx]
            center_idx = start_idx + (end_idx - start_idx) // 2
            center_angle = self.index_to_angle(center_idx, scan)
            
            obj = self.detect_object_in_zone(zone_ranges, f"Zone_{start_idx}")
            if obj:
                obj['center_angle'] = center_angle
                obj['center_index'] = center_idx
                objects.append(obj)
        
        return objects

    def find_opposite_pairs(self, objects):
        """
        Tìm cặp vật thể đối nhau - copy CHÍNH XÁC từ sample_code line 114-125
        """
        opposite_pairs = []
        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects[i+1:], i+1):
                angle_diff = self.get_angle_difference(obj1['center_angle'], obj2['center_angle'])
                if angle_diff >= self.min_opposite_distance and self.are_opposite(obj1['center_angle'], obj2['center_angle']):
                    opposite_pairs.append({'object1': obj1, 'object2': obj2, 'angle_difference': angle_diff})
        return opposite_pairs
    
    def check_intersection(self):
        """
        Kiểm tra xem có phải giao lộ không - wrapper cho process_detection
        """
        return self.process_detection()
    
    def process_detection(self):
        """
        Xử lý phát hiện giao lộ - copy CHÍNH XÁC từ sample_code line 127-150
        """
        if self.latest_scan is None: 
            return False
            
        scan = self.latest_scan
        timestamp = rospy.get_time()
        
        # Tìm TẤT CẢ vật thể như sample_code
        all_objects = self.find_all_objects(scan)
        if len(all_objects) < 2: 
            rospy.logdebug(f"📡 LiDAR: Chỉ tìm thấy {len(all_objects)} vật thể (cần ít nhất 2)")
            return False
        
        # Tìm cặp đối nhau như sample_code
        opposite_pairs = self.find_opposite_pairs(all_objects)
        
        if opposite_pairs:
            # Sắp xếp theo độ gần với 180°
            opposite_pairs.sort(key=lambda x: abs(x['angle_difference'] - 180.0))
            best_pair = opposite_pairs[0]
            
            self.detection_count += 1
            
            # Log chi tiết như sample_code
            obj1 = best_pair['object1'] 
            obj2 = best_pair['object2']
            angle_diff = best_pair['angle_difference']
            
            rospy.logwarn(f"🚨 [{timestamp:.1f}] *** OPPOSITE OBJECTS DETECTED *** (#{self.detection_count})")
            rospy.logwarn(f"   📍 Object 1: {obj1['center_angle']:.1f}°, "
                         f"distance: {obj1['distance']:.2f}m, "
                         f"points: {obj1['point_count']}")
            rospy.logwarn(f"   📍 Object 2: {obj2['center_angle']:.1f}°, "
                         f"distance: {obj2['distance']:.2f}m, "
                         f"points: {obj2['point_count']}")
            rospy.logwarn(f"   📐 Angle difference: {angle_diff:.1f}° (target: 180°)")
            rospy.logwarn(f"   ✅ GIAO LỘ XÁC NHẬN!")
            
            return True
        else:
            rospy.logdebug(f"📡 LiDAR: Tìm thấy {len(all_objects)} vật thể nhưng không có cặp đối nhau")
            return False
    
    def detect_edge(self):
        """
        Hàm chính để phát hiện đỉnh (giao lộ)
        Trả về True nếu phát hiện giao lộ, False nếu không
        """
        if not self.scanning_active:
            rospy.logwarn("Edge detection not active. Call start_scanning() first.")
            return False
            
        return self.process_detection()
    
    def get_detection_info(self):
        """
        Lấy thông tin chi tiết về việc phát hiện giao lộ - tương thích với logic mới
        """
        if self.latest_scan is None:
            return None
            
        all_objects = self.find_all_objects(self.latest_scan)
        opposite_pairs = self.find_opposite_pairs(all_objects)
        
        info = {
            'timestamp': rospy.get_time(),
            'total_objects': len(all_objects),
            'opposite_pairs': len(opposite_pairs),
            'intersection_detected': len(opposite_pairs) > 0,
            'objects': all_objects,
            'best_pair': None
        }
        
        if opposite_pairs:
            opposite_pairs.sort(key=lambda x: abs(x['angle_difference'] - 180.0))
            info['best_pair'] = opposite_pairs[0]
        
        return info