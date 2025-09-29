"""
Configuration file for FPT Hackathon 2025
Chứa các cấu hình chung cho toàn bộ project
"""

# API Configuration
API_CONFIG = {
    "base_url": "https://hackathon2025-dev.fpt.edu.vn",
    "token": "0fc23b6180e87b08179607c4ad6861d3",
    "timeout": 5,
    "endpoints": {
        "get_map": "/api/maps/get_active_map/",
        "submit": "/api/sign-submissions/submit/"
    }
}

# Map Types
MAP_TYPES = {
    "SAMPLE": "map_b",    # Map mẫu - luôn mở
    "PROBLEM_A": "map_a", # Map cho problem A - mở sau 27h
    "PROBLEM_B": "map_b"  # Map cho problem B - mở sau 27h
}

# Direction Mapping (Hướng đi)
DIRECTION_MAP = {
    (1, 0): 'E',   # Đi sang phải (Đông - East)
    (-1, 0): 'W',  # Đi sang trái (Tây - West)  
    (0, 1): 'S',   # Đi xuống dưới (Nam - South)
    (0, -1): 'N'   # Đi lên trên (Bắc - North)
}

# Output Configuration
OUTPUT_CONFIG = {
    "map_files": {
        "sample": "map_z.json",
        "problem_a": "map_a.json", 
        "problem_b": "map_b.json"
    },
    "encoding": "utf-8",
    "indent": 2
}

# Algorithm Configuration
PATHFINDING_CONFIG = {
    "algorithm": "dijkstra",  # hoặc "astar"
    "max_iterations": 10000,
    "timeout_seconds": 15
}

# Logging Configuration
LOGGING_CONFIG = {
    "level": "INFO",
    "format": "%(asctime)s - %(levelname)s - %(message)s",
    "enable_debug": False
}