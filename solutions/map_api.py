"""
Map API Handler for FPT Hackathon 2025
Xử lý việc lấy và xử lý dữ liệu bản đồ từ API
Version: Cải tiến để xử lý đúng định dạng bản đồ thực tế
"""

import requests
import json
import networkx as nx
from typing import Dict, List, Optional, Any, Tuple
import logging
from config import API_CONFIG, OUTPUT_CONFIG

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MapAPIHandler:
    """
    Lớp xử lý API bản đồ và các thao tác liên quan.
    """
    
    def __init__(self):
        """Khởi tạo handler với cấu hình từ config.py"""
        self.base_url = API_CONFIG["base_url"]
        self.token = API_CONFIG["token"]
        self.timeout = API_CONFIG["timeout"]
        self.endpoints = API_CONFIG["endpoints"]
        
    def get_map_from_api(self, map_type: str = "map_z") -> Optional[Dict[str, Any]]:
        """
        Lấy bản đồ từ API server.
        
        Args:
            map_type (str): Loại bản đồ (map_a, map_b, map_z)
        
        Returns:
            Optional[Dict]: Dữ liệu bản đồ từ API hoặc None nếu lỗi
        """
        url = f"{self.base_url}{self.endpoints['get_map']}"
        params = {
            "token": self.token,
            "map_type": map_type
        }
        
        try:
            logger.info(f" Đang lấy bản đồ: {map_type}")
            response = requests.get(url, params=params, timeout=self.timeout)
            response.raise_for_status()
            
            data = response.json()
            logger.info(f" Lấy bản đồ thành công: {data.get('name', 'Unknown')}")
            
            return data
            
        except requests.exceptions.Timeout:
            logger.error(f"Timeout khi gọi API sau {self.timeout}s")
            return None
        except requests.exceptions.ConnectionError:
            logger.error("Lỗi kết nối đến server")
            return None
        except requests.exceptions.HTTPError as e:
            logger.error(f"HTTP Error: {e}")
            return None
        except json.JSONDecodeError as e:
            logger.error(f" Lỗi parse JSON: {e}")
            return None
        except Exception as e:
            logger.error(f" Lỗi không xác định: {e}")
            return None
    
    def process_map_data(self, raw_map_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Xử lý dữ liệu bản đồ thô từ API thành định dạng chuẩn.
        
        Args:
            raw_map_data: Dữ liệu thô từ API
        
        Returns:
            Optional[Dict]: Bản đồ đã xử lý hoặc None nếu lỗi
        """
        if not raw_map_data:
            logger.error("Không có dữ liệu để xử lý")
            return None
            
        # Tạo dictionary để tra cứu nhanh node theo ID
        nodes_lookup = {node['id']: node for node in raw_map_data['nodes']}
        
        # Xử lý nodes - chuyển đổi định dạng
        processed_nodes = []
        start_nodes = []  # Có thể có nhiều start
        end_nodes = []
        load_nodes = []   # Nodes có type="Load"
        
        for node in raw_map_data['nodes']:
            node_type = node.get('type', 'None')
            
            # Phân loại nodes
            if node_type == "Start":
                start_nodes.append(node['id'])
            elif node_type == "End":
                end_nodes.append(node['id'])
            elif node_type == "Load":
                load_nodes.append(node['id'])
            
            processed_nodes.append({
                "id": node['id'],
                "x": node['x'],
                "y": node['y'],
                "type": node_type.lower() if node_type in ["Start", "End"] else "normal",
                "name": node.get('name', f"Node_{node['id']}"),
                "original_type": node_type  # Giữ lại type gốc (Load, None, etc.)
            })
        
        # Xác định start node
        # Ưu tiên 1: startingPositions
        # Ưu tiên 2: node có type="Start"
        start_node = None
        
        if 'startingPositions' in raw_map_data and raw_map_data['startingPositions']:
            start_node = raw_map_data['startingPositions'][0]
            logger.info(f"Start node từ startingPositions: {start_node}")
        elif start_nodes:
            start_node = start_nodes[0]
            logger.info(f"Start node từ type='Start': {start_node}")
        else:
            logger.error(" Không tìm thấy start node!")
            return None
        
        # Log thông tin nodes đặc biệt
        start_node_name = nodes_lookup.get(start_node, {}).get('name', 'Unknown')
        logger.info(f" Start: Node {start_node} ('{start_node_name}')")
        
        if end_nodes:
            end_names = [f"{n}('{nodes_lookup.get(n, {}).get('name', '?')}')" for n in end_nodes]
            logger.info(f" End nodes: {', '.join(end_names)}")
        
        if load_nodes:
            load_names = [f"{n}('{nodes_lookup.get(n, {}).get('name', '?')}')" for n in load_nodes]
            logger.info(f" Load nodes: {', '.join(load_names)}")
        
        # Xử lý edges - sử dụng trực tiếp label do API cung cấp
        processed_edges = []
        direction_stats = {'N': 0, 'E': 0, 'S': 0, 'W': 0}
        
        for edge in raw_map_data.get('edges', []):
            try:
                source_id = edge['source']
                target_id = edge['target']
                direction_label = edge.get('label')

                if direction_label is None:
                    logger.warning(f" Edge thiếu nhãn hướng: {source_id}→{target_id}")
                    continue

                if source_id not in nodes_lookup or target_id not in nodes_lookup:
                    logger.warning(f" Edge tham chiếu node không tồn tại: {source_id}→{target_id}")
                    continue

                # Thống kê hướng
                if direction_label in direction_stats:
                    direction_stats[direction_label] += 1
                
                processed_edges.append({
                    "source": source_id,
                    "target": target_id,
                    "label": direction_label,
                    "id": edge.get('id', f"{source_id}-{target_id}")
                })
                
            except KeyError as e:
                logger.warning(f" Edge thiếu trường bắt buộc: {edge} ({e})")
            except Exception as e:
                logger.warning(f" Lỗi xử lý edge {edge}: {e}")
        
        # Log thống kê chi tiết
        logger.info("="*50)
        logger.info(" THỐNG KÊ BẢN ĐỒ:")
        logger.info(f"    Tổng số nodes: {len(processed_nodes)}")
        logger.info(f"    Tổng số edges: {len(processed_edges)}")
        logger.info(f"   Kích thước: {raw_map_data.get('dimensions', {}).get('width')}x{raw_map_data.get('dimensions', {}).get('height')}")
        logger.info(" THỐNG KÊ HƯỚNG ĐI:")
        
        direction_names = {'N': 'Bắc', 'E': 'Đông', 'S': 'Nam', 'W': 'Tây'}
        for direction, count in direction_stats.items():
            if count > 0:
                logger.info(f"   {direction} ({direction_names[direction]}): {count} cạnh")
        logger.info("="*50)
        
        # Tạo bản đồ đã xử lý
        processed_map = {
            "map_info": {
                "id": raw_map_data.get('id'),
                "name": raw_map_data.get('name'),
                "mapType": raw_map_data.get('mapType'),
                "dimensions": raw_map_data.get('dimensions', {}),
                "start_node": start_node,
                "end_nodes": end_nodes,
                "load_nodes": load_nodes,  # Thêm thông tin load nodes
                "starting_positions": raw_map_data.get('startingPositions', []),
                "destination_positions": raw_map_data.get('destinationPositions', []),
                "total_nodes": len(processed_nodes),
                "total_edges": len(processed_edges),
                "direction_stats": direction_stats
            },
            "nodes": processed_nodes,
            "edges": processed_edges
        }
        
        return processed_map
    
    def save_map_to_file(self, processed_map: Dict[str, Any], filename: str = None) -> bool:
        """
        Lưu bản đồ đã xử lý vào file JSON.
        
        Args:
            processed_map: Bản đồ đã được xử lý
            filename: Tên file để lưu (optional)
        
        Returns:
            bool: True nếu thành công, False nếu lỗi
        """
        if not filename:
            map_type = processed_map.get('map_info', {}).get('mapType', 'unknown')
            if map_type == 'map_z':
                filename = OUTPUT_CONFIG['map_files']['sample']
            elif map_type == 'map_a':
                filename = OUTPUT_CONFIG['map_files']['problem_a']
            elif map_type == 'map_b':
                filename = OUTPUT_CONFIG['map_files']['problem_b']
            else:
                filename = f"{map_type}.json"
        
        try:
            with open(filename, 'w', encoding=OUTPUT_CONFIG['encoding']) as f:
                json.dump(processed_map, f, indent=OUTPUT_CONFIG['indent'], ensure_ascii=False)
            logger.info(f" Đã lưu bản đồ vào file: {filename}")
            return True
        except Exception as e:
            logger.error(f" Lỗi khi lưu file: {e}")
            return False
    
    def create_networkx_graph(self, processed_map: Dict[str, Any]) -> Optional[nx.DiGraph]:
        """
        Tạo NetworkX graph từ dữ liệu bản đồ đã xử lý.
        Sử dụng DiGraph (đồ thị có hướng) vì các edge có hướng cụ thể.
        
        Args:
            processed_map: Bản đồ đã được xử lý
        
        Returns:
            Optional[nx.DiGraph]: Graph hoặc None nếu lỗi
        """
        if not processed_map:
            logger.error(" Không có dữ liệu để tạo graph")
            return None
        
        try:
            # Tạo đồ thị có hướng
            G = nx.DiGraph()
            
            # Thêm nodes với thuộc tính
            for node in processed_map['nodes']:
                G.add_node(node['id'], **node)
            
            # Thêm edges với thuộc tính (bao gồm cả hướng)
            for edge in processed_map['edges']:
                G.add_edge(
                    edge['source'], 
                    edge['target'], 
                    label=edge['label'],
                    direction=edge['label']
                )
            
            logger.info(f" Đã tạo graph: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
            
            # Kiểm tra tính liên thông
            if nx.is_weakly_connected(G):
                logger.info(" Graph liên thông yếu (có thể đi từ bất kỳ node nào đến node khác)")
            else:
                logger.warning(" Graph không liên thông hoàn toàn")
                
            return G
            
        except Exception as e:
            logger.error(f" Lỗi khi tạo NetworkX graph: {e}")
            return None

class PathFinder:
    """
    Lớp tìm đường đi ngắn nhất sử dụng NetworkX.
    """
    
    def __init__(self, processed_map: Dict[str, Any]):
        """
        Khởi tạo PathFinder.
        
        Args:
            processed_map: Bản đồ đã được xử lý
        """
        self.processed_map = processed_map
        self.graph = None
        self._create_graph()
    
    def _create_graph(self):
        """Tạo NetworkX graph nội bộ."""
        handler = MapAPIHandler()
        self.graph = handler.create_networkx_graph(self.processed_map)
    
    def find_shortest_path(self, start_node: int = None, end_node: int = None) -> Optional[List[int]]:
        """
        Tìm đường đi ngắn nhất sử dụng thuật toán Dijkstra.
        
        Args:
            start_node: Node bắt đầu (optional, sẽ lấy từ map nếu không có)
            end_node: Node kết thúc (optional, sẽ lấy từ map nếu không có)
        
        Returns:
            Optional[List[int]]: Danh sách các node trong đường đi hoặc None nếu không tìm thấy
        """
        if not self.graph:
            logger.error("Graph chưa được tạo")
            return None
        
        # Lấy start và end node từ map nếu không được cung cấp
        map_info = self.processed_map.get('map_info', {})
        
        if start_node is None:
            start_node = map_info.get('start_node')
            logger.info(f"Sử dụng start node từ map: {start_node}")
        
        if end_node is None:
            end_nodes = map_info.get('end_nodes', [])
            if end_nodes:
                end_node = end_nodes[0]  # Lấy end node đầu tiên
                logger.info(f" Sử dụng end node từ map: {end_node}")
        
        if start_node is None or end_node is None:
            logger.error(f"Thiếu start_node ({start_node}) hoặc end_node ({end_node})")
            return None
        
        # Lấy tên nodes để log
        nodes_dict = {n['id']: n['name'] for n in self.processed_map['nodes']}
        start_name = nodes_dict.get(start_node, f"Node_{start_node}")
        end_name = nodes_dict.get(end_node, f"Node_{end_node}")
        
        logger.info(f"Tìm đường từ {start_name}({start_node}) đến {end_name}({end_node})")
        
        try:
            # Tìm đường đi ngắn nhất bằng Dijkstra
            path = nx.shortest_path(self.graph, start_node, end_node)
            
            logger.info(f"Tìm thấy đường đi: {len(path)} nodes")
            
            # Log chi tiết đường đi với tên
            path_names = [f"{nodes_dict.get(n, f'Node_{n}')}({n})" for n in path]
            logger.info(f" Đường đi: {' → '.join(path_names)}")
            
            # Tính và log hướng đi
            self._log_path_directions(path)
            
            return path
            
        except nx.NetworkXNoPath:
            logger.error(f" Không tìm thấy đường đi từ {start_name}({start_node}) đến {end_name}({end_node})")
            return None
        except Exception as e:
            logger.error(f" Lỗi khi tìm đường đi: {e}")
            return None
    
    def _log_path_directions(self, path: List[int]):
        """
        Log chi tiết hướng đi cho từng bước.
        
        Args:
            path: Danh sách các node trong đường đi
        """
        if len(path) < 2:
            return
            
        logger.info(" CHI TIẾT HƯỚNG ĐI:")
        
        nodes_dict = {n['id']: n['name'] for n in self.processed_map['nodes']}
        direction_names = {'N': 'Bắc', 'E': 'Đông', 'S': 'Nam', 'W': 'Tây'}
        
        for i in range(len(path) - 1):
            current = path[i]
            next_node = path[i + 1]
            
            # Tìm edge và hướng
            direction = None
            for edge in self.processed_map['edges']:
                if edge['source'] == current and edge['target'] == next_node:
                    direction = edge['label']
                    break
            
            if direction:
                current_name = nodes_dict.get(current, f"Node_{current}")
                next_name = nodes_dict.get(next_node, f"Node_{next_node}")
                direction_vn = direction_names.get(direction, direction)
                
                logger.info(f"   Bước {i+1}: {current_name} → [{direction}({direction_vn})] → {next_name}")
    
    def find_all_paths(self, start_node: int = None, end_node: int = None, cutoff: int = 10) -> List[List[int]]:
        """
        Tìm tất cả các đường đi có thể từ start đến end.
        
        Args:
            start_node: Node bắt đầu
            end_node: Node kết thúc
            cutoff: Độ dài tối đa của đường đi
        
        Returns:
            List[List[int]]: Danh sách các đường đi
        """
        if not self.graph:
            return []
        
        map_info = self.processed_map.get('map_info', {})
        
        if start_node is None:
            start_node = map_info.get('start_node')
        
        if end_node is None:
            end_nodes = map_info.get('end_nodes', [])
            if end_nodes:
                end_node = end_nodes[0]
        
        try:
            all_paths = list(nx.all_simple_paths(self.graph, start_node, end_node, cutoff=cutoff))
            logger.info(f"Tìm thấy {len(all_paths)} đường đi khả thi")
            return all_paths
        except Exception as e:
            logger.error(f" Lỗi khi tìm tất cả đường đi: {e}")
            return []

def get_and_process_map(map_type: str = "map_z") -> Tuple[Optional[Dict], Optional[PathFinder]]:
    """
    Hàm tiện ích để lấy và xử lý bản đồ trong một lần gọi.
    
    Args:
        map_type: Loại bản đồ cần lấy
    
    Returns:
        Tuple[Optional[Dict], Optional[PathFinder]]: (processed_map, path_finder)
    """
    handler = MapAPIHandler()
    
    logger.info("="*60)
    logger.info(f" BẮT ĐẦU XỬ LÝ BẢN ĐỒ: {map_type}")
    logger.info("="*60)
    
    # Lấy dữ liệu từ API
    raw_map = handler.get_map_from_api(map_type)
    if not raw_map:
        return None, None
    
    # Xử lý dữ liệu
    processed_map = handler.process_map_data(raw_map)
    if not processed_map:
        return None, None
    
    # Lưu vào file
    handler.save_map_to_file(processed_map)
    
    # Tạo PathFinder
    path_finder = PathFinder(processed_map)
    
    logger.info("="*60)
    logger.info(" HOÀN THÀNH XỬ LÝ BẢN ĐỒ")
    logger.info("="*60)
    
    return processed_map, path_finder