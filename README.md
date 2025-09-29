# FPT Hackathon 2025 - Team Watchers

## Luồng hoạt động của chương trình điều khiển xe Jetbot

### 1. Điều kiện tiên quyết

Để code điều khiển xe chạy đúng, cần **cả 2 yếu tố sau**:

1. **API bản đồ (ảo)**

   * BTC cung cấp 1 API trả về thông tin bản đồ dưới dạng JSON (có ví dụ trong file `map_z.py` trong repo *watchers*).
   * API này cho biết: danh sách các điểm (node), đường đi (edge), điểm xuất phát (Start), điểm kết thúc (End).
   * Dựa vào đó, xe sẽ tính toán và chọn **đường đi ngắn nhất** từ Start → End.

2. **Bản đồ vật lý (thực tế)**

   * Đây là sa hình mà BTC đặt trên mặt đất, bao gồm:

     * Đường line đen để xe bám theo.
     * Cột mốc (flag) đánh dấu giao lộ.
     * Mã QR ghi tên từng giao lộ.
     * Biển báo/các item khác tuỳ bài thi (A, B, C).
   * Bản đồ có **4 hướng cố định**: Đông (E), Tây (W), Nam (S), Bắc (N).

👉 Nếu thiếu **API** hoặc **bản đồ thật**, code sẽ không làm xe chạy được. Và 2 thứ trên phải ăn khớp với nhau cùng số đỉnh, cố cạnh, cùng hướng, cùng đỉnh xuất phát, cùng đỉnh kết thúc

---

### 2. Cách thao tác với xe

Xe Jetbot bản chất là một máy tính mini chạy **Ubuntu 18**. Có nhiều cách điều khiển:

* **Cách 1:** Cắm trực tiếp màn hình, chuột, bàn phím vào xe → đăng nhập với user `jetbot` / password `jetbot`.
* **Cách 2 (khuyến nghị):** Dùng laptop SSH vào xe:

  1. Cho xe và laptop kết nối **cùng mạng WiFi**.
  2. Lấy IP của xe (hiển thị trên màn hình OLED gắn trên xe hoặc xem trong giao diện WiFi).
  3. Dùng lệnh:

     ```bash
     ssh jetbot@<ip_cua_jetbot>
     ```

  * Toàn bộ thao tác nên thực hiện qua SSH để **không cần cắm màn hình/chuột/phím** vào xe, ngoại trừ bước lấy IP hoặc kết nối WiFi ban đầu.

---

### 3. Chuẩn bị trước khi chạy

1. Đặt xe tại **điểm Start** trên bản đồ thật (tương ứng với điểm start trên dữ liệu nhận từ API).

   * Đầu xe phải quay về **hướng Đông (E)**.
2. Từ laptop SSH vào xe, clone code về:

   ```bash
   git clone https://github.com/vvtruong27/watchers.git
   ```

---

### 4. Các lệnh cần chạy

Sau khi vào repo, cần mở **3 cửa sổ terminal riêng biệt**, lần lượt chạy:

1. **Mở camera:**

   ```bash
   roslaunch jetbot_pro csi_camera.launch
   ```
2. **Mở LiDAR:**

   ```bash
   roslaunch jetbot_pro lidar.launch
   ```
3. **Chạy code chính:**

   ```bash
   cd watchers/solutions
   python3 problem_a.py
   ```

> ⚠️ Thứ tự quan trọng: phải bật **camera và LiDAR trước**, rồi mới chạy code chính.
> Mỗi lệnh mở ở **một terminal riêng** (đều SSH từ laptop vào xe).

---

### 5. Luồng hoạt động chính

Sau khi chạy lệnh
 ```bash
python3 problem_a.py
``` 
thì chương trình sẽ chạy:

1. Xe gọi API để lấy dữ liệu bản đồ JSON.
2. Xe phân tích và tính toán đường đi ngắn nhất từ Start → End.
3. Xe sử dụng **camera + LiDAR** để bám line, nhận diện QR/biển báo trên bản đồ thật.
4. Xe di chuyển theo kế hoạch đến đích, thực hiện thêm nhiệm vụ tuỳ từng bài (A, B, hoặc C).

---

### 6. Luồng hoạt động chi tiết của xe sau khi chạy code


#### **Bước 1. Gọi API lấy bản đồ và xác định đường đi**

Xe gửi yêu cầu đến API của BTC để lấy dữ liệu bản đồ dưới dạng JSON.

Từ JSON này, xe sẽ:
- Biết vị trí Start (điểm xuất phát) và End (điểm kết thúc).
- Tính toán đường đi ngắn nhất từ Start → End.
- Lưu lại danh sách các giao lộ cần đi qua và hướng đi tại mỗi bước.

**Ví dụ:** Đường đi từ A → D là:
- A → B (hướng Đông - E)
- B → C (hướng Nam - S)
- C → D (hướng Đông - E)

#### **Bước 2. Xác định vị trí ban đầu**

- Xe được đặt tại giao lộ A (Start) trên bản đồ vật lý, đầu xe quay về hướng Đông (E).
- Lúc này, xe sẽ dùng **LiDAR** để tìm 2 flag đặt chéo đối diện tại giao lộ. Khi thấy được 2 flag, xe hiểu rằng mình đang đứng ở một giao lộ.
- Trên terminal, chương trình hiện prompt hỏi:
  ```
  Ready to start? (y/N)
  ```
- Khi bạn nhập `y`, xe sẽ xoay camera về hướng **Đông-Bắc (NE)** (Hướng có mã QR chứa tên giao lộ) để quét mã QR.
- Nếu QR trả về đúng tên giao lộ A, xe xác nhận mình đang ở Start.

#### **Bước 3. Di chuyển sang đỉnh tiếp theo**

- Từ A → B, cần đi hướng E.
- Vì xe vừa quét QR ở hướng NE, nên nó sẽ xoay trở lại hướng E.
- Xe bắt đầu bám line đen để đi thẳng.
- Trong quá trình đi, LiDAR liên tục quét để tìm flag. Khi phát hiện một giao lộ mới, xe dừng lại.

#### **Bước 4. Xác định giao lộ và kiểm tra đúng điểm**

- Sau khi lidar phát hiện giao lộ, xe dừng, xe sẽ xoay sang hướng NE để quét QR của giao lộ vừa gặp.
- Nếu QR cho biết giao lộ này đúng là B (nơi cần đến), xe xác nhận đã tới đỉnh B.

#### **Bước 5. Chọn hướng tiếp theo**

- Từ B → C, hướng đi cần là S (Nam).
- Sau khi quét xong QR ở hướng NE, xe xoay lại hướng S và tiếp tục bám line để đi thẳng.
- Trong quá trình di chuyển, LiDAR vẫn quét liên tục để tìm giao lộ.

#### **Bước 6. Lặp lại tại các giao lộ**

- Khi gặp giao lộ tiếp theo, xe lại dừng, quay sang NE, quét QR.
- Nếu đúng giao lộ là C (giao lộ mục tiêu kế tiếp), xe xác nhận và xoay về hướng E để đi tới D (điểm End).

#### **Bước 7. Kết thúc hành trình**

- Trong quá trình đi tới điểm D, LiDAR tiếp tục tìm giao lộ.
- Khi trên đường hướng đến giao lộ đích là D thì xe sẽ không quét mã QR, nếu trước đó đã đến C và xe đã quay sang hướng đến D thì chỉ cần bám line đi thẳng và nếu lidar phát hiện giao lộ thì dừng hẳn luôn
- Chương trình kết thúc.

#### **Tóm tắt nguyên tắc hoạt động**

- **Di chuyển:** Xe luôn đi theo line đen, điều hướng bằng camera + LiDAR.
- **Phát hiện giao lộ:** LiDAR tìm flag để biết đang ở nút giao.
- **Xác định tên giao lộ:** Xe luôn quay sang hướng NE để quét QR.
- **Ra quyết định:** Nếu đúng giao lộ mong muốn, xe xoay lại đúng hướng đi tiếp theo trong lộ trình.
- **Kết thúc:** Khi tới đúng giao lộ đích (End), xe dừng lại hoàn toàn.

---

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
