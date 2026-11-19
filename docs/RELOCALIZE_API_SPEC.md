# Đặc Tả Kỹ Thuật API Tự Định Vị (Self-Relocalization API Specification)

Tài liệu này cung cấp toàn bộ hướng dẫn giao tiếp REST API từ **Web Kiosk & Gateway (`radxa_llm_web_v1`)** xuống **Robot Core Daemon (`robot_agent.py :5001`)** để xây dựng giao diện "Tự Định Vị Lại Khi Bị Bê / Mất Vị Trí".

---

## 1. Tổng Quan Kiến Trúc

- **Daemon Host:** `http://127.0.0.1:5001` (hoặc IP SBC từ mạng LAN)
- **CORS:** Đã kích hoạt `Access-Control-Allow-Origin: *`
- **Cơ chế hoạt động:**
  1. Khi người dùng bấm **`[🔄 Tự Định Vị 360°]`**: Daemon kích hoạt `RelocalizeManager` cho xe xoay tròn tại chỗ $360^\circ$ với tốc độ an toàn ($0.35$ rad/s $\approx 20^\circ$/s trong ~18 giây). LiDAR quét toàn bộ các góc tường xung quanh để SLAM Toolbox / Nav2 tự động hội tụ vị trí chính xác.
  2. Khi người dùng bấm **`[Chọn vị trí đặt: Quầy Lễ Tân / Trạm Sạc / Sofa...]`**: Web gửi lệnh 1 chạm xuống API, daemon phát ngay tin nhắn `PoseWithCovarianceStamped` tới `/set_pose` để lập tức khóa vị trí mà không cần kéo thả mũi tên phức tạp.

---

## 2. Danh Sách Endpoint REST API

### 2.1. Lấy Trạng Thái Tự Định Vị
* **Endpoint:** `GET /api/robot/relocalize/status`
* **Mục đích:** Web UI gọi định kỳ (polling mỗi 500ms - 1000ms khi đang tự định vị) để cập nhật thanh tiến trình (progress bar) và thông báo trạng thái.
* **Response `200 OK`:**
```json
{
  "state": "running",
  "progress": 45,
  "message": "Đang xoay quét LiDAR tìm vị trí (45%)...",
  "is_running": true
}
```
* **Các giá trị của `state`:**
  - `"idle"`: Đang rảnh rỗi, sẵn sàng nhận lệnh.
  - `"running"`: Đang trong quá trình tự xoay quét LiDAR tìm vị trí.
  - `"completed"`: Đã hoàn tất thành công 100%.
  - `"cancelled"`: Người dùng đã bấm dừng/hủy.
  - `"error"`: Gặp lỗi trong quá trình thực thi.

---

### 2.2. Bắt Đầu / Dừng / Đặt Vị Trí Ban Đầu
* **Endpoint:** `POST /api/robot/relocalize`
* **Headers:** `Content-Type: application/json`

#### a) Bắt đầu tự xoay 360° tìm vị trí:
* **Request Body:**
```json
{
  "action": "start"
}
```
* **Response `200 OK`:**
```json
{
  "success": true,
  "message": "Đã bắt đầu tiến trình tự định vị 360°.",
  "status": {
    "state": "running",
    "progress": 0,
    "message": "Bắt đầu xoay 360° quét LiDAR khớp bản đồ...",
    "is_running": true
  }
}
```

#### b) Dừng / Hủy khẩn cấp:
* **Request Body:**
```json
{
  "action": "stop"
}
```
* **Response `200 OK`:**
```json
{
  "success": true,
  "message": "Đã gửi lệnh dừng tự định vị."
}
```

#### c) Đặt vị trí 1 chạm (Set Initial Pose nhanh):
* **Request Body:**
```json
{
  "action": "set_pose",
  "pose": [0.0, 0.0, 0.0]
}
```
*(Ghi chú: `pose` là mảng `[x, y, yaw]` tính theo mét và radian).*
* **Response `200 OK`:**
```json
{
  "success": true,
  "pose": [0.0, 0.0, 0.0],
  "message": "Đã phát /set_pose tại (0.0, 0.0, 0.0)"
}
```

---

### 2.3. Lấy Danh Sách Điểm Neo Quen Thuộc (Anchors)
* **Endpoint:** `GET /api/robot/anchors`
* **Mục đích:** Lấy danh sách các nút bấm khu vực (Quầy lễ tân, Trạm sạc, v.v.) để render lên UI cho người dùng chọn nhanh.
* **Response `200 OK`:**
```json
{
  "success": true,
  "anchors": [
    {
      "id": "reception",
      "name": "Quầy Lễ Tân",
      "x": 0.0,
      "y": 0.0,
      "yaw": 0.0
    },
    {
      "id": "dock",
      "name": "Trạm Sạc",
      "x": -0.5,
      "y": 0.0,
      "yaw": 0.0
    },
    {
      "id": "entrance",
      "name": "Cửa Ra Vào",
      "x": 2.0,
      "y": 0.0,
      "yaw": 0.0
    },
    {
      "id": "living_area",
      "name": "Khu Sofa Khách",
      "x": 1.5,
      "y": 1.5,
      "yaw": 1.57
    }
  ]
}
```

---

## 3. Hướng Dẫn Tích Hợp Frontend (Dành Cho Agent `radxa_llm_web_v1`)

### 3.1. Thiết Kế Giao Diện Đề Xuất (UI / UX)
```
+-------------------------------------------------------------+
|               🤖 ĐỊNH VỊ LẠI VỊ TRÍ ROBOT                   |
+-------------------------------------------------------------+
|  [ 🔄 TỰ XOAY TÌM VỊ TRÍ ]      [ ⏹ DỪNG ]                 |
|                                                             |
|  Tiến độ: [██████████████░░░░░░░░░░] 55%                    |
|  Trạng thái: Đang xoay quét LiDAR tìm vị trí (55%)...       |
+-------------------------------------------------------------+
|  Hoặc chọn nhanh vị trí bạn vừa đặt robot:                 |
|  [ 🏢 Quầy Lễ Tân ]  [ ⚡ Trạm Sạc ]                        |
|  [ 🚪 Cửa Ra Vào ]   [ 🛋 Khu Sofa ]                        |
+-------------------------------------------------------------+
```

### 3.2. Code Mẫu JavaScript / Vue.js / React

```javascript
const API_BASE = "http://127.0.0.1:5001";

// 1. Kích hoạt tự xoay định vị
async function startAutoRelocalize() {
  const res = await fetch(`${API_BASE}/api/robot/relocalize`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "start" })
  });
  const data = await res.json();
  if (data.success) {
    pollRelocalizeStatus();
  }
}

// 2. Polling tiến độ
function pollRelocalizeStatus() {
  const interval = setInterval(async () => {
    const res = await fetch(`${API_BASE}/api/robot/relocalize/status`);
    const status = await res.json();
    
    updateUIProgressBar(status.progress);
    updateUIMessage(status.message);

    if (status.state === "completed" || status.state === "cancelled" || status.state === "error") {
      clearInterval(interval);
      if (status.state === "completed") {
        alert("Định vị thành công!");
      }
    }
  }, 800);
}

// 3. Chọn nhanh điểm neo (1-tap anchor)
async function selectQuickAnchor(anchor) {
  const res = await fetch(`${API_BASE}/api/robot/relocalize`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      action: "set_pose",
      pose: [anchor.x, anchor.y, anchor.yaw]
    })
  });
  const data = await res.json();
  alert(data.message);
}
```
