# Hướng Dẫn Vận Hành & Kiểm Thử Hệ Thống OTA (OTA_GUIDE.md)

Hệ thống **OTA Backend Engine** cho Receptionist Robot (`radxa_amr_v1`) cung cấp quy trình cập nhật phần mềm từ xa an toàn, mượt mà và tự động rollback nếu xảy ra sự cố.

---

## 1. Kiến Trúc Bộ Nhớ & Dữ Liệu Bền Vững (Persistent Data)

Để tránh mất mát bản đồ hoặc cấu hình khi nâng cấp phiên bản, toàn bộ dữ liệu động được lưu tại:
* **Bản đồ SLAM (`.data`, `.posegraph`, `.yaml`, `.pgm`):** `/home/rctrobot/robot_data/maps/`
* **Điểm Home (`home_pose.json`):** `/home/rctrobot/robot_data/home_pose.json`
* **Bản đồ active gần nhất:** `/home/rctrobot/robot_data/last_saved_map.txt`
* **Trạng thái phiên bản OTA:** `/home/rctrobot/ota/ota_state.json`

Thư mục `/home/rctrobot/radxa_amr_v1/map` là một symlink trỏ trực tiếp sang `/home/rctrobot/robot_data/maps`, đảm bảo tương thích ngược 100% với các package ROS 2 cũ.

---

## 2. Các Endpoint REST API (Port 5001)

### `GET /api/ota/status`
Trả về trạng thái thời gian thực của OTA State Machine.
```json
{
  "success": true,
  "current_version": "v1.0.0",
  "previous_version": null,
  "stage": "IDLE",
  "progress": 0,
  "message": "Sẵn sàng",
  "error": null,
  "is_busy": false,
  "free_disk_mb": 7160,
  "last_updated": null
}
```

### `POST /api/ota/check`
Kiểm tra kết nối mạng và đối chiếu phiên bản mới nhất từ GitHub Releases:
```bash
curl -X POST http://127.0.0.1:5001/api/ota/check
```
Phản hồi mẫu:
```json
{
  "success": true,
  "has_update": true,
  "current_version": "v1.0.0",
  "latest_version": "v1.1.0",
  "release_notes": "Cập nhật thuật toán điều hướng Nav2...",
  "download_url": "https://github.com/.../radxa_amr_v1.1.0_arm64.tar.gz",
  "sha256": "8a35..."
}
```

### `POST /api/ota/apply`
Bắt đầu tiến trình cập nhật bất đồng bộ (chạy trên thread riêng):
```bash
curl -X POST http://127.0.0.1:5001/api/ota/apply \
  -H "Content-Type: application/json" \
  -d '{
    "version": "v1.1.0",
    "download_url": "https://...",
    "sha256": "8a35..."
  }'
```

### `POST /api/ota/rollback`
Chủ động kích hoạt hoàn nguyên phiên bản trước đó:
```bash
curl -X POST http://127.0.0.1:5001/api/ota/rollback
```

---

## 3. Quy Trình Vận Hành Tự Động (OTA State Machine)
1. **CHECKING_SAFETY (5%):** Kiểm tra robot có đang di chuyển không. Nếu `/cmd_vel` > 0 hoặc đang chạy Nav2, lập tức từ chối và giữ an toàn.
2. **DOWNLOADING (10% $\rightarrow$ 60%):** Tải file `.tar.gz` vào `/home/rctrobot/ota/staging/` theo dạng stream, liên tục cập nhật phần trăm tiến trình cho Web.
3. **VERIFYING (70%):** So khớp mã băm SHA-256 với checksum mong đợi. Nếu sai, xóa ngay file đệm và báo lỗi.
4. **EXTRACTING (80%):** Giải nén vào `/home/rctrobot/ota/releases/amr_vX.Y.Z/`.
5. **SWITCHING_LINK (85%):** Sử dụng atomic symlink (`mv -Tf`) trỏ bản mới, gắn kết nối bản đồ sang `/home/rctrobot/robot_data/maps`.
6. **RESTARTING (90%):** Gọi `sudo systemctl restart robot_bringup.service`.
7. **HEALTHCHECK (95%):** Watchdog lắng nghe trong tối đa 45 giây để xác nhận:
   - Dịch vụ `robot_bringup.service` ở trạng thái `active`.
   - ESP32 micro-ROS hoạt động và có thông điệp truyền về trên `/odom`.
8. **COMPLETED (100%):** Ghi nhận phiên bản mới, tự động xóa các bản cũ (chỉ giữ lại 2 bản gần nhất để duy trì bộ nhớ đệm < 1GB).
9. **ROLLING_BACK $\rightarrow$ ROLLED_BACK (nếu thất bại):** Tự động đảo ngược symlink về bản cũ và khởi động lại dịch vụ nếu quá 45 giây không đạt health check.
