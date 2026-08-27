# Kế hoạch triển khai: LiDAR-native Endpoint (Phase 1 & Phase 2)

> **Phương pháp đã chốt (chat gần nhất)**  
> - **Camera**: chỉ **trigger** giơ tay + lọc đứng yên (không đo góc/mét tới người).  
> - **LiDAR**: nguồn duy nhất cho **endpoint** `(x, y)` trên `map` (θ, r native).  
> - **Zone m×n**: mọi vị trí trong zone đều hợp lệ (không ưu tiên “trên làn”).  
> - **Quỹ đạo**: đi thẳng tới goal (đã có `straight_line_controller`).  
> - **Phase 3** (queue ai vẫy trước) và **case che nắng** = ngoài phạm vi file này; chỉ để slot sẵn nếu cần.

---

## 0. Mục tiêu & ranh giới

| Phase | Mục tiêu | Không làm trong phase này |
|-------|----------|---------------------------|
| **Phase 1** | 1 người trong zone + giơ tay → endpoint LiDAR chính xác → `/person_goal` | Multi-track, L/C/R, queue |
| **Phase 2** | Nhiều người → track LiDAR + association thô khi vẫy → lock track → endpoint | Queue FIFO, phân biệt che nắng vs vẫy |

### Nguyên tắc cứng (mọi task phải tuân thủ)

1. Không dùng shoulder-width / mono depth làm nguồn mét cho endpoint.  
2. Không dùng góc suy từ pixel/bbox làm `(θ, r)` endpoint.  
3. Endpoint chỉ từ cụm/track LiDAR + TF lên `map`.  
4. Camera chỉ: `hand_raised`, `motion_ok`, và (Phase 2) hint `left|center|right`.

### Dependency tổng

```text
P1-T01 → P1-T02 → … → P1-T20 (Phase 1 done)
                         ↓
              P2-T01 → … → P2-T18 (Phase 2 done)
```

### Cấu trúc thư mục đề xuất (tạo dần theo task)

```text
src/yolov11_pose_detector/
  yolov11_pose_detector/
    lidar_endpoint/
      __init__.py
      zone.py
      scan_utils.py
      cluster.py
      endpoint.py
      tracker.py          # Phase 2
      associator.py       # Phase 2
      types.py
    pose_detector_node.py           # giữ trigger; gọi endpoint engine
    straight_line_controller.py     # giữ nguyên (đã có)
  config/
    service_zone.yaml
    lidar_endpoint.yaml
  test/
    test_zone.py
    test_cluster.py
    ...
```

---

# PHASE 1 — Single person, LiDAR latch @ trigger

---

## P1-T01 — Tạo package module skeleton `lidar_endpoint`

| | |
|---|---|
| **Input** | Package `yolov11_pose_detector` hiện có |
| **Output** | Thư mục `yolov11_pose_detector/lidar_endpoint/` + `__init__.py` rỗng export được |
| **Phương pháp** | Tạo package Python con; không đổi logic runtime. Verify: `from yolov11_pose_detector.lidar_endpoint import ...` sau khi thêm export. |
| **Done khi** | Import không lỗi; chưa có thuật toán. |

---

## P1-T02 — Định nghĩa types / dataclass dùng chung

| | |
|---|---|
| **Input** | Phác thảo cấu trúc chat (ServiceZone, LidarPoint2D, LidarCluster, TriggerEvent, PersonEndpoint, GoalPose) |
| **Output** | File `lidar_endpoint/types.py` với dataclass + type hints |
| **Phương pháp** | Dùng `@dataclass`; mỗi field ghi rõ frame (`laser` / `map`). Không phụ thuộc ROS msg trong types thuần (giữ test được). |
| **Done khi** | Unit test tạo được instance mẫu; không import `rclpy`. |

---

## P1-T03 — Config YAML: `service_zone.yaml` (placeholder)

| | |
|---|---|
| **Input** | Yêu cầu zone hình chữ nhật m×n trên `map` |
| **Output** | `config/service_zone.yaml` với `x_min, y_min, x_max, y_max, frame_id: map` (placeholder số) |
| **Phương pháp** | Viết YAML; comment hướng dẫn đo 4 góc trên RViz / map. Chưa cần tọa độ thật. |
| **Done khi** | File tồn tại; schema ổn định. |

---

## P1-T04 — Config YAML: `lidar_endpoint.yaml` (params Phase 1)

| | |
|---|---|
| **Input** | Bảng params Phase 1 (eps, n_min, width, range, safety_distance, gesture K/N, v_max) |
| **Output** | `config/lidar_endpoint.yaml` |
| **Phương pháp** | Gom mọi magic number Phase 1 vào YAML; đặt default an toàn. |
| **Done khi** | Mọi hằng số thuật toán Phase 1 đọc từ config, không hardcode trong logic (trừ test). |

---

## P1-T05 — Implement `ServiceZone.contains`

| | |
|---|---|
| **Input** | `types.ServiceZone` + tọa độ `(x, y)` map |
| **Output** | `zone.py`: `load_zone_from_dict/yaml`, `contains(x, y) -> bool` |
| **Phương pháp** | AABB: `x_min ≤ x ≤ x_max` và `y_min ≤ y ≤ y_max`. Optional epsilon biên. |
| **Done khi** | Unit test: điểm trong/ngoài/biên pass. |

---

## P1-T06 — Scan → danh sách điểm 2D (laser frame)

| | |
|---|---|
| **Input** | `sensor_msgs/LaserScan` (hoặc mock: ranges, angle_min, angle_increment) |
| **Output** | `scan_utils.py`: `laser_scan_to_points(scan) -> list[LidarPoint2D]` (frame=`laser`) |
| **Phương pháp** | Bỏ `inf/nan`; giữ `range_min ≤ r ≤ range_max`; `x=r*cos(θ)`, `y=r*sin(θ)`. |
| **Done khi** | Unit test với scan giả (1 tia phía trước) cho đúng `(x,y)`. |

---

## P1-T07 — TF điểm laser → map

| | |
|---|---|
| **Input** | `list[LidarPoint2D]` (laser) + TF `map ← laser` (hoặc `map ← base_link ← laser`) |
| **Output** | `scan_utils.py`: `transform_points_to_map(points, tf) -> list[LidarPoint2D]` (frame=`map`) |
| **Phương pháp** | Dùng `tf2` lookup; áp dụng rotation+translation từng điểm. Timeout ngắn; fail → return empty + flag. |
| **Done khi** | Test với TF identity / rotation 90° mock; không cần robot chạy. |

---

## P1-T08 — Lọc điểm nằm trong ServiceZone

| | |
|---|---|
| **Input** | Points frame=`map` + `ServiceZone` |
| **Output** | `filter_points_in_zone(points, zone) -> list[LidarPoint2D]` |
| **Phương pháp** | List comprehension `zone.contains`. |
| **Done khi** | Unit test: 10 điểm, chỉ giữ điểm trong AABB. |

---

## P1-T09 — Euclidean clustering (jump distance)

| | |
|---|---|
| **Input** | Points in zone (map hoặc laser — thống nhất 1 frame, khuyến nghị **map**) + `cluster_eps` |
| **Output** | `cluster.py`: `cluster_points(points, eps) -> list[LidarCluster]` |
| **Phương pháp** | Sort theo góc hoặc dùng DBSCAN đơn giản / jump: nếu khoảng cách tới điểm kế > eps → cụm mới. Tính centroid, n_points, width (max-min theo hướng vuông góc với tia trung bình hoặc max pairwise ≈ diameter), mean_range. |
| **Done khi** | Unit test: 2 cụm tách rõ → 2 clusters; 1 cụm dính → 1 cluster. |

---

## P1-T10 — Human-like cluster filter

| | |
|---|---|
| **Input** | `list[LidarCluster]` + params `n_min, width_min/max, r_min/max` |
| **Output** | `filter_human_clusters(clusters, params) -> list[LidarCluster]` |
| **Phương pháp** | Hard filter theo ngưỡng; không dùng camera. |
| **Done khi** | Unit test: cụm tường dài / nhiễu 1 điểm bị loại; cụm kích thước người giữ lại. |

---

## P1-T11 — Phase 1 cluster selector (0 / 1 / >1)

| | |
|---|---|
| **Input** | Filtered clusters |
| **Output** | `select_phase1_cluster(clusters) -> (cluster|None, status)` với `status ∈ {ok, empty, ambiguous}` |
| **Phương pháp** | `len==0 → empty`; `len==1 → ok`; `len>1 → ambiguous` (Phase 1: **không** đoán gần nhất — reject để tránh đi nhầm; log WARN). |
| **Done khi** | Unit test 3 nhánh status. |

---

## P1-T12 — Safety goal từ person + robot pose

| | |
|---|---|
| **Input** | `P_person (x,y)`, `P_robot (x,y)`, `safety_distance` |
| **Output** | `endpoint.py`: `compute_safety_goal(...) -> GoalPose | AlreadyThere` |
| **Phương pháp** | `d = ‖P_p - P_r‖`; nếu `d ≤ safety + ε` → AlreadyThere; else `goal = P_p - d_safe * (P_p-P_r)/d`; `yaw = atan2(...)`. **Không** chiếu lên roadmap. |
| **Done khi** | Unit test: người lệch trái/phải → goal trên đoạn thẳng robot–người; khoảng cách tới người ≈ `safety_distance`. |

---

## P1-T13 — EMA smooth endpoint

| | |
|---|---|
| **Input** | Chuỗi `P_person` theo thời gian + `alpha` / window |
| **Output** | `EndpointSmoother` class: `update(x,y) -> (x_s,y_s)`; `reset()` |
| **Phương pháp** | EMA hoặc mean N frame; reject nếu nhảy `> jump_max` so với smoothed trước. |
| **Done khi** | Unit test: nhiễu nhỏ bị làm mượt; spike bị reject. |

---

## P1-T14 — Latch scan buffer quanh thời điểm trigger

| | |
|---|---|
| **Input** | Stream `/scan` + timestamp `t*` khi trigger ON |
| **Output** | `ScanLatch`: giữ ring buffer N scan; `get_scans_around(t*, dt) -> list[LaserScan]` |
| **Phương pháp** | Deque maxlen; khi trigger lấy 1–3 scan gần `t*` nhất; có thể gộp points rồi cluster 1 lần (median centroid sau). |
| **Done khi** | Unit test buffer với stamp giả. |

---

## P1-T15 — `Phase1EndpointEngine` (orchestrator thuần, không ROS spin)

| | |
|---|---|
| **Input** | Zone, params, scan(s), robot pose map, TriggerEvent |
| **Output** | `Phase1EndpointEngine.compute(...) -> PersonEndpoint | None` + reason string |
| **Phương pháp** | Ghép T06–T14 theo pipeline: points → TF map → zone → cluster → filter → select → smooth → PersonEndpoint. |
| **Done khi** | Integration unit test (mock scan + TF identity): 1 cụm trong zone → endpoint đúng. |

---

## P1-T16 — Tách trigger camera: publish `TriggerEvent` nội bộ

| | |
|---|---|
| **Input** | Logic `_is_arm_raised` + gesture buffer hiện có trong `pose_detector_node.py` |
| **Output** | Hàm/API nội bộ: `get_trigger_event() -> TriggerEvent` (`hand_raised`, `motion_ok` tạm True nếu chưa có motion, `stamp`) |
| **Phương pháp** | **Không** đổi điều kiện giơ tay; chỉ đóng gói kết quả. Chưa tính che nắng. Motion gate có thể stub `motion_ok=True` rồi làm T17. |
| **Done khi** | Khi giơ tay, node có TriggerEvent rõ; chưa đổi cách tính goal cũ nếu chưa wire T18. |

---

## P1-T17 — Motion gate đứng yên (bbox velocity)

| | |
|---|---|
| **Input** | Tâm bbox (hoặc centroid keypoints) theo frame + dt |
| **Output** | `motion_ok: bool` trong TriggerEvent |
| **Phương pháp** | `|Δcenter|/Δt < v_bbox_max` trên N frame; hoặc ước lượng từ track đơn giản. Người đi/chạy → `motion_ok=False` → không trigger endpoint. |
| **Done khi** | Test với chuỗi bbox đứng yên / dịch nhanh. |

---

## P1-T18 — Wire Phase 1 engine vào `pose_detector_node` (thay endpoint cũ)

| | |
|---|---|
| **Input** | `TriggerEvent` ON + `/scan` + TF + Phase1EndpointEngine |
| **Output** | Khi trigger: publish `/person_goal` từ LiDAR engine; **không** gọi `_fuse_lidar_distance` / shoulder-width cho goal |
| **Phương pháp** | Flag param `use_lidar_native_endpoint: true`. Giữ path cũ khi false (rollback). Status ARRIVED vẫn qua `/straight_nav/status`. |
| **Done khi** | Param true → log “lidar_native”; goal không phụ thuộc vai. |

---

## P1-T19 — Launch / bringup: load config zone + lidar_endpoint

| | |
|---|---|
| **Input** | `service_zone.yaml`, `lidar_endpoint.yaml`, `bringup.launch.py` |
| **Output** | Node YOLO / endpoint nhận path params; install data_files trong `setup.py` |
| **Phương pháp** | Thêm `data_files` share config; launch truyền đường dẫn YAML. |
| **Done khi** | `ros2 launch` thấy param zone được load (ros2 param get). |

---

## P1-T20 — Kiểm thử thực địa Phase 1 (acceptance)

| | |
|---|---|
| **Input** | Robot + 1 người trong zone, nhiều vị trí (trái / giữa / phải / xa / gần), giơ tay |
| **Output** | Báo cáo checklist pass/fail + tune params |
| **Phương pháp** | RViz: Marker cụm + `/person_goal`; đo khoảng dừng ≈ safety; người đi bộ không trigger; >1 cụm human-like → không đi (ambiguous). |
| **Done khi** | Checklist Phase 1 signed-off (mục § Acceptance Phase 1 bên dưới). |

---

### Acceptance Phase 1

- [ ] Người đứng lệch trái/phải trong zone + giơ tay → robot đi thẳng tới gần đúng vị trí (sai số chấp nhận được, vd ≤ 0.3–0.5 m).  
- [ ] Không giơ tay → không publish goal mới.  
- [ ] Đi bộ trong zone → không trigger (motion gate).  
- [ ] Endpoint không dùng `person_avg_shoulder_width`.  
- [ ] Ambiguous (>1 cụm) → không tự chọn “gần nhất”.  

---

# PHASE 2 — Multi-person LiDAR tracks + coarse association

> **Điều kiện bắt đầu:** Phase 1 acceptance pass (P1-T20).

---

## P2-T01 — Mở rộng `types.py` cho Phase 2

| | |
|---|---|
| **Input** | Phác thảo: PersonTrack, ImageSideHint, AssociationResult, LockedTarget, WaveEvent (slot Phase 3) |
| **Output** | Dataclass bổ sung trong `types.py` |
| **Phương pháp** | Thêm fields; WaveEvent chỉ định nghĩa, chưa queue logic. |
| **Done khi** | Import OK; test tạo track mẫu. |

---

## P2-T02 — Config params Phase 2 trong YAML

| | |
|---|---|
| **Input** | `v_stand_max`, `assoc_gate`, `age_min`, `miss_max`, `side_angle_α`, `T_lost`, `dt_sync` |
| **Output** | Section `phase2:` trong `lidar_endpoint.yaml` |
| **Phương pháp** | Tách rõ phase1/phase2 keys. |
| **Done khi** | Loader đọc được dict phase2. |

---

## P2-T03 — Kalman filter 2D Constant Velocity (thuần)

| | |
|---|---|
| **Input** | Measurement `(x,y)`, dt |
| **Output** | `tracker.py`: class `KalmanCV2D` — `predict(dt)`, `update(x,y)` → state `(x,y,vx,vy)` |
| **Phương pháp** | KF đơn giản 4-state; Q,R cấu hình được. Không ROS. |
| **Done khi** | Unit test: đo nhiễu → vận tốc ước lượng hội tụ hướng đúng. |

---

## P2-T04 — Tạo / cập nhật / xóa `PersonTrack`

| | |
|---|---|
| **Input** | Cluster measurements + existing tracks |
| **Output** | Track lifecycle: `create`, `update`, `mark_miss`, `is_dead` |
| **Phương pháp** | `age_frames++`, `hits++` khi match; `misses++` khi không; dead nếu `misses > miss_max`. `standing = speed < v_stand_max`. |
| **Done khi** | Unit test lifecycle. |

---

## P2-T05 — Data association clusters ↔ tracks (nearest / Hungarian)

| | |
|---|---|
| **Input** | Tracks predict + clusters centroids |
| **Output** | Matching pairs + unmatched tracks/clusters; gate `assoc_gate` |
| **Phương pháp** | Cost = Euclidean; greedy nearest hoặc `scipy.optimize.linear_sum_assignment` nếu có scipy; nếu không → greedy. Cost > gate = không match. |
| **Done khi** | Unit test: 2 track / 2 cluster không bắt chéo khi xa nhau. |

---

## P2-T06 — `LidarPersonTracker.update(scan_points_in_zone)`

| | |
|---|---|
| **Input** | Points in zone mỗi chu kỳ |
| **Output** | `list[PersonTrack]` publishable |
| **Phương pháp** | cluster → filter human → associate → KF update → prune. Chạy **liên tục**, không chỉ lúc vẫy. |
| **Done khi** | Simulation: người giả di chuyển → track id ổn định ≥ N frame. |

---

## P2-T07 — Topic debug `/lidar_people/tracks`

| | |
|---|---|
| **Input** | list[PersonTrack] |
| **Output** | Publish `geometry_msgs/PoseArray` hoặc MarkerArray (id trong ns/text) |
| **Phương pháp** | RViz hiển thị; frame=`map`. |
| **Done khi** | RViz thấy nhiều track khi nhiều người đứng trong zone. |

---

## P2-T08 — ImageSideHint từ bbox đang vẫy (thô)

| | |
|---|---|
| **Input** | Bbox `(u1,u2)` hoặc center_u + image_width |
| **Output** | `ImageSideHint.side ∈ {left, center, right}` (+ optional area_rank) |
| **Phương pháp** | Chia ảnh 3 vùng theo `u` (vd 0–0.33 / 0.33–0.67 / 0.67–1). **Không** tính bearing rad cho endpoint. |
| **Done khi** | Unit test 3 vùng. |

---

## P2-T09 — Side của track theo LiDAR + robot yaw

| | |
|---|---|
| **Input** | Track `(x,y)`, robot `(x,y,yaw)`, `side_angle_α` |
| **Output** | `side_lidar ∈ {left, center, right}` |
| **Phương pháp** | `bearing = atan2(dy,dx) - yaw` normalize; so với ±α. Góc này từ **TF + LiDAR position**, không từ camera. |
| **Done khi** | Unit test robot nhìn +X, track trái/phải. |

---

## P2-T10 — `TriggerAssociator.associate(trigger, tracks, side_hint)`

| | |
|---|---|
| **Input** | TriggerEvent `t*`, tracks, optional ImageSideHint |
| **Output** | `AssociationResult` |
| **Phương pháp** | Lọc `standing ∧ in_zone ∧ age≥age_min ∧ |stamp-t*|≤dt_sync`; nếu 1 → unique_standing; nếu >1 → filter side_hint khớp side_lidar; nếu còn >1 → score thô (gần robot hơn + area_rank); nếu 0 → reject. |
| **Done khi** | Unit test: unique / side / reject. |

---

## P2-T11 — `LockedTarget` lifecycle

| | |
|---|---|
| **Input** | AssociationResult ok + track_id |
| **Output** | Lock state; unlock khi ARRIVED / cancel / lost > `T_lost` |
| **Phương pháp** | Trong lock: endpoint **chỉ** lấy từ `tracker.get(track_id)`; không re-associate mỗi frame trừ khi lost. |
| **Done khi** | Unit test lock/unlock/lost. |

---

## P2-T12 — Endpoint liên tục từ locked track

| | |
|---|---|
| **Input** | Locked track state `(x,y)` |
| **Output** | Cập nhật `/person_goal` (hoặc giữ goal cố định lần đầu — chọn 1 policy và ghi rõ) |
| **Phương pháp** | **Policy đề xuất Phase 2:** latch goal lúc lock (ổn định hơn); optional re-publish nếu person dịch < threshold. Document trong YAML `goal_update_policy: latch|follow`. |
| **Done khi** | Param chọn được; default `latch`. |

---

## P2-T13 — Wire Phase 2 vào node (param `endpoint_phase: 1|2`)

| | |
|---|---|
| **Input** | Phase1 engine + Tracker + Associator |
| **Output** | Node chạy phase 2 khi param=2; phase 1 khi =1 |
| **Phương pháp** | Tracker update mỗi scan; chỉ associate khi TriggerEvent cạnh lên (rising edge). |
| **Done khi** | Đổi param không cần sửa code path cứng. |

---

## P2-T14 — Publish `/lidar_people/locked` + association debug

| | |
|---|---|
| **Input** | LockedTarget, AssociationResult |
| **Output** | Topics debug (String / Int32 track_id) |
| **Phương pháp** | Log method: `unique_standing` / `side_hint` / `reject`. |
| **Done khi** | `ros2 topic echo` thấy khi vẫy tay. |

---

## P2-T15 — Rising-edge trigger (tránh spam associate)

| | |
|---|---|
| **Input** | TriggerEvent stream |
| **Output** | Chỉ associate 1 lần mỗi lần bắt đầu giơ tay (hoặc cooldown) |
| **Phương pháp** | `prev_trigger` → edge; cooldown `goal_cooldown_sec` tái sử dụng. |
| **Done khi** | Giơ tay giữ lâu không đổi lock liên tục. |

---

## P2-T16 — Slot Phase 3: struct `WaveEvent` (không implement queue)

| | |
|---|---|
| **Input** | Locked association thành công |
| **Output** | Optional log/publish WaveEvent `{track_id, t_wave, x, y}` — **không** FIFO serve |
| **Phương pháp** | Chỉ để sẵn interface; comment “Phase 3”. |
| **Done khi** | Event xuất hiện trong log/topic debug. |

---

## P2-T17 — Integration test giả lập 2–3 người

| | |
|---|---|
| **Input** | Fake clusters + fake trigger + side hint |
| **Output** | Pytest scenarios |
| **Phương pháp** | (1) 1 đứng 2 đi → lock đúng người đứng; (2) 2 đứng trái/phải + hint left → lock trái; (3) 2 đứng cùng side → reject hoặc score — assert hành vi đã chọn. |
| **Done khi** | CI/local pytest xanh. |

---

## P2-T18 — Kiểm thử thực địa Phase 2 (acceptance)

| | |
|---|---|
| **Input** | ≥2 người trong zone |
| **Output** | Checklist pass/fail |
| **Phương pháp** | Cases: chỉ 1 người vẫy đứng; 2 đứng khác phía; người đi không bị chọn; mất track → cancel. |
| **Done khi** | Acceptance Phase 2 signed-off. |

---

### Acceptance Phase 2

- [ ] Tracks ổn định trên RViz khi nhiều người trong zone.  
- [ ] 1 người đứng vẫy, người khác đi → lock đúng người đứng.  
- [ ] 2 người đứng trái/phải, vẫy bên trái → tới bên trái (endpoint từ LiDAR track).  
- [ ] Không dùng góc pixel làm mét goal.  
- [ ] Mất track > T_lost → dừng/cancel an toàn.  

---

# Thứ tự thực hiện khuyến nghị (sprint)

| Sprint | Tasks | Kết quả |
|--------|-------|---------|
| S1 | P1-T01 … P1-T05 | Skeleton + zone + config |
| S2 | P1-T06 … P1-T11 | Pipeline cluster thuần |
| S3 | P1-T12 … P1-T15 | Engine + goal + smooth + latch |
| S4 | P1-T16 … P1-T20 | Wire ROS + field test Phase 1 |
| S5 | P2-T01 … P2-T07 | Tracker + RViz |
| S6 | P2-T08 … P2-T15 | Association + lock + wire |
| S7 | P2-T16 … P2-T18 | Slot queue + tests + field |

---

# Định nghĩa xong 1 task

Mỗi task **Done** khi:

1. Có **Input/Output** đúng bảng trên.  
2. Có code hoặc test tương ứng (trừ task acceptance).  
3. Không phá nguyên tắc cứng §0.  
4. Task sau chỉ bắt đầu khi task trước Done (trừ task config/docs song song trong cùng sprint).

---

# Ngoài phạm vi (ghi nhớ, không làm trong plan này)

- Queue FIFO “ai vẫy trước” (Phase 3).  
- Phân biệt vẫy tay vs che nắng.  
- Đổi HW RGB-D / stereo.  
- Bật lại Nav2 Dijkstra/RPP cho serving (giữ straight-line).  
- Ưu tiên người “trên làn đường”.

---

# Checklist nhanh trước khi code P1-T18

- [ ] TF `map → laser` ổn định (SLAM + URDF).  
- [ ] Zone placeholder đã thay bằng tọa độ map thật (hoặc tạm zone rất rộng để test).  
- [ ] `/person_goal` vẫn được `straight_line_controller` subscribe.  
- [ ] Param rollback `use_lidar_native_endpoint: false` còn hoạt động.

---

*Tài liệu này phản ánh phương pháp LiDAR-native + camera-trigger đã thống nhất trong chat. Cập nhật file khi thay đổi policy (ví dụ ambiguous Phase 1, goal latch vs follow).*
