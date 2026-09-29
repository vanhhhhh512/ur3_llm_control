# ur3_llm_control

Bài thực hành 02 — điều khiển UR3e bằng câu lệnh ngôn ngữ tự nhiên qua LLM và
skill-based planning, trên ROS 2 Humble + MoveIt 2 + Ignition Gazebo Fortress.

| | |
|---|---|
| Sinh viên | Đỗ Việt Anh |
| MSSV | 23020719 |
| P = 19 mod 6 | **1** |
| Nhiệm vụ cá nhân | Zone A ← red_cube, Zone B ← blue_cube, Zone C ← yellow_cube |
| Video demo | *(cập nhật link Google Drive)* |

---

## 1. Kiến trúc

```
Câu lệnh ngôn ngữ tự nhiên  ("Đưa khối màu đỏ vào vùng A")
        │
        ▼
LLM Planner          bo_lap_ke_hoach_llm.py   ── 9Router ──► LLM
        │                                        (endpoint tương thích OpenAI)
        ▼
JSON Plan  (CHƯA đáng tin)
        │
        ▼
Plan Validator       kiem_tra_ke_hoach.py     ── sai một điểm là từ chối cả kế hoạch
        │
        ▼
Skill Executor       bo_thuc_thi.py           ── dừng ngay ở bước đầu tiên hỏng
        │
        ▼
Robot Skills         ky_nang_robot.py         ── home / pick / place
        │
        ▼
MoveIt 2             giao_tiep_moveit.py      ── /move_action, /compute_ik, planning scene
        │
        ▼
UR3e trong Gazebo    joint_trajectory_controller
```

**Ranh giới an toàn:** LLM chỉ được chọn và sắp xếp skill. Nó không bao giờ sinh
giá trị khớp, toạ độ Descartes, quỹ đạo hay lệnh controller — mọi quỹ đạo đều do
`ky_nang_robot.py` dựng từ `config/scene.yaml` rồi giao cho MoveIt lập kế hoạch
có kiểm tra va chạm. Nếu LLM trả về bất kỳ trường lạ nào (ví dụ `"joint": [...]`),
validator phát hiện và từ chối toàn bộ kế hoạch.

## 2. Cấu trúc thư mục

| Đường dẫn | Vai trò |
|---|---|
| `launch/workcell.launch.py` | Dựng Gazebo + UR3e + controllers + MoveIt + RViz |
| `launch/llm_robot.launch.py` | Chạy node nhận câu lệnh ngôn ngữ tự nhiên |
| `ur3_llm_control/nut_dieu_khien.py` | Node chính, nối toàn bộ luồng xử lý |
| `ur3_llm_control/bo_lap_ke_hoach_llm.py` | Gọi LLM qua 9Router, trích JSON |
| `ur3_llm_control/kiem_tra_ke_hoach.py` | Kiểm tra kế hoạch, fail-closed |
| `ur3_llm_control/bo_thuc_thi.py` | Chạy từng skill, in bảng trạng thái |
| `ur3_llm_control/ky_nang_robot.py` | Ba skill home / pick / place |
| `ur3_llm_control/giao_tiep_moveit.py` | Lớp mỏng bọc service & action của MoveIt |
| `ur3_llm_control/mo_hinh_workcell.py` | Đọc scene.yaml, giữ trạng thái workcell |
| `ur3_llm_control/nhiem_vu_sinh_vien.py` | Tính P = MSSV mod 6 → bảng phân công |
| `ur3_llm_control/dong_bo_gazebo.py` | Cho vật bám theo tool0 trong Gazebo |
| `ur3_llm_control/hien_thi_rviz.py` | Marker workcell và kế hoạch trên RViz |
| `ur3_llm_control/danh_muc.py` | Danh mục skill/object/zone dùng chung |
| `config/scene.yaml` | Toạ độ bàn, vật thể, vùng đặt, tham số chuyển động |
| `config/student_config.yaml` | Tên, MSSV, bảng quy ước 6 trường hợp của P |
| `config/llm.yaml` | Cấu hình 9Router (khoá đọc từ biến môi trường) |
| `config/ur3_controllers.yaml` | Controller, nới ngưỡng bám quỹ đạo cho mô phỏng |
| `prompt/planner_system_prompt.txt` | System prompt gửi cho LLM |
| `worlds/ur3_workcell.sdf` | World Gazebo: bàn, 3 khối, 3 vùng đặt |
| `rviz/workcell.rviz` | Cấu hình RViz kèm panel nút Next |
| `scripts/kiem_thu_ky_nang.py` | Kiểm thử tầng skill, **không** gọi LLM |
| `scripts/chay_container.sh` | Mở shell trong container đã có sẵn ROS |

## 3. Robot Skills

| Skill | Các chặng | Trạng thái trả về |
|---|---|---|
| `home()` | Về tư thế khớp `tu_the_home` | `SUCCESS`, `FAILED`, `PLANNING_FAILED` |
| `pick(object)` | tới phía trên vật → hạ xuống → gắn vào tool0 → nâng lên | thêm `INVALID_OBJECT` |
| `place(object, zone)` | mang tới vùng → hạ xuống → thả → rút lên | thêm `INVALID_ZONE` |

Gripper mô phỏng theo kiểu **giác hút**: khi gắp, khối được gắn vào `tool0` dưới
dạng `AttachedCollisionObject` (MoveIt tính va chạm cho cả khối đang cầm), đồng
thời vị trí khối trong Gazebo được cập nhật bám theo `tool0` ở 25 Hz nên nhìn
thấy vật đi theo tay máy.

Mọi quỹ đạo đều do MoveIt lập với kiểm tra va chạm bật, nên robot không vượt giới
hạn khớp, không tự va chạm và không đâm vào bàn.

## 4. Môi trường mô phỏng

Bàn thao tác cao 0.15 m đặt trước robot. Ba khối lập phương cạnh 45 mm và ba vùng
đặt 100 × 100 mm, toạ độ trong hệ `base_link` (đơn vị mét):

| Vật thể | Vị trí ban đầu | | Vùng đặt | Tâm vùng |
|---|---|---|---|---|
| red_cube | (0.36, −0.15, 0.1725) | | zone_a | (0.26, −0.15, 0.15) |
| yellow_cube | (0.36, 0.00, 0.1725) | | zone_b | (0.26, 0.00, 0.15) |
| blue_cube | (0.36, +0.15, 0.1725) | | zone_c | (0.26, +0.15, 0.15) |

## 5. Cá nhân hoá theo MSSV

`nhiem_vu_sinh_vien.py` không viết cứng kết quả của một sinh viên: nó lấy hai chữ
số cuối của `student_id`, tính `P = XX mod 6`, rồi tra bảng quy ước đầy đủ 6
trường hợp trong `config/student_config.yaml`. Đổi MSSV là đổi nhiệm vụ ngay.

```
MSSV 23020719  →  19 mod 6 = 1  →  zone_a ← red_cube, zone_b ← blue_cube, zone_c ← yellow_cube
```

Bảng này được chèn vào hội thoại dưới dạng một system message nhãn
`ASSIGNMENT TABLE`, nên khi người dùng nói *"Arrange all objects according to my
student ID"* thì LLM dùng đúng bảng do chương trình tính, không tự suy diễn.

## 6. Cài đặt và chạy

Cần ROS 2 Humble, MoveIt 2, Ignition Fortress và các gói chính thức của
Universal Robots (`ur_description`, `ur_moveit_config`, `ur_simulation_gz`).

```bash
mkdir -p ~/ur3_llm_ws/src && cd ~/ur3_llm_ws/src
git clone <URL repo này> ur3_llm_control
cd ~/ur3_llm_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ur3_llm_control
source install/setup.bash
```

### Kết nối LLM qua 9Router

```bash
docker run -d --name 9router -p 20128:20128 \
  -v "$HOME/.9router:/app/data" -e DATA_DIR=/app/data \
  -e INITIAL_PASSWORD='doi-mat-khau-cua-ban' decolua/9router:latest
```

Mở `http://localhost:20128`, vào **Providers** bật một provider (ví dụ
`OpenCode Free` — không cần đăng nhập), vào **API Keys** tạo khoá, rồi:

```bash
export NINEROUTER_BASE_URL=http://127.0.0.1:20128/v1
export NINEROUTER_API_KEY=sk-...            # khoá vừa tạo
export NINEROUTER_MODEL=oc/muse-spark-1.3-contributor-free
```

### Chạy

Terminal 1 — dựng workcell:

```bash
ros2 launch ur3_llm_control workcell.launch.py
```

Terminal 2 — điều khiển bằng ngôn ngữ tự nhiên:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

Gõ câu lệnh rồi Enter. Thêm `cho_nut_bam:=true` để robot dừng trước mỗi skill
cho tới khi bấm **Next** trên panel *RvizVisualToolsGui* — tiện khi quay video.

Chạy một câu lệnh rồi thoát:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py lenh:="Đưa khối màu đỏ vào vùng A"
```

## 7. Ví dụ kết quả

```
USER COMMAND:
  Put the red cube in zone A.

LLM PLAN:
  pick(red_cube)
  place(red_cube, zone_a)
  home()

EXECUTION:
  pick(red_cube) ............. SUCCESS
  place(red_cube, zone_a) .... SUCCESS
  home() ..................... SUCCESS

TASK SUCCESS
```

Kế hoạch không hợp lệ bị chặn trước khi robot nhúc nhích:

```
PLAN REJECTED:
  - buoc 1: skill 'weld' khong nam trong danh sach cho phep

TASK REJECTED, robot khong di chuyen
```

## 8. Ghi chú kỹ thuật

Một số vấn đề gặp trong quá trình làm và cách xử lý:

- **`PATH_TOLERANCE_VIOLATED`, sai số khớp đúng 6.283 rad.** Bộ giải IK trả về
  nghiệm lệch đúng một vòng 2π so với vị trí hiện tại — cùng tư thế vật lý nhưng
  controller thấy sai số 360°. Xử lý bằng `quy_ve_gan_nhat()`: quy mọi mục tiêu
  khớp về nghiệm tương đương gần vị trí hiện tại nhất.
- **Tách lập kế hoạch và thực thi làm hai bước gây lệch trạng thái.** Chuyển sang
  gọi `/move_action` một lần cho cả hai, để MoveIt tự dùng trạng thái hiện tại.
- **IK trả về nghiệm vươn ngược ra sau lưng** (`shoulder_pan = π`). Mồi IK bằng
  `shoulder_pan = atan2(y, x)` của điểm đích.
- **Thả vật xong là mọi kế hoạch đều hỏng.** Lệnh `REMOVE` trên một
  `AttachedCollisionObject` được MoveIt hiểu là "gỡ ra rồi trả về thế giới ngay
  tại đầu kẹp", nên robot lập tức bị coi là đang va chạm. Phải xoá hẳn khỏi thế
  giới rồi thêm lại sau khi tay máy đã rút lên.
- **Hộp va chạm của bàn trùm lên đế robot.** Trong MoveIt chỉ mô tả mặt bàn bằng
  một tấm mỏng thay vì nguyên khối bàn.
- **Quỹ đạo Cartesian chạy hết tốc độ.** `GetCartesianPath` bỏ qua hệ số tỉ lệ
  nếu không truyền `max_velocity_scaling_factor`.
- **OMPL thất bại ngẫu nhiên.** Thử lại nhiều hạt giống IK, nhiều lần gọi
  `/move_action`, và ghé qua một tư thế trung chuyển khi đường trực tiếp bí.

## 9. Kiểm thử

Tách riêng tầng robot khỏi tầng LLM để gỡ lỗi:

```bash
python3 scripts/kiem_thu_ky_nang.py --kich-ban co_ban     # 1 vật
python3 scripts/kiem_thu_ky_nang.py --kich-ban nang_cao   # 3 vật theo MSSV
```

Script này nạp thẳng kế hoạch có sẵn, **không** gọi LLM. Đường chạy chính của bài
tập luôn đi qua LLM.

## 10. Kết quả chạy thử đã đo

Chạy trên ROS 2 Humble + Ignition Fortress, LLM `oc/muse-spark-1.3-contributor-free`
qua 9Router.

**Hiểu câu lệnh** — 9 câu thử, đúng cả 9:

| Câu lệnh | Kế hoạch LLM sinh ra |
|---|---|
| Dua khoi mau do vao vung A | pick(red_cube) → place(red_cube, zone_a) → home() |
| Hay lay khoi mau vang va dat no vao o C | pick(yellow_cube) → place(yellow_cube, zone_c) → home() |
| Move the blue cube to zone B | pick(blue_cube) → place(blue_cube, zone_b) → home() |
| Please put the red cube in zone A. | pick(red_cube) → place(red_cube, zone_a) → home() |
| Arrange all objects according to my student ID | 7 bước theo bảng P = 1 |
| Sap xep tat ca cac vat theo ma so sinh vien cua toi | 7 bước theo bảng P = 1 |
| Han khoi mau tim vao vung Z | **từ chối** |
| Dat khop so 3 bang 1.57 radian | **từ chối** |
| Lay cai do kia | **từ chối** |

**Mức cơ bản** — `TASK SUCCESS`, 3/3 skill.

**Mức nâng cao** — `Arrange all objects according to my student ID`, `TASK SUCCESS`, 7/7 skill:

```
LLM PLAN:
  pick(red_cube)
  place(red_cube, zone_a)
  pick(blue_cube)
  place(blue_cube, zone_b)
  pick(yellow_cube)
  place(yellow_cube, zone_c)
  home()

EXECUTION:
  pick(red_cube) ................ SUCCESS
  place(red_cube, zone_a) ....... SUCCESS
  pick(blue_cube) ............... SUCCESS
  place(blue_cube, zone_b) ...... SUCCESS
  pick(yellow_cube) ............. SUCCESS
  place(yellow_cube, zone_c) .... SUCCESS
  home() ........................ SUCCESS

TASK SUCCESS
```

Trạng thái workcell cuối: red_cube ở zone_a, blue_cube ở zone_b, yellow_cube ở
zone_c — đúng bảng phân công của MSSV 23020719.
