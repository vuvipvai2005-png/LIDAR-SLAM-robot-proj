import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import atexit
import heapq
import time
from scipy.spatial import cKDTree
from scipy.ndimage import binary_erosion, binary_dilation
from matplotlib.patches import Circle
from matplotlib.ticker import MultipleLocator
# ==========================================
# Cấu hình
# ==========================================
ROBOT_RADIUS = 0.13
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
ESP32_IP = "192.168.1.4"
ESP32_PORT = 12346
MAX_POINTS = 180

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def cleanup():
    sock.close()
    
atexit.register(cleanup)
print(f" Khởi động phần mềm SLAM: {UDP_IP}:{UDP_PORT}")

MAP_SIZE   = 600
RESOLUTION = 0.05
origin     = MAP_SIZE // 2

# ==========================================
#  Xác suất công dồn đối với mỗi loại ô
# ==========================================
L_OCC  = np.log(0.75 / 0.25)   # +1.099  (vùng có vật chắn)
L_FREE = np.log(0.25 / 0.75)   # -1.099  (vùng trống)
L_FREE_WEAK = np.log(0.38 / 0.62) # ~ -0.49 (nhẹ hơn, gần tường, chưa chắc chắn)
WALL_PROTECT_THRESH = 2.0      # Ngưỡng tường chắc chắn
# lập bản đồ xác suất sàn (mảng quan trọng nhất)
log_odds_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.float32)

# ==========================================
# Tìm đường bằng thuật toán A* 
# ==========================================
def heuristic(a, b):
    # Ước tính khoảng cách từ node a đến goal b
    # hàm hypot = căn(a^2+b^2)
    # f(n) = g(n) + h(n), trong đó h(n) = heuristic(current, goal)
    return np.hypot(a[0] - b[0], a[1] - b[1])


def is_valid(cell):
    # Kiểm tra cell có nằm trong ranh giới map không
    # Map kích thước 600×600, nên x,y thuộc [0, 599]
    x, y = cell
    return 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE


def is_occupied(cell):
    # Kiểm tra cell có bị chiếm dụng (obstacle hoặc inflated wall) không
    # global_costmap được cập nhật bởi update_costmap()
    # True = không thể đi, False = có thể đi
    x, y = cell
    return global_costmap[y, x]


def is_diagonal_blocked(current, nb):
    """
    Kiểm tra đi chéo có bị chặn bởi góc tường không.
    Ví dụ: đi từ (x,y) sang (x+1,y+1) phải kiểm tra:
    - Ô (x+1,y) không bị chặn
    - Ô (x,y+1) không bị chặn
    Nếu một trong hai bị chặn => không được đi chéo
    """
    x, y = current
    nx, ny = nb

    dx = nx - x
    dy = ny - y

    # Chỉ kiểm tra chuyển động chéo (|dx|==1 và |dy|==1)
    if abs(dx) == 1 and abs(dy) == 1:
        cell1 = (x + dx, y)      # Ô bên cạnh theo hướng X
        cell2 = (x, y + dy)      # Ô bên cạnh theo hướng Y

        if not is_valid(cell1) or not is_valid(cell2):
            return True

        if is_occupied(cell1) or is_occupied(cell2):
            return True

    return False


def get_neighbors(node):
    # Lấy tất cả 8 ô lân cận (4 hướng chính + 4 hướng chéo)
    # Thứ tự: phải, trái, lên, xuống, 4 góc chéo
    x, y = node
    return [
        (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),      # 4 hướng chính
        (x + 1, y + 1), (x - 1, y - 1),                      # 2 góc chéo
        (x + 1, y - 1), (x - 1, y + 1)                       # 2 góc chéo còn lại
    ]


def a_star(start, goal):
    """
    Thuật toán A* để tìm đường đi ngắn nhất từ start đến goal.
    - Dùng open_set (priority queue) để lưu các node cần khám phá
    - Dùng g_score để lưu chi phí từ start đến mỗi node
    - Dùng f = g + h để đánh giá ưu tiên khám phá node nào trước
    - closed_set để tránh khám phá lại node đã xử lý
    """
    
    # Kiểm tra đầu vào trước
    if not is_valid(start):
        print(" START nằm ngoài map!")
        return None

    if not is_valid(goal):
        print(" GOAL nằm ngoài map!")
        return None

    if is_occupied(start):
        print(" START nằm trong vùng vật chắn")
        return None

    if is_occupied(goal):
        print(" GOAL nằm trong vùng vật chắn")
        return None

    # Khởi tạo open_set (heap) với start node
    # Mỗi phần tử: (f_score, position) để ưu tiên node có f nhỏ nhất
    # heap: Cấu trúc dữ liệu dạng cây nhị phân, phần tử nhỏ nhất luôn ở gốc, mảng
    open_set = []
    heapq.heappush(open_set, (0.0, start))
    # Thêm start node vào open_set với f_score = 0.0
    # Heap tự động giữ node có f nhỏ nhất ở đầu
    # Khi heappop sẽ lấy node ưu tiên cao nhất (f nhỏ nhất)
    came_from = {}                  # Lưu parent của mỗi node để backtrack đường đi
    g_score = {start: 0.0}          # Chi phí từ start đến mỗi node
    closed_set = set()              # Node đã được xử lý hoàn toàn

    while open_set:
        # Lấy node có f_score nhỏ nhất (ưu tiên cao nhất)
        _, current = heapq.heappop(open_set)

        # Tránh expand lại node cũ do heap có thể chứa duplicate
        if current in closed_set:
            continue

        # Nếu đạt goal, backtrack để lấy đường đi
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # Đánh dấu node này đã xử lý xong
        closed_set.add(current)

        # Khám phá tất cả ô lân cận
        for nb in get_neighbors(current):
            # Bỏ qua nếu ô nằm ngoài map
            if not is_valid(nb):
                continue

            # Bỏ qua nếu ô đã xử lý
            if nb in closed_set:
                continue

            # Bỏ qua nếu ô bị chiếm dụng
            if is_occupied(nb):
                continue

            # Bỏ qua nếu đi chéo bị chặn bởi góc tường
            if is_diagonal_blocked(current, nb):
                continue

            # Tính chi phí chuyển động:
            # - Chuyển động thẳng (4 hướng): chi phí = 1.0
            # - Chuyển động chéo (4 góc): chi phí = √2 ≈ 1.414
            dx = abs(nb[0] - current[0])
            dy = abs(nb[1] - current[1])
            move_cost = np.sqrt(2) if dx == 1 and dy == 1 else 1.0

            # Tính chi phí tích lũy từ start đến neighbor
            tg = g_score[current] + move_cost

            # Nếu tìm được đường tốt hơn đến neighbor, cập nhật
            if nb not in g_score or tg < g_score[nb]:
                came_from[nb] = current
                g_score[nb] = tg
                # f = g + h, đánh giá tổng chi phí ước tính
                f = tg + heuristic(nb, goal)
                heapq.heappush(open_set, (f, nb))

    return None
# chuyển tọa độ thực và map trên PC
def world_to_map(x, y):
    return int(round(x / RESOLUTION)) + origin, int(round(y / RESOLUTION)) + origin
def map_to_world(mx, my):
    return (mx - origin) * RESOLUTION, (my - origin) * RESOLUTION

# ==========================================
# Cập nhập map
# ==========================================
def make_circular_kernel(radius_cells):
    y, x = np.ogrid[-radius_cells:radius_cells + 1,
                    -radius_cells:radius_cells + 1]
    return x*x + y*y <= radius_cells*radius_cells
# gọi a:b tức a là chỉ số bắt đầu(bao gồm) và b là chỉ số kết thúc(không bao gồm)
def update_costmap():
    global global_costmap

    wall_mask = log_odds_map > 1.5
    free_mask = log_odds_map < -1.0

    inflate_cells = int(np.ceil(ROBOT_RADIUS / RESOLUTION))
    #np.ceil làm tròn lên số thực đến số nguyên nhỏ nhất >= giá trị đó
    kernel = make_circular_kernel(inflate_cells)

    inflated_wall = binary_dilation(wall_mask, structure=kernel)
# binary_dilation (scipy.ndimage) là hàm mở rộng vùng TRUE trong mảng nhị phân bằng cách thêm các pixel xung quanh 
# mỗi một điểm tường trong wall_mask mở rộng thành kernel 
    # ô không biết và ô tường không đi được
    global_costmap = ~free_mask
    global_costmap |= inflated_wall
#global_costmap = mảng 2 chiều T/F
#cập nhập bản đồ xác suất log_odds_map bằng hàm update_map() 
def update_map(scan_pts, robot_x, robot_y):
    global log_odds_map
    rx, ry = world_to_map(robot_x, robot_y)

    all_free_x,      all_free_y      = [], []
    all_free_near_x, all_free_near_y = [], []
    all_wall_x,      all_wall_y      = [], []

    for px, py in scan_pts:
        mx, my = world_to_map(px, py)
        dist = max(abs(mx - rx), abs(my - ry))
        if dist > 0:
            x_vals = np.linspace(rx, mx, dist, endpoint=False).astype(int)
            y_vals = np.linspace(ry, my, dist, endpoint=False).astype(int)
            #VD: np.linspace(0, 10, 5(số giá trị), endpoint=True) sẽ tạo ra mảng [0, 2.5, 5, 7.5, 10], nếu endpoint=False sẽ tạo ra [0, 2, 4, 6, 8]
            #Tạo một đường line từ robot tới điểm LiDAR
            #astype để chuyển đổi kiểu dữ liệu, các số thực trong x_vals thành int 
            valid  = ((x_vals >= 0) & (x_vals < MAP_SIZE) &
                      (y_vals >= 0) & (y_vals < MAP_SIZE))
            xv, yv = x_vals[valid], y_vals[valid]
            
            if len(xv) > 2:
                all_free_x.append(xv[:-2]);      all_free_y.append(yv[:-2])
                all_free_near_x.append(xv[-2:]); all_free_near_y.append(yv[-2:])
                # chỉ chắc chắn ô trống offset tường ra 1 khoảng (2 ô), điểm gần tường không chắc chắn(2 điểm cuối) cho vào all_free_near_x
            else:
                all_free_x.append(xv); all_free_y.append(yv)
                
        if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
            all_wall_x.append(mx); all_wall_y.append(my)

    # L_FREE (xa tường): xóa nhưng bảo vệ ô log_odds >= 2.0
    if all_free_x:
        fx = np.concatenate(all_free_x); fy = np.concatenate(all_free_y)
        # concatenate để gộp, nối nhiều mảng thành một mảng duy nhất
        mask = log_odds_map[fy, fx] < WALL_PROTECT_THRESH
        #nếu một ô có độ chắc chắn là tường >2, coi là tường chắc, không được phép xóa đi
        log_odds_map[fy[mask], fx[mask]] += L_FREE
        # log_odds >> ô đó càng chắc là tường
    # L_FREE nhẹ (sát tường): dùng L_FREE_WEAK, cũng bảo vệ tường
    if all_free_near_x:
        fnx = np.concatenate(all_free_near_x); fny = np.concatenate(all_free_near_y)
        mask_near = log_odds_map[fny, fnx] < WALL_PROTECT_THRESH
        log_odds_map[fny[mask_near], fnx[mask_near]] += L_FREE_WEAK

    # L_OCC tại tường, clamp 4.0
    if all_wall_x:
        wx = np.array(all_wall_x); wy = np.array(all_wall_y)
        log_odds_map[wy, wx] = np.minimum(log_odds_map[wy, wx] + L_OCC, 10.0)

    np.clip(log_odds_map, -5.0, 5.0, out=log_odds_map)

# ==========================================
# Chuyển bản đồ xác suất sang thang độ xám hiển thị (4 loại) 
# ==========================================
def build_display_map():
    # 1:Mảng 600×600 tất cả giá trị = 50 (xám/ chưa xác định)
    display = np.full((MAP_SIZE, MAP_SIZE), 50, dtype=np.uint8)
    
    # 2: Vẽ free space - Các ô có log_odds < -1.0 = chắc chắn trống => đặt thành 0 (trắng)
    display[log_odds_map < -1.0] = 0
    
    # 3: Phân loại tường thành 2 loại:
    # - Tường mờ: log_odds từ 1.5 → 2.5
    # - Tường chắc: log_odds > 2.5 
    wall_faint_mask = (log_odds_map >= 1.5) & (log_odds_map <= 2.5)
    wall_solid_mask = log_odds_map > 2.5
    
    # 4: Làm dày tường - Giãn pixel tường thêm 1 lớp xung quanh
    # nhìn rõ hơn
    if np.any(wall_faint_mask):
        faint_thick = binary_dilation(wall_faint_mask, iterations=1)
        display[faint_thick] = 80  # Tường mờ -> 80 (xám đậm)
        
    if np.any(wall_solid_mask):
        solid_thick = binary_dilation(wall_solid_mask, iterations=1)
        display[solid_thick] = 100 # Tường chắc -> 100 (đen kịt)
    
    # 5: Trả về mảng giá trị 0-100 để hiển thị với colormap gray_r:
    # 0 = Trắng (trống an toàn), 50 = Xám (chưa khám phá), 80 = Xám đậm (tường yếu), 100 = Đen (tường chắc)
    return display
# ==========================================
# NORMALIZE ANGLE
# ==========================================
def normalize_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

# ==========================================
# Thuần công thức ICP, TH robot đi dọc hành lang bị hiện tượng gọi là suy biến
# N/Xét: Khối ICP có công thức phức tạp, TÓM TẮT:
"""
best_fit_transform_weighted(A, B, weights) trả về R2x2, t1x2
check_icp_degeneracy(src_inliers) trả về:
-True nếu phát hiện suy biến, ngược lại
-strong_axis: vector 2D hướng mà ICP có thông tin mạnh nhất 
-degen_ratio: suy biến lớn hay bé
run_icp(source, target, max_iterations=30, tolerance=0.0003 trả về Ma trận tích lũy cuối
cùng R_accum để khớp source với target, t_accum vector tịnh tiến cuối cùng,...
icp_pose_correction(R, t, pred_x, pred_y, pred_theta) fix, hiệu chỉnh tại tọa độ thật của robot
"""
# ==========================================
def best_fit_transform_weighted(A, B, weights):
    w_sum      = np.sum(weights)
    c_A        = np.sum(A * weights[:, None], axis=0) / w_sum # ma trận A nhân với ma trận W được quay theo kiểu từng phần tủ (x1*w1,y1*w1)
    c_B        = np.sum(B * weights[:, None], axis=0) / w_sum
    AA, BB     = A - c_A, B - c_B
    H          = (AA * weights[:, None]).T @ BB
    U, S, Vt   = np.linalg.svd(H)
    # phép quay U
    # phép do giãn S
    # phép quay Vt
    # "@" là toán tử nhân ma trận
    R          = Vt.T @ U.T
    # .T là phép chuyển vị ma trận
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = c_B - R @ c_A
    return R, t

def check_icp_degeneracy(src_inliers):
    # src_inliers là tập hợp các điểm 2D [x,y];
    # cov = [[ Var(X),  Cov(X,Y) ],
    #        [ Cov(Y,X),  Var(Y)  ]] là ma trận hiệp phương sai thể hiẹn hướng và hình dạng dữ liệu

    if len(src_inliers) < 5:
        return False, np.array([1,0]), 1.0
    cov    = np.cov(src_inliers.T)
    vals, vecs = np.linalg.eigh(cov)  # trị riêng và vector riêng
    ratio  = vals[0] / (vals[1] + 1e-9)
    
    # vals[0] là λ_min. Eigenvector của nó (vecs[:, 0]) là pháp tuyến tường (⊥ hành lang).
    # Đây là hướng mà ICP có đủ thông tin (Strong Axis của ICP)
    icp_strong_axis = vecs[:, 0]  
    
    is_degenerate = ratio < 0.15
    return is_degenerate, icp_strong_axis, ratio 

def run_icp(source, target, max_iterations=30, tolerance=0.0003):
    src        = np.copy(source)              # Copy source để không thay đổi dữ liệu gốc
    prev_error = float('inf')                 # Sai số vòng lặp trước (dùng so sánh hội tụ)
    tree       = cKDTree(target)              # Xây KD-Tree target để tìm NN nhanh
    mean_error = float('inf')
    inlier_ratio = 0.0

    R_accum = np.eye(2)                       # Tích lũy phép quay (chuyển từ source sang target trong hệ global)
    t_accum = np.zeros(2)                     # Tích lũy tịnh tiến

    for _ in range(max_iterations):
        distances, indices = tree.query(src)  # Tìm điểm target gần nhất cho mỗi điểm src

        # Tính ngưỡng loại outlier: lấy 65 percentile (75% gần, 25% xa)
        percentile_thr = np.percentile(distances, 65)
        hard_thr       = 0.4                  # Giới hạn cứng (không vượt 40cm)
        threshold      = min(percentile_thr, hard_thr)  # Dùng cái nào bé hơn
        mask           = distances < threshold  # Mặt nạ lọc điểm inlier
        inlier_ratio   = np.sum(mask) / len(distances)  # Tỷ lệ điểm tốt

        if np.sum(mask) < 15:                 # Quá ít inlier -> dừng (không đủ dữ liệu để calibrate)
            break

        src_f    = src[mask]                  # Lọc source inlier
        tgt_f    = target[indices][mask]      # Lọc target inlier tương ứng
        dists_f  = distances[mask]            # Khoảng cách đã lọc

        # Trọng số inversed-distance: khoảng cách gần thì trọng số cao
        weights  = 1.0 / (dists_f + 0.01)
        # Cắt bỏ outlier trọng số (lấy 95 percentile làm mức tối đa) để tránh bias
        weights  = np.clip(weights, 0, np.percentile(weights, 95))

        # Tính phép quay R và tịnh tiến t từ cặp điểm đã lọc + trọng số
        R, t     = best_fit_transform_weighted(src_f, tgt_f, weights)
        src      = src @ R.T + t              # Áp dụng lên tất cả điểm 

        R_accum  = R @ R_accum
        t_accum  = R @ t_accum + t

        mean_error = np.mean(dists_f)         # Sai số trung bình (khoảng cách trung bình)
        
        # Điều kiện hội tụ: sai số không thay đổi đáng kể
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # Kiểm tra tình trạng suy biến của ICP cuối: robot có đi trong hành lang không?
    final_dist, final_idx = tree.query(src)
    final_mask = final_dist < min(np.percentile(final_dist, 65), 0.4)

    if np.sum(final_mask) >= 5:
        # Gọi PCA để phát hiện suy biến (ratio < 0.15) và lấy strong axis (pháp tuyến tường)
        is_degen, strong_axis, degen_ratio = check_icp_degeneracy(src[final_mask])
    else:
        # Quá ít inlier -> coi như không suy biến
        is_degen, strong_axis, degen_ratio = False, None, 1.0

    return R_accum, t_accum, src, mean_error, inlier_ratio, is_degen, strong_axis, degen_ratio

# ==========================================
# Giao diện
# ==========================================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 10), facecolor='#1a1a2e')
ax.set_facecolor('#1a1a2e')
ax.set_aspect('equal')
ax.set_xlim(-6.0, 6.0)
ax.set_ylim(-6.0, 6.0)
ax.xaxis.set_major_locator(MultipleLocator(1.0))
ax.yaxis.set_major_locator(MultipleLocator(1.0))
ax.grid(True, color='#2a2a4a', linestyle='-', linewidth=0.8)

map_extent = [-origin * RESOLUTION, (MAP_SIZE - origin) * RESOLUTION,
              -origin * RESOLUTION, (MAP_SIZE - origin) * RESOLUTION]

occ_map_display = np.full((MAP_SIZE, MAP_SIZE), 50, dtype=np.uint8)
map_img = ax.imshow(occ_map_display, cmap='gray_r', origin='lower',
                    extent=map_extent, vmin=0, vmax=100, interpolation='nearest')

trajectory_plot, = ax.plot([], [], color='#00d4ff', linewidth=1.2, alpha=0.8, label="Trajectory")
lidar_plot,      = ax.plot([], [], '.', color='#00ff88', markersize=2, alpha=0.7, label="Laser Scan")
heading_plot,    = ax.plot([], [], color='#ff6b6b', linewidth=2.5, label="Heading")
robot_circle     = Circle((0, 0), ROBOT_RADIUS, color='#4ecdc4', alpha=0.8, zorder=5)
ax.add_patch(robot_circle)
target_plot, = ax.plot([], [], 'y*', markersize=15, label="Nav Goal")
path_plot,   = ax.plot([], [], color='#ffd700', linewidth=2, alpha=0.9, label="A* Path")

icp_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                   color='#aaaaaa', fontsize=8, va='top', fontfamily='monospace')

ax.legend(loc='upper right', facecolor='#1a1a2e', labelcolor='white', edgecolor='#444', fontsize=8)
ax.tick_params(colors='#888888')
for spine in ax.spines.values():
    spine.set_color('#333333')

# ==========================================
# Biến toàn cục
# ==========================================
angles = np.linspace(0, 358, MAX_POINTS) * np.pi / 180.0  # Góc quay của LiDAR :0-358°, 180 điểm

trajectory_x, trajectory_y = [], []              # Lưu lộ trình di chuyển của robot
last_draw_time    = time.time()                 # Thời điểm vẽ lần trước

corrected_x, corrected_y, corrected_theta = 0.0, 0.0, 0.0  # Vị trí robot hiệu chỉnh (từ ICP + Odometry)
last_rx, last_ry, last_rtheta             = None, None, None  # Vị trí ESP32 lần trước(để tính delta)
kf_x, kf_y, kf_theta                     = 0.0, 0.0, 0.0 

nav_goal_active   = False                  # Cờ: có mục tiêu điều hướng đang hoạt động không?
target_x_slam, target_y_slam = 0.0, 0.0   # Tọa độ mục tiêu click chuột vào
last_goal_send_time = 0                    # Thời gian gửi waypoint liền trước
path_world        = []                     # Danh sách waypoint A* chuyển sang hệ toàn cục
# ==========================================
# Gói tin UDP từ ESP32
# ==========================================
SYNC_SIZE   = 2                                           # Byte đồng bộ (0xAA 0x55)
HEADER_SIZE = 20                                          # Header: timestamp, rx, ry, rtheta, num_points
RANGES_SIZE = MAX_POINTS * 4                             # Dữ liệu khoảng cách (180 điểm × 4 bytes mỗi cái)
PACKET_SIZE = SYNC_SIZE + HEADER_SIZE + RANGES_SIZE + (MAX_POINTS * 2) + 1  # Tổng packet (+ checksum)

# ==========================================
# zoom+click
# ==========================================
def on_scroll(event):
    if event.inaxes != ax:
        return
    base_scale   = 1.2
    cur_xlim     = ax.get_xlim()
    cur_ylim     = ax.get_ylim()
    xdata, ydata = event.xdata, event.ydata
    scale_factor = 1 / base_scale if event.button == 'up' else base_scale
    new_w = (cur_xlim[1] - cur_xlim[0]) * scale_factor
    new_h = (cur_ylim[1] - cur_ylim[0]) * scale_factor
    relx  = (cur_xlim[1] - xdata) / (cur_xlim[1] - cur_xlim[0])
    rely  = (cur_ylim[1] - ydata) / (cur_ylim[1] - cur_ylim[0])
    ax.set_xlim([xdata - new_w * (1 - relx), xdata + new_w * relx])
    ax.set_ylim([ydata - new_h * (1 - rely), ydata + new_h * rely])
    fig.canvas.draw_idle()

def onclick(event):
    global nav_goal_active, target_x_slam, target_y_slam, path_world
    if event.inaxes != ax or event.button != 1:
        return
    trajectory_x.clear()
    trajectory_y.clear()
    target_x_slam, target_y_slam = event.xdata, event.ydata
    print(f"\n Goal: X={target_x_slam:.2f}, Y={target_y_slam:.2f}")

    start_m = world_to_map(corrected_x, corrected_y)
    goal_m  = world_to_map(target_x_slam, target_y_slam)
    print(" Đang chạy A*...")
    path = a_star(start_m, goal_m)

    if path is None:
        print("Không tìm được đường!")
        return
    print(f" Path length: {len(path)}")
    path_world     = [map_to_world(x, y) for (x, y) in path]
    nav_goal_active = True
    target_plot.set_data([target_x_slam], [target_y_slam])
    fig.canvas.draw_idle()

fig.canvas.mpl_connect('scroll_event',       on_scroll)
fig.canvas.mpl_connect('button_press_event', onclick)
def icp_pose_correction(R, t, pred_x, pred_y, pred_theta):
    p_pred = np.array([pred_x, pred_y])
    p_icp = R @ p_pred + t

    dpos = p_icp - p_pred
    dtheta = np.arctan2(R[1, 0], R[0, 0])

    return dpos, dtheta
# ======================================================
# =====================vÒNG LẶP=========================
# ======================================================
while True:
    try:
        data = None
        while True:
            try:
                packet, addr = sock.recvfrom(2048)
                data = packet
            except BlockingIOError:
                break

        if data is None:
            fig.canvas.flush_events()
            time.sleep(0.01)
            continue

        if len(data) != PACKET_SIZE or data[0] != 0xAA or data[1] != 0x55:
            continue
        checksum = 0
        for byte in data[2:-1]:
            checksum ^= byte
        if checksum != data[-1]:
            continue

        header_data = data[SYNC_SIZE: SYNC_SIZE + HEADER_SIZE]
        timestamp, rx, ry, rtheta, num_points = struct.unpack('<IfffHxx', header_data)

        ranges_offset = SYNC_SIZE + HEADER_SIZE
        ranges = np.array(struct.unpack(f'<{MAX_POINTS}f',
                          data[ranges_offset: ranges_offset + RANGES_SIZE]))

        if last_rx is None:
            last_rx, last_ry, last_rtheta   = rx, ry, rtheta
            corrected_x, corrected_y, corrected_theta = rx, ry, rtheta
            kf_x, kf_y, kf_theta            = rx, ry, rtheta

        # ==========================================
        # 1. ODOMETRY PREDICTION
        # ==========================================
        delta_x     = rx - last_rx
        delta_y     = ry - last_ry
        delta_theta = normalize_angle(rtheta - last_rtheta)
        last_rx, last_ry, last_rtheta = rx, ry, rtheta
        # tính sự thay đổi tọa độ (x,y,theta) giữa 2 lần nhận từ esp32
        if abs(delta_x) > 0.5 or abs(delta_y) > 0.5:
            print("LAG!")
            delta_x, delta_y, delta_theta = 0.0, 0.0, 0.0

        pred_x     = corrected_x + delta_x
        pred_y     = corrected_y + delta_y
        pred_theta = normalize_angle(corrected_theta + delta_theta)
        # dự đoán vị trí mới bằng cách dùng tọa độ đã hiệu chỉnh cộng với sự thay đổi trên
        valid_mask  = (ranges > 0.1) & (ranges < 9.0)
        if np.sum(valid_mask) < 15:
            continue
        # lọc, lấy khoảng cách lớn hơn 10cm và nhỏ hơn 9m
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        global_angles = valid_angles + pred_theta
        gx = pred_x + valid_ranges * np.cos(global_angles)
        gy = pred_y + valid_ranges * np.sin(global_angles)
        guess_scan        = np.vstack((gx, gy)).T
        current_draw_scan = guess_scan

        # ==========================================
        # 2. SCAN-TO-MAP ICP
        # ==========================================
        y_idx, x_idx = np.where(log_odds_map > 2.0)
        icp_status = "No map yet"

        if len(x_idx) > 20:
            target_pts_x   = (x_idx - origin) * RESOLUTION
            target_pts_y   = (y_idx - origin) * RESOLUTION
            full_target_map = np.vstack((target_pts_x, target_pts_y)).T

            dist_sq     = ((full_target_map[:, 0] - pred_x) ** 2 +
                           (full_target_map[:, 1] - pred_y) ** 2)
            target_map  = full_target_map[dist_sq < 16.0]

            if len(target_map) > 50:
                target_map = target_map[::2]

                R, t, matched_scan, mean_err, inlier_ratio, is_degen, strong_axis, degen_ratio = run_icp(
                    guess_scan, target_map, max_iterations=30)

                correction_vec, dtheta_icp = icp_pose_correction(
                    R, t, pred_x, pred_y, pred_theta
                )

                dx_icp = correction_vec[0]
                dy_icp = correction_vec[1]

                if mean_err < 0.25 and inlier_ratio > 0.4:
                    # 1. TÍNH ĐỘ LỆCH GIỮA ICP VÀ ODOMETRY
                    # Xem ICP đang muốn kéo robot đi bao xa so với dự đoán của bánh xe
                    icp_shift_dist = np.hypot(dx_icp, dy_icp)
                    
                    # 2. CHỐNG TRƯỢT HÀNH LANG
                    # Nếu ICP đòi kéo robot nhảy quá 4cm trong 1 chớp mắt -> ICP đang ảo giác!
                    # -> TỪ CHỐI ICP, TIN TƯỞNG 100% ODOMETRY
                    if icp_shift_dist > 0.04:
                        corrected_x = pred_x
                        corrected_y = pred_y
                        corrected_theta = pred_theta
                        icp_status = f"ICP SLIP REJECT ({icp_shift_dist*100:.1f}cm) -> Trust Odom"
                        
                    # 3. NẾU ĐANG TRONG HÀNH LANG NHƯNG ĐỘ LỆCH NHỎ
                    elif is_degen and strong_axis is not None:
                        # Giảm sự tin tưởng vào ICP (alpha_xy = 0.3)
                        # Chỉ bù sai số bạt ngang (giúp robot không đâm vào tường)
                        # Chiều dọc (chiều đi) hoàn toàn do Odometry quyết định
                        alpha_xy = 0.3 
                        correction_vec = np.array([dx_icp, dy_icp])
                        proj = np.dot(correction_vec, strong_axis) * strong_axis
                        
                        corrected_x = pred_x + alpha_xy * proj[0]
                        corrected_y = pred_y + alpha_xy * proj[1]
                        corrected_theta = pred_theta # IMU luôn đúng
                        
                        icp_status = f"ICP CORRIDOR err={mean_err:.3f}m"
                        
                    # 4. KHÔNG GIAN XUNG QUANH NHIỀU ĐIỂM NEO, TƯỜNG KHÁC NHAU
                    else:
                        # Tin ICP bình thường nhưng luôn giới hạn alpha để mượt mà
                        alpha_xy = np.clip(0.4 + inlier_ratio * 0.4, 0.4, 0.8)
                        
                        if abs(dtheta_icp) > np.radians(2.5):
                            alpha_th = 0.0  
                            icp_status = f"ICP XY-only (Angle Reject)"
                        else:
                            alpha_th = np.clip(inlier_ratio * 0.1, 0.01, 0.05)
                            icp_status = f"ICP OK err={mean_err:.3f}m"

                        corrected_x     = pred_x + alpha_xy * dx_icp
                        corrected_y     = pred_y + alpha_xy * dy_icp
                        corrected_theta = normalize_angle(pred_theta + alpha_th * dtheta_icp)
                else:
                    corrected_x, corrected_y, corrected_theta = pred_x, pred_y, pred_theta
                    icp_status = (f"ICP SKIP  err={mean_err:.3f}m  "
                                  f"inliers={inlier_ratio*100:.0f}%")
            else:
                corrected_x, corrected_y, corrected_theta = pred_x, pred_y, pred_theta
                icp_status = "ICP: not enough map pts"
        else:
            corrected_x, corrected_y, corrected_theta = pred_x, pred_y, pred_theta
        corrected_angles = valid_angles + corrected_theta
        gx_corr = corrected_x + valid_ranges * np.cos(corrected_angles)
        gy_corr = corrected_y + valid_ranges * np.sin(corrected_angles)
        current_draw_scan = np.vstack((gx_corr, gy_corr)).T
        # ==========================================
        # 3. TRAJECTORY & KEYFRAME
        # ==========================================
        if (len(trajectory_x) == 0 or
                np.hypot(corrected_x - trajectory_x[-1],
                         corrected_y - trajectory_y[-1]) > 0.01):
            trajectory_x.append(corrected_x)
            trajectory_y.append(corrected_y)
        # ghi lại quỹ đạo
        dist_from_kf  = np.hypot(corrected_x - kf_x, corrected_y - kf_y)
        angle_from_kf = abs(normalize_angle(corrected_theta - kf_theta))
        is_map_empty  = np.max(log_odds_map) < 2.0

        if is_map_empty or dist_from_kf > 0.2 or angle_from_kf > 0.3:
            update_map(current_draw_scan, corrected_x, corrected_y)
            update_costmap() # Gọi ngay sau khi map thay đổi để A* nhận đường

            occ_map_display = build_display_map()
            map_img.set_data(occ_map_display)

            kf_x, kf_y, kf_theta = corrected_x, corrected_y, corrected_theta


        # ==========================================
        # 4. NAV GOAL / A* FOLLOWING & RE-PLANNING
        # ==========================================
        current_time = time.time()

        if nav_goal_active and len(path_world) > 0 and (current_time - last_goal_send_time > 0.1):
            last_goal_send_time = current_time

            # check1: XÓA CÁC WAYPOINT BỊ LỠ KHI NÉ VẬT CẢN ---
            if len(path_world) > 1:
                # Chỉ check 10 điểm tiếp theo để tìm điểm gần xe nhất (tiết kiệm CPU)
                search_len = min(10, len(path_world))
                dists = [np.hypot(p[0] - corrected_x, p[1] - corrected_y) for p in path_world[:search_len]]
                closest_idx = int(np.argmin(dists))

                # Nếu điểm gần xe nhất không phải là điểm đầu tiên, nghĩa là xe đã đi vượt qua
                # các điểm trước đó (do VFH bẻ lái vòng qua). Ta cắt bỏ hết điểm cũ!
                if closest_idx > 0:
                    path_world = path_world[closest_idx:]

                # Pop waypoint nếu đã đủ gần (Nới lỏng ra 0.25m để dễ ăn điểm hơn)
                if np.hypot(path_world[0][0] - corrected_x, path_world[0][1] - corrected_y) < 0.25:
                    path_world.pop(0)

            # check2: KIỂM TRA QUỸ ĐẠO CÓ BỊ LỆCH NHIỀU QUÁ KHÔNG ĐỂ A* LẠI ---
            if len(path_world) > 0:
                dist_to_path = np.hypot(path_world[0][0] - corrected_x, path_world[0][1] - corrected_y)
                
                # Nếu VFH đẩy xe văng ra khỏi quỹ đạo A* lớn hơn 0.4 m
                if dist_to_path > 0.4:
                    print(f"Xe lệch quỹ đạo ({dist_to_path:.2f}m)! Đang tìm đường mới...")
                    start_m = world_to_map(corrected_x, corrected_y)
                    goal_m  = world_to_map(target_x_slam, target_y_slam)
                    new_path = a_star(start_m, goal_m)
                    
                    if new_path is not None:
                        path_world = [map_to_world(x, y) for (x, y) in new_path]
                        print("Tìm đường mới thành công!")
                    else:
                        print("Bị kẹt! Không tìm được đường mới.")
                        nav_goal_active = False # Dừng xe tạm thời
            
            # check3: GỬI TỌA ĐỘ CHO ESP32 ---
            if nav_goal_active and len(path_world) > 0:
                # Lấy 1 điểm làm mục tiêu cục bộ 
                target_idx = min(1, len(path_world) - 1) # -2 => mượt hơn
                local_target_x, local_target_y = path_world[target_idx]
                
                dx_slam = local_target_x - corrected_x
                dy_slam = local_target_y - corrected_y

                dist_to_final = np.hypot(target_x_slam - corrected_x, target_y_slam - corrected_y)

                if dist_to_final < 0.15:
                    nav_goal_active = False
                    print("Đã đến đích!")
                else:
                    rel_angle      = normalize_angle(np.arctan2(dy_slam, dx_slam) - corrected_theta)
                    dist_to_local  = np.hypot(dx_slam, dy_slam)
                    
                    # Odometry
                    esp_target_x   = rx + dist_to_local * np.cos(rtheta + rel_angle)
                    esp_target_y   = ry + dist_to_local * np.sin(rtheta + rel_angle)

                    sync    = [0xBB, 0x66]
                    payload = struct.pack('<ff', esp_target_x, esp_target_y)
                    chk     = 0
                    for b in payload:
                        chk ^= b
                    pkt = bytes(sync) + payload + bytes([chk])
                    try:
                        sock.sendto(pkt, (ESP32_IP, ESP32_PORT))
                    except Exception:
                        pass

        # ==========================================
        # 5. RENDER
        # ==========================================
        if current_time - last_draw_time > 0.05:
            trajectory_plot.set_data(trajectory_x, trajectory_y)

            hx = corrected_x + 0.3 * np.cos(corrected_theta)
            hy = corrected_y + 0.3 * np.sin(corrected_theta)
            heading_plot.set_data([corrected_x, hx], [corrected_y, hy])

            lidar_plot.set_data(current_draw_scan[:, 0], current_draw_scan[:, 1])
            robot_circle.center = (corrected_x, corrected_y)

            if len(path_world) > 0:
                path_plot.set_data([p[0] for p in path_world],
                                   [p[1] for p in path_world])
            else:
                path_plot.set_data([], [])

            icp_text.set_text(icp_status)

            ax.set_title(
                f"SLAM  |  X:{corrected_x:.2f}m  Y:{corrected_y:.2f}m  "
                f"θ:{corrected_theta * 180 / np.pi:.1f}°",
                color='white', fontsize=10)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            last_draw_time = current_time

    except KeyboardInterrupt:
        print("\nĐã thoát!")
        break
    except Exception as e:
        print(f"[LỖI] {e}")
        time.sleep(0.1)

sock.close()