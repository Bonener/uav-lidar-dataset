"""
3D 点云地图 + 飞行轨迹叠加图
布局: 左侧 3D 主图 + 右侧 2D 俯视图

Usage:
    python scripts/plot_map_trajectory.py                           # 默认处理 bags/ 下第一个 bag
    python scripts/plot_map_trajectory.py bags/xxx.bag              # 指定 bag
    python scripts/plot_map_trajectory.py bags/xxx.bag out.png      # 指定 bag + 输出路径
"""
import sys
import struct
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from rosbags.rosbag1 import Reader

# 项目根目录（scripts/ 的上一级）
ROOT = Path(__file__).resolve().parent.parent
BAGS_DIR = ROOT / "bags"
PREVIEW_DIR = ROOT / "docs" / "preview"


# ---------- 手动解析器 ----------
class MsgParser:
    def __init__(self, data):
        self.data = data; self.pos = 0
    def uint32(self):
        v = struct.unpack_from("<I", self.data, self.pos)[0]; self.pos += 4; return v
    def float64(self):
        v = struct.unpack_from("<d", self.data, self.pos)[0]; self.pos += 8; return v
    def string(self):
        n = self.uint32(); s = self.data[self.pos:self.pos+n].decode("utf-8", errors="replace"); self.pos += n; return s
    def time(self):
        sec = self.uint32(); nsec = self.uint32(); return sec + nsec*1e-9
    def bytes(self, n):
        b = self.data[self.pos:self.pos+n]; self.pos += n; return b

def parse_header(p):
    p.uint32()  # seq
    p.time()
    p.string()

def parse_odom_position(data):
    p = MsgParser(data)
    parse_header(p)
    p.string()  # child_frame_id
    x = p.float64(); y = p.float64(); z = p.float64()
    return x, y, z

def parse_pointcloud2_xyz(data, max_points=800):
    p = MsgParser(data)
    parse_header(p)
    height = p.uint32(); width = p.uint32()
    n_fields = p.uint32()
    offsets = {}
    for _ in range(n_fields):
        fname = p.string()
        foffset = p.uint32()
        p.pos += 5  # datatype(1) + count(4)
        offsets[fname] = foffset
    p.pos += 1  # is_bigendian
    point_step = p.uint32()
    p.uint32()  # row_step
    data_len = p.uint32()
    cloud_data = p.bytes(data_len)

    n = width * height
    idx = np.linspace(0, n-1, min(n, max_points), dtype=int)
    xo = offsets.get("x", 0); yo = offsets.get("y", 4); zo = offsets.get("z", 8)
    xs = np.empty(len(idx)); ys = np.empty(len(idx)); zs = np.empty(len(idx))
    for k, i in enumerate(idx):
        base = i * point_step
        xs[k] = struct.unpack_from("<f", cloud_data, base+xo)[0]
        ys[k] = struct.unpack_from("<f", cloud_data, base+yo)[0]
        zs[k] = struct.unpack_from("<f", cloud_data, base+zo)[0]
    return xs, ys, zs


def generate_plot(bag_path: Path, out_path: Path):
    print(f"读取 bag: {bag_path.name}")
    reader = Reader(bag_path); reader.open()
    topic_conn = {}
    for c in reader.connections:
        if c.topic in ("/mavros/local_position/odom", "/cloud_registered"):
            topic_conn[c.topic] = c

    odom_traj = []
    cloud_xs, cloud_ys, cloud_zs = [], [], []

    for conn, ts_ns, raw in reader.messages(connections=list(topic_conn.values())):
        try:
            if conn.topic == "/mavros/local_position/odom":
                x, y, z = parse_odom_position(raw)
                odom_traj.append((ts_ns/1e9, x, y, z))
            elif conn.topic == "/cloud_registered":
                xs, ys, zs = parse_pointcloud2_xyz(raw, max_points=800)
                cloud_xs.append(xs); cloud_ys.append(ys); cloud_zs.append(zs)
        except: pass
    reader.close()
    print(f"  轨迹帧: {len(odom_traj)}, 点云帧: {len(cloud_xs)}")

    if not odom_traj or not cloud_xs:
        print("Error: 缺少轨迹或点云数据")
        return

    all_x = np.concatenate(cloud_xs)
    all_y = np.concatenate(cloud_ys)
    all_z = np.concatenate(cloud_zs)
    print(f"  累计点: {len(all_x)}")

    traj_t = np.array([r[0] for r in odom_traj])
    traj_x = np.array([r[1] for r in odom_traj])
    traj_y = np.array([r[2] for r in odom_traj])
    traj_z = np.array([r[3] for r in odom_traj])
    t0 = traj_t[0]; traj_t_rel = traj_t - t0; duration = traj_t_rel[-1]

    print("绘图...")
    fig = plt.figure(figsize=(20, 9))

    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.scatter(all_x, all_y, all_z, s=0.5, c="gray", alpha=0.4, label="LiDAR map")
    sc = ax3d.scatter(traj_x, traj_y, traj_z, c=traj_t_rel, cmap="viridis", s=12, label="trajectory", zorder=5)
    ax3d.scatter(traj_x[0], traj_y[0], traj_z[0], c="lime", s=100, marker="o", edgecolors="black", linewidth=1, label="start", zorder=10)
    ax3d.scatter(traj_x[-1], traj_y[-1], traj_z[-1], c="red", s=100, marker="s", edgecolors="black", linewidth=1, label="end", zorder=10)
    ax3d.set_xlabel("X (m)", fontsize=10)
    ax3d.set_ylabel("Y (m)", fontsize=10)
    ax3d.set_zlabel("Z (m)", fontsize=10)
    ax3d.set_title(f"3D View  ({duration:.1f}s, {len(cloud_xs)} frames)", fontsize=12, fontweight="bold")
    ax3d.legend(loc="upper left", fontsize=9)
    ax3d.view_init(elev=35, azim=-60)
    ax3d.set_box_aspect([1, 1, 0.6])

    ax2d = fig.add_subplot(1, 2, 2)
    ax2d.scatter(all_x, all_y, s=0.3, c="lightgray", alpha=0.5, label="LiDAR map")
    sc2 = ax2d.scatter(traj_x, traj_y, c=traj_t_rel, cmap="viridis", s=15, label="trajectory", zorder=5)
    ax2d.scatter(traj_x[0], traj_y[0], c="lime", s=100, marker="o", edgecolors="black", linewidth=1, label="start", zorder=10)
    ax2d.scatter(traj_x[-1], traj_y[-1], c="red", s=100, marker="s", edgecolors="black", linewidth=1, label="end", zorder=10)
    ax2d.set_xlabel("X (m)", fontsize=10)
    ax2d.set_ylabel("Y (m)", fontsize=10)
    ax2d.set_title("Top-Down View (XY plane)", fontsize=12, fontweight="bold")
    ax2d.axis("equal")
    ax2d.legend(loc="upper left", fontsize=9)
    ax2d.grid(True, alpha=0.3)
    plt.colorbar(sc2, ax=ax2d, label="Time (s)")

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"已保存: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate trajectory + point cloud overlay plot")
    parser.add_argument("bag", nargs="?", help="Path to .bag file (default: first bag in bags/)")
    parser.add_argument("output", nargs="?", help="Output PNG path (default: docs/preview/<bag_name>.png)")
    args = parser.parse_args()

    # 确定 bag 路径
    if args.bag:
        bag_path = Path(args.bag).resolve()
    else:
        bags = sorted(BAGS_DIR.glob("**/*.bag"))
        if not bags:
            print(f"No .bag files found in {BAGS_DIR}")
            sys.exit(1)
        bag_path = bags[0]
        print(f"未指定 bag，使用: {bag_path.name}")

    if not bag_path.exists():
        print(f"Error: {bag_path} not found")
        sys.exit(1)

    # 确定输出路径
    if args.output:
        out_path = Path(args.output).resolve()
    else:
        # 默认输出到 docs/preview/，文件名基于 bag 名
        stem = bag_path.stem  # e.g. "2025-11-25-16-20-29"
        # 提取时间部分 "16-20-29"
        time_part = stem.split("-")[-3:] + stem.split("-")[-2:] + stem.split("-")[-1:]
        # 简化：直接用 preview_ + 后三段
        parts = stem.split("-")
        if len(parts) >= 6:
            name = f"preview_{parts[3]}-{parts[4]}-{parts[5]}.png"
        else:
            name = f"preview_{stem}.png"
        out_path = PREVIEW_DIR / name

    generate_plot(bag_path, out_path)


if __name__ == "__main__":
    main()
