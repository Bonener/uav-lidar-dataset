"""
Inspect ROS bag metadata: topics, types, counts, frequencies.

Usage:
    python scripts/inspect_bag.py                     # 扫描 bags/ 目录下所有 .bag
    python scripts/inspect_bag.py bags/xxx.bag         # 指定单个 bag 文件
"""
import sys
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader

# 项目根目录（scripts/ 的上一级）
ROOT = Path(__file__).resolve().parent.parent
BAGS_DIR = ROOT / "bags"


def inspect_one(bag_path: Path):
    print(f"\n{'=' * 80}")
    print(f"File: {bag_path.name}  ({bag_path.stat().st_size / 1e6:.1f} MB)")
    print(f"{'=' * 80}")
    with AnyReader([bag_path]) as reader:
        duration_s = reader.duration / 1e9
        print(f"Duration: {duration_s:.2f} s")
        print(f"Total messages: {reader.message_count}")
        print()
        header = f"{'Topic':<45} {'Type':<40} {'Count':>8} {'Freq(Hz)':>10}"
        print(header)
        print("-" * len(header))
        for conn in reader.connections:
            count = sum(1 for _ in reader.messages(connections=[conn]))
            freq = count / duration_s if duration_s > 0 else 0
            print(f"{conn.topic:<45} {conn.msgtype:<40} {count:>8} {freq:>10.2f}")


def main():
    parser = argparse.ArgumentParser(description="Inspect ROS bag metadata")
    parser.add_argument("bag", nargs="?", help="Path to .bag file (default: scan bags/ dir)")
    args = parser.parse_args()

    if args.bag:
        bag_path = Path(args.bag).resolve()
        if not bag_path.exists():
            print(f"Error: {bag_path} not found")
            sys.exit(1)
        inspect_one(bag_path)
    else:
        # 默认扫描 bags/ 目录
        if not BAGS_DIR.exists():
            print(f"Error: {BAGS_DIR} not found. Run from project root or specify bag path.")
            sys.exit(1)
        bags = sorted(BAGS_DIR.glob("**/*.bag"))
        if not bags:
            print(f"No .bag files found in {BAGS_DIR}")
            sys.exit(0)
        for bag in bags:
            inspect_one(bag)


if __name__ == "__main__":
    main()
