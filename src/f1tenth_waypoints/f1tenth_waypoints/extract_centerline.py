#!/usr/bin/env python3
"""
맵(pgm/yaml)에서 트랙 센터라인을 오프라인으로 1회 추출해서 CSV로 저장하는 스크립트.

ROS 노드가 아니라 커맨드라인 스크립트입니다. slam_toolbox로 맵을 완성한 뒤
딱 한 번(트랙이 바뀌지 않는 한) 실행해서 waypoint CSV를 생성하세요.

사용법:
  python3 extract_centerline.py --map ../../f1tenth_maps/maps/lab_track.yaml \
      --out ../config/centerline.csv

알고리즘:
  1. yaml에서 pgm 경로/해상도/원점을 읽는다.
  2. free space(주행 가능 영역)를 이진화한다.
  3. skimage.morphology.skeletonize로 트랙의 중심선을 1px 두께로 뽑는다.
  4. 스켈레톤 픽셀들을 인접 관계로 정렬해 순서가 있는 폐루프 waypoint 리스트로 만든다.
  5. 픽셀 좌표 -> map frame 실좌표(m)로 변환해서 CSV(x,y,speed)로 저장한다.

필요 패키지: numpy, pyyaml, pillow, scikit-image
  pip install numpy pyyaml pillow scikit-image --break-system-packages
"""
import argparse
import csv
import os

import numpy as np
import yaml
from PIL import Image
from skimage.morphology import skeletonize


def load_map(yaml_path: str):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    img = np.array(Image.open(pgm_path))
    resolution = meta['resolution']          # m/pixel
    origin = meta['origin']                  # [x, y, theta] map frame origin (하단좌측)
    occupied_thresh = meta.get('occupied_thresh', 0.65)
    free_thresh = meta.get('free_thresh', 0.196)
    negate = meta.get('negate', 0)
    return img, resolution, origin, occupied_thresh, free_thresh, negate


def binarize_free_space(img, free_thresh, negate):
    norm = img.astype(np.float32) / 255.0
    if negate:
        norm = 1.0 - norm
    # pgm은 밝을수록 free space인 게 표준(occupancy grid 규칙과 반대 방향 주의)
    free = norm > (1.0 - free_thresh)
    return free


def order_skeleton_points(skel: np.ndarray):
    """스켈레톤 픽셀들을 최근접 이웃 방식으로 순서화 (폐루프 트랙 가정)."""
    ys, xs = np.nonzero(skel)
    pts = list(zip(xs.tolist(), ys.tolist()))
    if not pts:
        return []
    ordered = [pts.pop(0)]
    pts_arr = np.array(pts)
    while pts:
        last = np.array(ordered[-1])
        dists = np.sum((pts_arr - last) ** 2, axis=1)
        idx = int(np.argmin(dists))
        ordered.append(tuple(pts_arr[idx]))
        pts_arr = np.delete(pts_arr, idx, axis=0)
        pts = pts_arr.tolist()
    return ordered


def pixel_to_map(x_px, y_px, resolution, origin, img_height):
    # pgm은 top-left가 원점이므로 y를 뒤집어야 map frame과 맞음
    x_m = origin[0] + x_px * resolution
    y_m = origin[1] + (img_height - y_px) * resolution
    return x_m, y_m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='map yaml 경로')
    ap.add_argument('--out', required=True, help='출력 csv 경로')
    ap.add_argument('--default_speed', type=float, default=2.0,
                     help='초기 목표 속도(m/s), 나중에 곡률 기반으로 바꿀 수 있음')
    args = ap.parse_args()

    img, resolution, origin, occ_thresh, free_thresh, negate = load_map(args.map)
    free = binarize_free_space(img, free_thresh, negate)
    skel = skeletonize(free)

    ordered_px = order_skeleton_points(skel)
    if not ordered_px:
        raise RuntimeError('스켈레톤에서 포인트를 찾지 못했습니다. free_thresh/negate 값을 확인하세요.')

    rows = []
    for x_px, y_px in ordered_px:
        x_m, y_m = pixel_to_map(x_px, y_px, resolution, origin, img.shape[0])
        rows.append((x_m, y_m, args.default_speed))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'speed'])
        writer.writerows(rows)

    print(f'{len(rows)}개의 waypoint를 {args.out} 에 저장했습니다.')
    print('주의: 최근접 이웃 정렬은 트랙 형태에 따라 꼬일 수 있으니, '
          'RViz에서 Path로 시각화해 순서가 매끄러운지 꼭 확인하세요.')


if __name__ == '__main__':
    main()
