#!/usr/bin/env python3

import numpy as np


class TempSpeedProfile:
    """
    [수정 11] 임시 곡률 기반 속도 프로파일 생성기.

    목적:
        - global planner의 velocity profile이 아직 없을 때만 사용한다.
        - controller_manager.py에서 /local_waypoints의 speed column만 임시로 덮어쓴다.
        - mppi.py는 항상 waypoints[:, 2]를 target speed로 추종한다.

    공식:
        a_lat = v^2 * |kappa|
        v = sqrt(a_lat_max / (|kappa| + eps))

    후처리:
        1. v_min, v_max clip
        2. forward pass로 가속 제한
        3. backward pass로 감속 제한
    """

    def __init__(
        self,
        v_min=0.03,
        v_max=0.10,
        a_lat_max=0.03,
        a_accel_max=0.05,
        a_decel_max=0.08,
        kappa_eps=1e-3,
    ):
        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.a_lat_max = float(a_lat_max)
        self.a_accel_max = float(a_accel_max)
        self.a_decel_max = float(a_decel_max)
        self.kappa_eps = float(kappa_eps)

    def build_profile(self, waypoints):
        """
        waypoints column:
            0: x
            1: y
            2: vx_mps
            4: s
            5: kappa_radpm

        return:
            v_profile [m/s]
        """

        waypoints = np.asarray(waypoints, dtype=float)

        if waypoints.ndim != 2 or waypoints.shape[0] == 0:
            return np.array([], dtype=float)

        if waypoints.shape[0] == 1:
            return np.array([self.v_min], dtype=float)

        kappa = np.abs(waypoints[:, 5])

        # 곡률 기반 raw speed
        v_raw = np.sqrt(self.a_lat_max / (kappa + self.kappa_eps))

        # 기본 속도 제한
        v_profile = np.clip(v_raw, self.v_min, self.v_max)

        # waypoint 간 거리 ds
        xy = waypoints[:, 0:2]
        ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        ds = np.maximum(ds, 1e-4)

        # forward pass: 가속 제한
        for i in range(1, len(v_profile)):
            v_allowed = np.sqrt(
                v_profile[i - 1] ** 2 + 2.0 * self.a_accel_max * ds[i - 1]
            )
            v_profile[i] = min(v_profile[i], v_allowed)

        # backward pass: 감속 제한
        for i in range(len(v_profile) - 2, -1, -1):
            v_allowed = np.sqrt(
                v_profile[i + 1] ** 2 + 2.0 * self.a_decel_max * ds[i]
            )
            v_profile[i] = min(v_profile[i], v_allowed)

        return v_profile.astype(float)

    def speed_from_curvature(self, kappa):
        """
        단일 waypoint용 속도 계산.
        전체 profile smoothing이 필요하면 build_profile()을 사용한다.
        """

        kappa = abs(float(kappa))
        v = np.sqrt(self.a_lat_max / (kappa + self.kappa_eps))
        v = np.clip(v, self.v_min, self.v_max)

        return float(v)
