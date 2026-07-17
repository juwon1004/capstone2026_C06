#!/usr/bin/env python3

import logging
import numpy as np


class MPPI_Controller:
    """
    MPPI controller for waypoint / centerline tracking.

    Structure:
        1. sample steering sequences
        2. rollout trajectories using kinematic bicycle model
        3. evaluate costs
        4. compute weighted average control sequence
        5. apply first steering command

    This version includes:
        - reference trajectory cost
        - centerline distance cost
        - terminal reference cost
        - heading alignment cost
        - steering effort cost
        - steering smoothness cost
        - steering rate cost
        - waypoint speed tracking via waypoint_array_in_map[:, 2]
        - scan based obstacle cost for reactive obstacle avoidance
    """

    def __init__(
                self,
                t_clip_min,
                t_clip_max,
                m_l1,
                q_l1,
                speed_lookahead,
                lat_err_coeff,
                acc_scaler_for_steer,
                dec_scaler_for_steer,
                start_scale_speed,
                end_scale_speed,
                downscale_factor,
                speed_lookahead_for_steer,

                prioritize_dyn,
                trailing_gap,
                trailing_p_gain,
                trailing_i_gain,
                trailing_d_gain,
                blind_trailing_speed,

                loop_rate,
                wheelbase,
                state_machine_rate,

                logger_info=logging.info,
                logger_warn=logging.warn
            ):

        # =========================
        # 1. Parameters from controller_manager
        # =========================

        self.t_clip_min = t_clip_min
        self.t_clip_max = t_clip_max
        self.m_l1 = m_l1
        self.q_l1 = q_l1
        self.speed_lookahead = speed_lookahead
        self.lat_err_coeff = lat_err_coeff
        self.acc_scaler_for_steer = acc_scaler_for_steer
        self.dec_scaler_for_steer = dec_scaler_for_steer
        self.start_scale_speed = start_scale_speed
        self.end_scale_speed = end_scale_speed
        self.downscale_factor = downscale_factor
        self.speed_lookahead_for_steer = speed_lookahead_for_steer

        self.prioritize_dyn = prioritize_dyn
        self.trailing_gap = trailing_gap
        self.trailing_p_gain = trailing_p_gain
        self.trailing_i_gain = trailing_i_gain
        self.trailing_d_gain = trailing_d_gain
        self.blind_trailing_speed = blind_trailing_speed

        self.loop_rate = loop_rate
        self.wheelbase = wheelbase
        self.state_machine_rate = state_machine_rate

        self.logger_info = logger_info
        self.logger_warn = logger_warn

        # =========================
        # 2. MPPI parameters
        # =========================

        self.horizon = 60
        self.num_samples = 180

        # 작을수록 best sample 쪽으로 강하게 따라감.
        # 너무 작으면 조향이 튈 수 있음.
        self.lambda_ = 0.75

        # Steering sampling noise [rad]
        self.steer_sigma = 0.12

        # Steering limit [rad]
        self.steer_limit = 0.55

        # Steering rate limit per control cycle [rad]
        # 이건 cost가 아니라 hard constraint 쪽에 가까움.
        self.steer_rate_limit = 0.09

        # =========================
        # 3. Speed limits
        # =========================

        # [수정 9] 속도 프로파일을 manager에서 만들어 waypoints[:, 2]에 넣으므로,
        # MPPI 내부 clamp가 임시 프로파일을 망가뜨리지 않도록 범위를 넓힌다.
        # 실제 안전 속도 제한은 controller_manager.py의 TempSpeedProfile에서 한다.
        self.max_speed = 1.00
        self.min_speed = 0.05
        self.default_speed = 0.10

        # =========================
        # 4. dt
        # =========================

        if self.loop_rate is not None and self.loop_rate > 0:
            self.dt = 1.0 / self.loop_rate
        else:
            self.dt = 0.05

        # =========================
        # 5. Nominal control sequence
        # =========================

        self.u_nominal = np.zeros(self.horizon)
        self.curr_steering_angle = 0.0
        self.idx_nearest_waypoint = 0

        self.rng = np.random.default_rng()

        # =========================
        # 6. Cost weights
        # =========================
        # 여기만 바꾸면서 cost 조합 실험하면 됨.
        #
        # 중앙선/path tracking:
        #   w_center, w_ref, w_terminal
        #
        # heading:
        #   w_heading
        #
        # input:
        #   w_steer, w_steer_smooth, w_steer_rate

        # 전체 궤적이 기준 경로 근처에 머무는 비용
        self.w_center = 0.15

        # 시간별 reference trajectory를 따라가는 비용
        self.w_ref = 10.0

        # 예측 yaw가 reference path yaw와 정렬되도록 하는 비용
        self.w_heading = 1.6

        # 마지막 예측점이 reference 마지막 점 근처에 가는 비용
        self.w_terminal = 2.8

        # 조향 크기 비용
        self.w_steer = 0.06

        # 조향 변화 smooth 비용
        self.w_steer_smooth = 0.22

        # 현재 조향과 첫 조향 차이 비용
        self.w_steer_rate = 0.35

        # =========================
        # Obstacle cost parameters
        # =========================
        # /scan으로 보이는 점들을 map 좌표 obstacle point로 변환한 뒤,
        # rollout trajectory가 이 점들에 가까워지면 비용을 준다.
        # 초반 튜닝은 여기 값부터 조절하면 된다.
        self.w_obstacle = 80.0
        self.safety_radius = 0.52
        self.collision_radius = 0.22
        self.hard_collision_cost = 2e5
        self.scan_max_range = 2.20
        self.scan_downsample = 4
        self.scan_angle_limit = np.deg2rad(110.0)
        self.scan_min_x_body = 0.05
        self.scan_max_y_body = 1.00

        # =========================
        # 7. Reference lookahead
        # =========================

        self.min_ref_step = 3
        self.extra_ref_step = 2

        # =========================
        # 8. Local waypoint window
        # =========================

        self.local_back_points = 8
        self.local_front_points = 110

        self.path_direction = 1

    def main_loop(
            self,
            state,
            position_in_map,
            waypoint_array_in_map,
            speed_now,
            opponent,
            position_in_map_frenet,
            acc_now,
            track_length,
            scan=None
        ):

        if position_in_map is None:
            return self.stop_return()

        if waypoint_array_in_map is None or len(waypoint_array_in_map) < 2:
            return self.stop_return()

        self.state = state
        self.position_in_map = position_in_map
        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.opponent = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.acc_now = acc_now
        self.track_length = track_length
        self.scan = scan

        x = float(position_in_map[0, 0])
        y = float(position_in_map[0, 1])
        yaw = float(position_in_map[0, 2])

        car_position = np.array([x, y])

        # manager에서 centerline_array를 넘기면 중앙선을 따라간다.
        # racing line을 넘기면 racing line을 따라간다.
        waypoints_xy = waypoint_array_in_map[:, :2]

        self.idx_nearest_waypoint = self.nearest_waypoint(
            car_position,
            waypoints_xy
        )

        # =========================
        # Speed command
        # =========================

        # [수정 10] MPPI는 속도 프로파일을 직접 만들지 않는다.
        # controller_manager.py가 임시 곡률 기반 speed를 waypoints[:, 2]에 넣어주고,
        # 나중에 global planner가 완성되면 그 vx_mps가 그대로 waypoints[:, 2]에 들어온다.
        if waypoint_array_in_map.shape[1] >= 3:
            waypoint_speed = float(waypoint_array_in_map[self.idx_nearest_waypoint, 2])
        else:
            waypoint_speed = self.default_speed

        if (not np.isfinite(waypoint_speed)) or waypoint_speed <= 0.0:
            waypoint_speed = self.default_speed

        speed_command = np.clip(
            waypoint_speed,
            self.min_speed,
            self.max_speed
        )

        steering_angle, mppi_point, mppi_distance = self.solve_mppi(
            x=x,
            y=y,
            yaw=yaw,
            speed=speed_command,
            waypoints_xy=waypoints_xy,
            scan=scan
        )

        acceleration = 0.0
        jerk = 0.0

        return (
            float(speed_command),
            float(acceleration),
            float(jerk),
            float(steering_angle),
            mppi_point,
            float(mppi_distance),
            int(self.idx_nearest_waypoint)
        )

    def solve_mppi(self, x, y, yaw, speed, waypoints_xy, scan=None):
        car_position = np.array([x, y])

        local_waypoints = self.get_local_waypoints(
            waypoints_xy=waypoints_xy,
            nearest_idx=self.idx_nearest_waypoint
        )

        if local_waypoints is None or len(local_waypoints) < 2:
            return 0.0, np.array([x, y]), 0.0

        self.path_direction = self.select_path_direction(
            car_position=car_position,
            yaw=yaw,
            waypoints_xy=waypoints_xy,
            nearest_idx=self.idx_nearest_waypoint
        )

        mean_spacing = self.estimate_mean_spacing(waypoints_xy)

        ref_points, ref_yaws = self.build_reference_points_and_yaws(
            waypoints_xy=waypoints_xy,
            nearest_idx=self.idx_nearest_waypoint,
            direction=self.path_direction,
            speed=speed,
            mean_spacing=mean_spacing
        )

        # =========================
        # 1. Sample control sequences
        # =========================

        control_samples = np.zeros((self.num_samples, self.horizon))

        # deterministic samples
        # 튜닝 포인트:
        # 너무 큰 고정 조향 후보가 매 주기 번갈아 best가 되면 차량이 좌우로 흔들 수 있다.
        # 그래서 초기 안정화 버전에서는 extreme steering sample을 줄였다.
        control_samples[0, :] = 0.0
        control_samples[1, :] = 0.08
        control_samples[2, :] = -0.08
        control_samples[3, :] = 0.16
        control_samples[4, :] = -0.16
        control_samples[5, :] = 0.28
        control_samples[6, :] = -0.28
        control_samples[7, :] = 0.40
        control_samples[8, :] = -0.40

        # ramp steering samples
        control_samples[9, :] = np.linspace(0.0, 0.25, self.horizon)
        control_samples[10, :] = np.linspace(0.0, -0.25, self.horizon)
        control_samples[11, :] = np.linspace(0.08, 0.35, self.horizon)
        control_samples[12, :] = np.linspace(-0.08, -0.35, self.horizon)

        num_deterministic = 13
        num_random = self.num_samples - num_deterministic

        noise = self.rng.normal(
            loc=0.0,
            scale=self.steer_sigma,
            size=(num_random, self.horizon)
        )

        control_samples[num_deterministic:, :] = self.u_nominal + noise

        control_samples = np.clip(
            control_samples,
            -self.steer_limit,
            self.steer_limit
        )

        # =========================
        # 2. Rollout
        # =========================

        trajs = self.rollout_batch(
            x=x,
            y=y,
            yaw=yaw,
            speed=speed,
            control_samples=control_samples
        )

        # =========================
        # 3. Cost
        # =========================

        obstacle_points = self.scan_to_map_points(
            scan=scan,
            x=x,
            y=y,
            yaw=yaw
        )

        costs = self.calc_total_cost_batch(
            trajs=trajs,
            control_samples=control_samples,
            local_waypoints=local_waypoints,
            ref_points=ref_points,
            ref_yaws=ref_yaws,
            obstacle_points=obstacle_points
        )

        # =========================
        # 4. MPPI weighted average
        # =========================

        min_cost = np.min(costs)

        weights = np.exp(
            -(costs - min_cost) / max(self.lambda_, 1e-6)
        )

        weight_sum = np.sum(weights)

        if weight_sum < 1e-9 or not np.isfinite(weight_sum):
            new_u = control_samples[int(np.argmin(costs))].copy()
        else:
            weights = weights / weight_sum
            new_u = weights @ control_samples

        best_idx = int(np.argmin(costs))

        # =========================
        # 5. Damped nominal update
        # =========================

        shifted = np.zeros_like(self.u_nominal)
        shifted[:-1] = new_u[1:]
        shifted[-1] = 0.0

        self.u_nominal = 0.80 * shifted

        # =========================
        # 6. First steering command
        # =========================

        steering_angle = float(new_u[0])

        steering_angle = np.clip(
            steering_angle,
            -self.steer_limit,
            self.steer_limit
        )

        steering_angle = np.clip(
            steering_angle,
            self.curr_steering_angle - self.steer_rate_limit,
            self.curr_steering_angle + self.steer_rate_limit
        )

        self.curr_steering_angle = steering_angle

        # =========================
        # 7. Visualization point
        # =========================

        best_traj = trajs[best_idx]

        marker_step = min(12, self.horizon)

        mppi_point = best_traj[marker_step, :2]

        mppi_distance = np.linalg.norm(mppi_point - car_position)

        return steering_angle, mppi_point, mppi_distance

    def rollout_batch(self, x, y, yaw, speed, control_samples):
        num_samples = control_samples.shape[0]

        trajs = np.zeros((num_samples, self.horizon + 1, 3))

        cur_x = np.full(num_samples, x, dtype=float)
        cur_y = np.full(num_samples, y, dtype=float)
        cur_yaw = np.full(num_samples, yaw, dtype=float)

        trajs[:, 0, 0] = cur_x
        trajs[:, 0, 1] = cur_y
        trajs[:, 0, 2] = cur_yaw

        for t in range(self.horizon):
            delta = control_samples[:, t]

            cur_x = cur_x + speed * np.cos(cur_yaw) * self.dt
            cur_y = cur_y + speed * np.sin(cur_yaw) * self.dt

            cur_yaw = cur_yaw + (
                speed / self.wheelbase
            ) * np.tan(delta) * self.dt

            cur_yaw = self.normalize_angle(cur_yaw)

            trajs[:, t + 1, 0] = cur_x
            trajs[:, t + 1, 1] = cur_y
            trajs[:, t + 1, 2] = cur_yaw

        return trajs

    def calc_total_cost_batch(
            self,
            trajs,
            control_samples,
            local_waypoints,
            ref_points,
            ref_yaws,
            obstacle_points=None
        ):
        center_cost = self.calc_centerline_cost_batch(
            trajs=trajs,
            local_waypoints=local_waypoints
        )

        ref_cost = self.calc_reference_cost_batch(
            trajs=trajs,
            ref_points=ref_points
        )

        heading_cost = self.calc_heading_cost_batch(
            trajs=trajs,
            ref_yaws=ref_yaws
        )

        terminal_cost = self.calc_terminal_cost_batch(
            trajs=trajs,
            ref_points=ref_points
        )

        steer_cost = np.sum(control_samples ** 2, axis=1)

        steer_diff = np.diff(control_samples, axis=1)
        smooth_cost = np.sum(steer_diff ** 2, axis=1)

        steer_rate_cost = (control_samples[:, 0] - self.curr_steering_angle) ** 2

        obstacle_cost = self.calc_obstacle_cost_batch(
            trajs=trajs,
            obstacle_points=obstacle_points
        )

        total_cost = (
            self.w_center * center_cost
            + self.w_ref * ref_cost
            + self.w_heading * heading_cost
            + self.w_terminal * terminal_cost
            + self.w_steer * steer_cost
            + self.w_steer_smooth * smooth_cost
            + self.w_steer_rate * steer_rate_cost
            + self.w_obstacle * obstacle_cost
        )

        return total_cost


    def scan_to_map_points(self, scan, x, y, yaw):
        """
        LaserScan을 map 좌표계의 obstacle point cloud로 변환한다.

        scan.ranges는 LiDAR 기준 polar 좌표다.
        먼저 차량 기준 body 좌표로 바꾸고, 현재 차량 pose(x, y, yaw)를 이용해
        map 좌표로 회전/이동시킨다.

        주의:
            - 이 1차 버전은 LiDAR frame이 base_link와 거의 같다고 가정한다.
            - 실차에서 LiDAR 위치/각도가 다르면 TF 변환으로 바꾸는 게 더 정확하다.
        """
        if scan is None:
            return None

        ranges = np.asarray(scan.ranges, dtype=float)
        if ranges.size == 0:
            return None

        idx = np.arange(0, ranges.size, max(1, int(self.scan_downsample)))
        ranges_ds = ranges[idx]
        angles = scan.angle_min + idx * scan.angle_increment

        max_range = min(float(scan.range_max), float(self.scan_max_range))

        valid = (
            np.isfinite(ranges_ds)
            & (ranges_ds > float(scan.range_min))
            & (ranges_ds < max_range)
            & (np.abs(angles) <= self.scan_angle_limit)
        )

        if not np.any(valid):
            return None

        ranges_valid = ranges_ds[valid]
        angles_valid = angles[valid]

        xs_body = ranges_valid * np.cos(angles_valid)
        ys_body = ranges_valid * np.sin(angles_valid)

        # 뒤쪽 점과 너무 옆쪽의 벽 점은 현재 rollout 회피에 거의 도움이 없고,
        # obstacle cost를 불필요하게 흔들 수 있으므로 제외한다.
        # 작은 직사각형 장애물처럼 전방 주행 경로 근처에 있는 점을 우선 반영한다.
        body_valid = (
            (xs_body > self.scan_min_x_body)
            & (np.abs(ys_body) < self.scan_max_y_body)
        )
        if not np.any(body_valid):
            return None

        xs_body = xs_body[body_valid]
        ys_body = ys_body[body_valid]

        c = np.cos(yaw)
        s = np.sin(yaw)

        xs_map = x + c * xs_body - s * ys_body
        ys_map = y + s * xs_body + c * ys_body

        return np.stack([xs_map, ys_map], axis=1)

    def calc_obstacle_cost_batch(self, trajs, obstacle_points):
        """
        각 rollout trajectory가 scan obstacle point에 얼마나 가까운지 비용화한다.

        trajs shape:
            [num_samples, horizon + 1, 3]
        obstacle_points shape:
            [num_obstacles, 2]

        비용 구조:
            1. safety_radius 안으로 들어오면 soft cost 증가
            2. collision_radius 안으로 들어오면 hard penalty 추가
        """
        num_samples = trajs.shape[0]

        if obstacle_points is None or len(obstacle_points) == 0:
            return np.zeros(num_samples)

        positions = trajs[:, 1:, :2]

        diff = positions[:, :, None, :] - obstacle_points[None, None, :, :]
        dist = np.linalg.norm(diff, axis=-1)

        min_dist = np.min(dist, axis=2)

        soft = np.maximum(self.safety_radius - min_dist, 0.0) / max(self.safety_radius, 1e-6)
        soft_cost = soft ** 2

        # 가까운 미래보다 먼 미래의 충돌도 놓치지 않기 위해 뒤쪽 time step에 약간 더 큰 weight를 준다.
        time_weights = np.linspace(1.0, 1.4, self.horizon)

        obstacle_cost = np.sum(
            soft_cost * time_weights[None, :],
            axis=1
        )

        collision = np.any(min_dist < self.collision_radius, axis=1)
        obstacle_cost[collision] += self.hard_collision_cost

        return obstacle_cost

    def calc_centerline_cost_batch(self, trajs, local_waypoints):
        positions = trajs[:, 1:, :2]

        diff = positions[:, :, None, :] - local_waypoints[None, None, :, :]
        dist_sq = np.sum(diff ** 2, axis=-1)

        min_dist_sq = np.min(dist_sq, axis=2)

        time_weights = np.linspace(0.4, 1.0, self.horizon)

        center_cost = np.sum(
            min_dist_sq * time_weights[None, :],
            axis=1
        )

        return center_cost

    def calc_reference_cost_batch(self, trajs, ref_points):
        positions = trajs[:, 1:, :2]

        diff = positions - ref_points[None, :, :]

        dist_sq = np.sum(diff ** 2, axis=-1)

        time_weights = np.linspace(0.7, 2.5, self.horizon)

        ref_cost = np.sum(
            dist_sq * time_weights[None, :],
            axis=1
        )

        return ref_cost

    def calc_heading_cost_batch(self, trajs, ref_yaws):
        predicted_yaws = trajs[:, 1:, 2]

        yaw_errors = self.normalize_angle(
            predicted_yaws - ref_yaws[None, :]
        )

        yaw_error_sq = yaw_errors ** 2

        time_weights = np.linspace(0.7, 2.0, self.horizon)

        heading_cost = np.sum(
            yaw_error_sq * time_weights[None, :],
            axis=1
        )

        return heading_cost

    def calc_terminal_cost_batch(self, trajs, ref_points):
        terminal_points = trajs[:, -1, :2]
        terminal_ref = ref_points[-1]

        diff = terminal_points - terminal_ref[None, :]
        dist_sq = np.sum(diff ** 2, axis=1)

        return dist_sq

    def build_reference_points_and_yaws(self, waypoints_xy, nearest_idx, direction, speed, mean_spacing):
        n_wp = len(waypoints_xy)

        if n_wp == 0:
            return np.zeros((self.horizon, 2)), np.zeros(self.horizon)

        is_closed = self.is_path_closed(waypoints_xy)

        ref_points = np.zeros((self.horizon, 2))
        ref_yaws = np.zeros(self.horizon)

        for t in range(1, self.horizon + 1):
            progress_distance = speed * self.dt * t

            step_offset = int(round(progress_distance / max(mean_spacing, 1e-6)))

            # 커브를 미리 보게 하기 위한 최소 lookahead
            step_offset = max(self.min_ref_step, step_offset + self.extra_ref_step)

            target_idx = nearest_idx + direction * step_offset

            if is_closed:
                target_idx = target_idx % n_wp
            else:
                target_idx = min(max(target_idx, 0), n_wp - 1)

            next_idx = target_idx + direction

            if is_closed:
                next_idx = next_idx % n_wp
            else:
                next_idx = min(max(next_idx, 0), n_wp - 1)

            p = waypoints_xy[target_idx]
            p_next = waypoints_xy[next_idx]

            tangent = p_next - p

            if np.linalg.norm(tangent) < 1e-9:
                if t > 1:
                    ref_yaw = ref_yaws[t - 2]
                else:
                    ref_yaw = 0.0
            else:
                ref_yaw = np.arctan2(tangent[1], tangent[0])

            ref_points[t - 1] = p
            ref_yaws[t - 1] = ref_yaw

        return ref_points, ref_yaws

    def get_local_waypoints(self, waypoints_xy, nearest_idx):
        if waypoints_xy is None or len(waypoints_xy) < 2:
            return None

        n_wp = len(waypoints_xy)
        is_closed = self.is_path_closed(waypoints_xy)

        indices = []

        start = nearest_idx - self.local_back_points
        end = nearest_idx + self.local_front_points

        for idx in range(start, end + 1):
            if is_closed:
                indices.append(idx % n_wp)
            else:
                indices.append(min(max(idx, 0), n_wp - 1))

        indices = np.array(indices, dtype=int)

        return waypoints_xy[indices]

    def select_path_direction(self, car_position, yaw, waypoints_xy, nearest_idx):
        if waypoints_xy is None or len(waypoints_xy) < 2:
            return self.path_direction

        heading_vec = np.array([
            np.cos(yaw),
            np.sin(yaw)
        ])

        plus_point = self.get_index_lookahead_point(
            waypoints_xy=waypoints_xy,
            nearest_idx=nearest_idx,
            direction=1,
            lookahead_steps=15
        )

        minus_point = self.get_index_lookahead_point(
            waypoints_xy=waypoints_xy,
            nearest_idx=nearest_idx,
            direction=-1,
            lookahead_steps=15
        )

        plus_score = np.dot(plus_point - car_position, heading_vec)
        minus_score = np.dot(minus_point - car_position, heading_vec)

        if plus_score >= minus_score:
            return 1

        return -1

    def get_index_lookahead_point(self, waypoints_xy, nearest_idx, direction, lookahead_steps):
        n_wp = len(waypoints_xy)

        if n_wp == 0:
            return np.array([0.0, 0.0])

        is_closed = self.is_path_closed(waypoints_xy)

        target_idx = nearest_idx + direction * lookahead_steps

        if is_closed:
            target_idx = target_idx % n_wp
        else:
            target_idx = min(max(target_idx, 0), n_wp - 1)

        return waypoints_xy[target_idx]

    def estimate_mean_spacing(self, waypoints_xy):
        if waypoints_xy is None or len(waypoints_xy) < 2:
            return 0.05

        diffs = waypoints_xy[1:] - waypoints_xy[:-1]
        segment_lengths = np.linalg.norm(diffs, axis=1)

        mean_spacing = float(np.mean(segment_lengths))

        if not np.isfinite(mean_spacing) or mean_spacing < 1e-6:
            return 0.05

        return mean_spacing

    def is_path_closed(self, waypoints_xy):
        if waypoints_xy is None or len(waypoints_xy) < 3:
            return False

        diffs = waypoints_xy[1:] - waypoints_xy[:-1]
        segment_lengths = np.linalg.norm(diffs, axis=1)

        mean_spacing = np.mean(segment_lengths)

        if mean_spacing < 1e-6:
            return False

        end_gap = np.linalg.norm(waypoints_xy[-1] - waypoints_xy[0])

        return end_gap < 3.0 * mean_spacing

    def nearest_waypoint(self, position, waypoints):
        if waypoints is None or len(waypoints) == 0:
            return 0

        diff = waypoints - position[None, :]
        distances = np.linalg.norm(diff, axis=1)

        return int(np.argmin(distances))

    def stop_return(self):
        return (
            0.0,
            0.0,
            0.0,
            0.0,
            np.array([0.0, 0.0]),
            0.0,
            0
        )

    def normalize_angle(self, angle):
        return np.arctan2(
            np.sin(angle),
            np.cos(angle)
        )
