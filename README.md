# F1TENTH Capstone Workspace — 통합 뼈대 (race_stack 구조 참고)

ForzaETH/race_stack(https://github.com/ForzaETH/race_stack)의 모듈 분리 방식을 참고해서,
"매핑 / 웨이포인트(경로) / 컨트롤러 / 차량 인터페이스"를 완전히 분리된 ROS2 패키지로 나눴습니다.
race_stack 전체를 이식하지 않고, **인터페이스 계약(토픽 이름 · 메시지 타입)만 동일한 구조로 가져와**
학부 수준에서 이해·유지보수 가능한 최소 구성으로 새로 짰습니다.

## 패키지 구성 (race_stack 대응)

| 이 워크스페이스 | race_stack 대응 모듈 | 담당 |
|---|---|---|
| `f1tenth_maps` | (map 파일 자체, base_system 일부) |  |
| `f1tenth_waypoints` | `planner` | 
| `f1tenth_controller_interface` | `controller` (MPPI) | 
| `f1tenth_vehicle_interface` | `base_system`, `sensors` | 
| `f1tenth_bringup` | `stack_master` | 전체 통합용 launch |

## 핵심 아이디어: 시뮬/실차는 "입출력단"만 다르다

```
[map_server]  →  [localization: slam_toolbox(localization mode)]
       ↓                         ↓
       └──────────→ map -> odom TF 발행 ──────────┐
                                                    ↓
[f1tenth_waypoints] --(/global_path: nav_msgs/Path)--> [controller (MPPI)] --(/drive: AckermannDriveStamped)--> [vehicle_interface]
       ↑ 오프라인 1회 생성                                                                              ↑ sim:=true → 시뮬레이터 노드
       (센터라인 추출 스크립트)                                                                          sim:=false → 실제 VESC/Jetson 노드
```

`f1tenth_bringup/launch/bringup.launch.py` 하나에서 `sim:=true/false` 인자만 바꾸면
map → localization → waypoints → controller 는 100% 동일한 노드가 재사용되고,
맨 끝의 vehicle_interface만 스왑됩니다. 이게 "매핑 따로 시뮬 따로"를 없애는 핵심입니다.

## 토픽/메시지 계약 (합의할 것)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 실차: Hokuyo 드라이버 / 시뮬: 시뮬레이터 | controller |
| `/odom` | `nav_msgs/Odometry` | localization (slam_toolbox or ekf) | controller, waypoints |
| `/global_path` | `nav_msgs/Path` | f1tenth_waypoints (waypoint_loader) | controller |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | controller (MPPI, 조원 작업) | vehicle_interface |
| `map -> odom -> base_link` | TF | localization | 전체 |

컨트롤러를 짜는 조원에게는 "`/scan`, `/odom`, `/global_path` 구독 → `/drive` 퍼블리시"
이 계약만 지키면 된다고 전달하면 됩니다. 내부 MPPI 구현은 완전히 독립적으로 개발 가능합니다.

## 실행 순서 (우선순위)

1. `f1tenth_maps/maps/`에 기존에 만든 pgm/yaml을 넣는다.
2. `f1tenth_waypoints/f1tenth_waypoints/extract_centerline.py`로 맵에서 센터라인 waypoint csv를 1회 생성한다.
3. `f1tenth_bringup/launch/bringup.launch.py sim:=true`로 RViz/시뮬에서 waypoint_loader + (더미)controller 통합 테스트.
4. 조원이 만든 MPPI를 `f1tenth_controller_interface` 계약에 맞춰 연결.
5. `sim:=false`로 실차 저속 테스트 → 속도/장애물 회피 단계적으로 확장.

## 아직 채워야 할 것 (TODO)

- [ ] `f1tenth_maps/maps/`에 실제 pgm/yaml 복사
- [ ] `extract_centerline.py`로 실제 트랙 waypoint 생성 및 검증
- [ ] `f1tenth_controller_interface/controller_node_template.py` → 조원의 MPPI 로직으로 교체
- [ ] `f1tenth_vehicle_interface`에 실제 VESC/조향 드라이버 연결 (f1tenth_system 참고)
- [ ] 시뮬레이터 노드 연결 (f1tenth_gym_ros 또는 기존 사용 중인 시뮬 그대로 사용 가능)
