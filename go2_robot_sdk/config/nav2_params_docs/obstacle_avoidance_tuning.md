# Obstacle Avoidance Tuning Summary (Nav2)

- **Config 파일**: `go2_robot_sdk/config/nav2_params.yaml`
- **문서 위치**: `go2_robot_sdk/config/nav2_params_docs/obstacle_avoidance_tuning.md`
- **목표**
  - 벽/장애물에 **너무 바짝 붙는 주행 완화**
  - 장애물 인지가 늦어서 생기는 **급회피/진동/충돌 위험 감소**
  - 맵에 없는 동적 장애물을 **global planner 단계부터 더 크게 우회**하도록 유도
  - 회피 성능을 올리되, Jetson 환경에서 **계산량/안정성 트레이드오프**를 관리

## 변경 요약(핵심만)

- **Controller(DWB)**
  - `sim_time`을 늘려 더 먼 미래 궤적을 평가 → 회피 시작을 앞당김
  - critic 가중치 조정으로 **장애물 근접 페널티 강화** + **경로/목표 정렬 집착 완화**
- **Local costmap**
  - 창 크기 확대 + footprint padding 증가 → 로컬 회피 시 **여유 거리 확보**
  - `raytrace_max_range`/`obstacle_max_range` 확장 → 장애물/클리어링을 더 멀리 반영
  - inflation을 더 두껍게 → 벽을 따라 스치듯 주행하는 현상 완화
- **Global costmap / Planner**
  - `track_unknown_space: True` + `allow_unknown: true` → 회색(Unknown) 공간을 막지 않고 통과 가능
  - `obstacle_layer` 추가 → 맵에 없는 동적 장애물을 global 단계에서 반영

## 변경 상세 (현재 YAML 값 기준)

아래 표의 **After는 `nav2_params.yaml`에 적힌 현재 값**입니다.

### 1) Controller Server (Local Planner / DWB)

| 파라미터(경로) | After | 의도/효과 | 리스크/재조정 포인트 |
|---|---:|---|---|
| `controller_server.ros__parameters.controller_frequency` | `5.0` | 제어 루프 주기 개선(반응성) | CPU 여유 있으면 상향 가능. 지나치면 CPU/지터↑ |
| `...FollowPath.max_vel_theta` | `2.0` | 과도한 회전 속도 완화(회전 시 안정성) | 좁은 공간에서 회피가 답답하면 상향 |
| `...FollowPath.acc_lim_x` / `decel_lim_x` | `1.5` / `-1.5` | 급가감속으로 인한 slip/진동 완화 | 너무 낮으면 회피가 둔해짐 |
| `...FollowPath.sim_time` | `2.0` | 더 먼 미래 충돌을 평가하여 **조기 회피** | 너무 보수적 우회/정체면 `1.5~2.0`에서 재조정 |
| `...FollowPath.BaseObstacle.scale` | `0.14` | 장애물 근접 페널티 강화 → 벽에 바짝 붙는 현상 감소 | 너무 높으면 회피가 과도해짐 |
| `...FollowPath.PathAlign.scale` | `18.0` | 경로 정렬 집착 완화 → 회피 시 여유 | 너무 낮으면 경로 이탈↑ |
| `...FollowPath.GoalAlign.scale` | `16.0` | 목표 정렬 집착 완화 → 좁은 곳 진동/근접 주행 완화 | goal 근처 정렬이 나빠지면 소폭 상향 |
| `...FollowPath.PathDist.scale` | `22.0` | 경로 거리 비용 완화 → 큰 우회/부드러운 회피 허용 | 너무 낮으면 경로 이탈↑ |
| `...FollowPath.GoalDist.scale` | `20.0` | 목표 직진 성향 완화 → 장애물 앞에서 더 큰 우회 허용 | 목표 직진성이 부족하면 소폭 상향 |

### 2) Local Costmap

| 파라미터(경로) | After | 의도/효과 | 리스크/재조정 포인트 |
|---|---:|---|---|
| `local_costmap...update_frequency` | `5.0` | 로컬 맵 갱신 개선(회피 반응성) | CPU 부하↑ 가능 |
| `local_costmap...width/height` | `8` / `8` | 로컬 맵 창 확대 → 장애물을 더 일찍 반영 | 너무 크면 계산량↑ |
| `local_costmap...footprint_padding` | `0.05` | 최소 이격 확보(벽 스침 감소) | 좁은 문틀 통과가 막히면 `0.03~0.05` |
| `local_costmap...plugins` | `obstacle_layer`, `inflation_layer` | static_layer 제거(센서 기반 회피) | 정적 지도 의존 회피가 필요하면 재검토 |
| `local_costmap...scan.raytrace_max_range` | `8.0` | free-space 클리어링 멀리 반영(잔상 완화) | 센서 노이즈 환경에서는 과도한 clearing 주의 |
| `local_costmap...scan.obstacle_max_range` | `7.0` | 장애물 멀리 반영(조기 회피) | 원거리 오탐 증가 가능 |
| `local_costmap...inflation_layer.cost_scaling_factor` | `2.5` | 비용 감소를 완만하게 → 늦은 회피 완화 | 좁은 공간 통과가 어려우면 `2.5~3.0` |
| `local_costmap...inflation_layer.inflation_radius` | `0.75` | 안전거리 확대(벽 근접 감소) | 통로 막힘이면 `0.65~0.85` |

### 3) Global Costmap

| 파라미터(경로) | After | 의도/효과 | 리스크/재조정 포인트 |
|---|---:|---|---|
| `global_costmap...track_unknown_space` | `True` | Unknown 공간을 인식(Planner와 연동) | Unknown을 막아야 하는 운영 정책이면 주의 |
| `global_costmap...footprint_padding` | `0.05` | global 경로도 벽 근접 생성 감소 | 좁은 통로 막힘이면 `0.03~0.05` |
| `global_costmap...plugins` | `static_layer`, `obstacle_layer`, `inflation_layer` | 동적 장애물 반영(맵에 없는 장애물 회피) | 센서 오탐이 경로에 영향을 더 크게 줌 |
| `global_costmap...scan.raytrace_max_range` | `8.0` | free-space 갱신성 개선 |  |
| `global_costmap...scan.obstacle_max_range` | `7.0` | 더 멀리 장애물을 global에 반영해 크게 우회 |  |
| `global_costmap...inflation_layer.inflation_radius` | `0.85` | global 경로 안전거리 확대 | 너무 크면 경로가 안 생길 수 있음 |

### 4) Planner Server

| 파라미터(경로) | After | 의도/효과 | 리스크/재조정 포인트 |
|---|---:|---|---|
| `planner_server...GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | 비교적 가볍고 일반적인 플래너로 단순화 | 복잡한 동역학 고려는 약함 |
| `planner_server...GridBased.allow_unknown` | `true` | Unknown(회색) 영역도 경로 생성 가능 | Unknown을 위험으로 간주하면 비활성화 필요 |
| `planner_server...GridBased.use_astar` | `true` | A*로 안정적 경로 탐색 | 계산량 증가 가능 |

### 5) AMCL (Localization) — 회피 튜닝과 연동되는 안정성 보강

| 파라미터(경로) | After | 의도/효과 | 리스크/재조정 포인트 |
|---|---:|---|---|
| `amcl.alpha1~5` | `0.01` | odom을 더 신뢰(노이즈 낮춤) | 실제 slip이 크면 반대로 위험 |
| `amcl.max_particles` / `min_particles` | `10000` / `1000` | 대칭 환경에서 튐 완화(분포 확보) | CPU 증가 가능 |
| `amcl.update_min_a` / `update_min_d` | `0.05` / `0.05` | 미세
움직임에 과민 갱신 방지 | 너무 크면 갱신이 늦을 수 있음 |
