# ae_hyu_monitor

## 패키지 설명
- **차량 주행 상태 모니터링**을 위한 패키지
- **RViz OverlayText 플러그인을 사용**하여 주요 주행 데이터를 화면에 표시

## 실행 방법

### 시뮬레이션 환경
```bash
ros2 launch ae_hyu_monitor monitor_launch.py mod:=sim
```

### 실차 환경
```bash
ros2 launch ae_hyu_monitor monitor_launch.py mod:=real
```

### RViz 설정 (!!!중요!!!)
- RViz 좌상단 File/Open Config 클릭
- **ae_hyu_monitor.rviz** 선택

## 시각화 요소들

### 1. 차량 속도 (Vehicle Speed)
- 현재 차량의 주행 속도 표시
- 단위: m/s

### 2. 랩 정보 (Lap Information)
- 현재 랩 수, 랩  타임
- 최단 랩 타임
- 평균 랩 타임

### 4. CTE (Cross Track Error)
- Current CTE: 현재 횡방향 오차
- Max CTE: 최대 횡방향 오차
- Mean CTE: 평균 횡방향 오차


### 5. 장애물 정보 (Obstacle Information)
- Static: 정적 장애물 감지 여부
- Dynamic: 동적 장애물 감지 여부

## 토픽 구독
- `/ego_racecar/odom` (시뮬레이션) 또는 `/odom` (실차): 차량 상태 정보
- `/car_state/frenet/odom`: Frenet 좌표계 차량 상태 (랩 카운팅, CTE 계산용)
- `/global_waypoints`: 글로벌 웨이포인트 (트랙 길이 계산용)
- `/tracking/obstacles`: 장애물 정보

## 토픽 발행
- `/monitor/lap_info`: 랩 정보 시각화 (OverlayText)
- `/monitor/current_speed`: 현재 속도 정보 (Float32)
- `/monitor/mean_lap_time`: 평균 랩 타임 시각화 (OverlayText)
- `/monitor/fastest_lap_time`: 최단 랩 타임 시각화 (OverlayText)
- `/monitor/mean_cte`: 평균 CTE 시각화 (OverlayText)
- `/monitor/max_cte`: 최대 CTE 시각화 (OverlayText)
- `/monitor/current_cte`: 현재 CTE 시각화 (OverlayText)
- `/monitor/obstacle_info`: 장애물 정보 시각화 (OverlayText)
