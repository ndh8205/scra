# KARI Dual-Arm Space Manipulator - Gazebo Simulation

## 프로젝트 개요

Free-floating 위성에 장착된 7-DOF 듀얼 로봇팔 시뮬레이션 환경 구축.
- 플랫폼: WSL2 / Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic
- 목적: 임피던스 제어, 협조 제어, 도킹 시뮬레이션

---

## 디렉토리 구조

```
~/space_ros_ws/src/orbit_sim/
├── urdf/
│   ├── kari_arm.urdf              # 싱글암 (기존)
│   └── kari_dual_arm.urdf         # 듀얼암
├── models/
│   ├── kari_arm/                  # 싱글암 모델
│   │   ├── model.sdf
│   │   ├── model.config
│   │   └── meshes/
│   │       ├── ASM_J0.STL ~ ASM_J7.STL
│   └── kari_dual_arm/             # 듀얼암 모델
│       ├── model.sdf
│       ├── model.config
│       └── meshes/
│           ├── satellite_body.stl
│           ├── satellite_panel.stl
│           └── satellite_front.stl
├── worlds/
│   ├── kari_arm.sdf               # 싱글암 월드
│   └── kari_dual_arm.sdf          # 듀얼암 월드
├── config/
│   ├── kari_arm_controllers.yaml
│   └── kari_dual_arm_controllers.yaml
└── launch/
    ├── kari_arm.launch.py
    └── kari_dual_arm.launch.py
```

---

## 위성 모델 상세

### STL 파일 바운딩 박스

| 파일 | X (min~max) | Y (min~max) | Z (min~max) |
|------|-------------|-------------|-------------|
| satellite_body.stl | -1.775 ~ +1.195 | -1.310 ~ +1.310 | -1.512 ~ +1.512 |
| satellite_panel.stl | -1.975 ~ +0.824 | -4.250 ~ +4.250 | -4.250 ~ +4.250 |
| satellite_front.stl | +1.175 ~ +1.975 | -0.900 ~ +0.900 | -0.900 ~ +0.900 |

### 물리 파라미터

```
질량: 500 kg
관성 텐서 [kg·m²]:
  [260.0  -0.2   0.6]
  [-0.2  280.0   4.0]
  [ 0.6    4.0 170.0]
```

### Collision 설정 (model.sdf)

```xml
<!-- Body (본체) -->
<collision name="satellite_body_collision">
  <pose>-0.29 0 0 0 0 0</pose>
  <geometry>
    <box><size>2.97 2.62 3.02</size></box>
  </geometry>
</collision>

<!-- Panel Left (+Y) -->
<collision name="satellite_panel_L_collision">
  <pose>0 3.05 0 0 0 0</pose>
  <geometry>
    <box><size>2.0 3.05 0.05</size></box>
  </geometry>
</collision>

<!-- Panel Right (-Y) -->
<collision name="satellite_panel_R_collision">
  <pose>0 -3.05 0 0 0 0</pose>
  <geometry>
    <box><size>2.0 3.05 0.05</size></box>
  </geometry>
</collision>

<!-- Front (전방부) -->
<collision name="satellite_front_collision">
  <pose>1.575 0 0 0 0 0</pose>
  <geometry>
    <box><size>0.8 1.8 1.8</size></box>
  </geometry>
</collision>
```

### Visual 색상

| 부위 | 색상 | RGBA |
|------|------|------|
| Body (본체) | 금색 (MLI) | 0.85, 0.65, 0.13, 1 |
| Panel (태양전지판) | 어두운 파란색 | 0.1, 0.2, 0.5, 1 |
| Front (전방부) | 흰색 | 0.95, 0.95, 0.95, 1 |

---

## 듀얼암 구조

### 장착점

| 팔 | 위치 [m] | 회전 (roll) | 설명 |
|----|----------|-------------|------|
| Left (+Y) | [1.9, 0.9, 0] | -90° (-1.5708 rad) | 로봇팔 Z축 → 위성 +Y |
| Right (-Y) | [1.9, -0.9, 0] | +90° (+1.5708 rad) | 로봇팔 Z축 → 위성 -Y |

### Joint 구조 (각 팔 동일)

```
satellite_body
    └── arm_[L/R]_base (fixed: arm_[L/R]_mount)
        └── arm_[L/R]_link1 (revolute: arm_[L/R]_joint1, Z축)
            └── arm_[L/R]_link2 (revolute: arm_[L/R]_joint2, Y축)
                └── arm_[L/R]_link3 (revolute: arm_[L/R]_joint3, Z축)
                    └── arm_[L/R]_link4 (revolute: arm_[L/R]_joint4, Y축)
                        └── arm_[L/R]_link5 (revolute: arm_[L/R]_joint5, Z축)
                            └── arm_[L/R]_link6 (revolute: arm_[L/R]_joint6, Y축)
                                └── arm_[L/R]_link7 (revolute: arm_[L/R]_joint7, Z축)
                                    └── arm_[L/R]_eef_link (fixed)
```

### Joint 오프셋 (로봇팔 로컬 좌표계)

| Joint | 오프셋 [m] |
|-------|-----------|
| joint1 | [0, 0, 0.094] |
| joint2 | [0, 0.088, 0.105] |
| joint3 | [0, 0.104, 0.131] |
| joint4 | [0, -0.088, 0.8] |
| joint5 | [0, -0.104, 0.131] |
| joint6 | [0, -0.071, 0.408] |
| joint7 | [0, -0.121, 0.088] |
| eef | [0, 0, 0.097] |

### Joint 한계

| Joint | 토크 [Nm] | 속도 [rad/s] | 범위 [rad] |
|-------|----------|--------------|-----------|
| 1~4 | 65 | 0.17453 | ±3.14 |
| 5~7 | 65 | 0.17453 | ±3.14 |

---

## 컨트롤러 설정

### kari_dual_arm_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    effort_controller_L:
      type: effort_controllers/JointGroupEffortController
    effort_controller_R:
      type: effort_controllers/JointGroupEffortController

effort_controller_L:
  ros__parameters:
    joints:
      - arm_L_joint1
      - arm_L_joint2
      - arm_L_joint3
      - arm_L_joint4
      - arm_L_joint5
      - arm_L_joint6
      - arm_L_joint7

effort_controller_R:
  ros__parameters:
    joints:
      - arm_R_joint1
      - arm_R_joint2
      - arm_R_joint3
      - arm_R_joint4
      - arm_R_joint5
      - arm_R_joint6
      - arm_R_joint7
```

---

## World 설정 (우주 환경)

### kari_dual_arm.sdf

```xml
<!-- 무중력 -->
<gravity>0 0 0</gravity>

<!-- 태양광 (정면 위에서) -->
<light type="directional" name="sun">
  <diffuse>1 1 1 1</diffuse>
  <direction>-1 0 -0.3</direction>
</light>

<!-- 지구 반사광 (약한 파란빛) -->
<light type="directional" name="earth_reflection">
  <diffuse>0.1 0.1 0.15 1</diffuse>
  <direction>0 0 1</direction>
</light>

<!-- 우주 배경 -->
<scene>
  <background>0 0 0 1</background>
  <grid>false</grid>
</scene>
```

---

## 실행 방법

### 빌드

```bash
cd ~/space_ros_ws
colcon build --symlink-install --packages-select orbit_sim
source install/setup.bash
```

### 실행

```bash
ros2 launch orbit_sim kari_dual_arm.launch.py
```

### 토크 제어 테스트

```bash
# Left arm joint2에 10Nm
ros2 topic pub /effort_controller_L/commands std_msgs/msg/Float64MultiArray "{data: [0, 10, 0, 0, 0, 0, 0]}" --once

# Right arm joint2에 10Nm
ros2 topic pub /effort_controller_R/commands std_msgs/msg/Float64MultiArray "{data: [0, 10, 0, 0, 0, 0, 0]}" --once

# 토크 초기화
ros2 topic pub /effort_controller_L/commands std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0, 0]}" --once
ros2 topic pub /effort_controller_R/commands std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0, 0]}" --once
```

### 토픽 확인

```bash
# Joint 상태
ros2 topic echo /joint_states

# 위성 pose
ros2 topic echo /world/kari_dual_arm_world/pose/info
```

---

## 문제 해결 내역

### 1. 위성 드리프트 문제

**원인:** 로봇팔 장착점 pose 누락 → 두 팔이 원점(중앙)에서 겹쳐서 충돌

**해결:**
```xml
<joint name="arm_L_mount" type="fixed">
  <pose relative_to="satellite_body">1.9 0.9 0 -1.5708 0 0</pose>
  ...
</joint>
```

### 2. Trimesh 충돌 오류

**증상:**
```
ODE Message 2: Trimesh-trimesh contact hash table bucket overflow
ODE INTERNAL ERROR 1: assertion failed
```

**원인:** 
- 위성 collision box가 너무 커서 로봇팔 base와 겹침
- Gazebo Harmonic에서 `<disable_collisions>` 미지원

**해결:**
- 위성 collision을 body/panel/front로 분리
- Front collision이 로봇팔 장착부와 겹치지 않도록 크기 조정

### 3. disable_collisions 미지원

**증상:**
```
Warning: XML Element[disable_collisions], child of element[model], not defined in SDF
```

**상태:** Gazebo Harmonic에서 무시됨. 현재는 collision 분리로 해결.

---

## 파일 수정 명령어 모음

### Collision 수정

```bash
# Body collision
sed -i '/<collision name="satellite_body_collision">/,/<\/collision>/c\
      <collision name="satellite_body_collision">\
        <pose>-0.29 0 0 0 0 0</pose>\
        <geometry>\
          <box><size>2.97 2.62 3.02</size></box>\
        </geometry>\
      </collision>' ~/space_ros_ws/src/orbit_sim/models/kari_dual_arm/model.sdf

# Front collision
sed -i '/<collision name="satellite_front_collision">/,/<\/collision>/c\
      <collision name="satellite_front_collision">\
        <pose>1.575 0 0 0 0 0</pose>\
        <geometry>\
          <box><size>0.8 1.8 1.8</size></box>\
        </geometry>\
      </collision>' ~/space_ros_ws/src/orbit_sim/models/kari_dual_arm/model.sdf
```

### 조명 수정

```bash
# 태양 방향 (정면 위에서)
sed -i 's/<direction>.*<\/direction>/<direction>-1 0 -0.3<\/direction>/g' ~/space_ros_ws/src/orbit_sim/worlds/kari_dual_arm.sdf
```

---

## 다음 단계

1. [ ] 임피던스 제어 노드 구현 (듀얼암 버전)
2. [ ] 타겟 위성 모델 추가
3. [ ] 양팔 협조 제어
4. [ ] 도킹 시뮬레이션

---

## 참고 파일 경로

| 항목 | 경로 |
|------|------|
| URDF | ~/space_ros_ws/src/orbit_sim/urdf/kari_dual_arm.urdf |
| Model SDF | ~/space_ros_ws/src/orbit_sim/models/kari_dual_arm/model.sdf |
| World SDF | ~/space_ros_ws/src/orbit_sim/worlds/kari_dual_arm.sdf |
| Controllers | ~/space_ros_ws/src/orbit_sim/config/kari_dual_arm_controllers.yaml |
| Launch | ~/space_ros_ws/src/orbit_sim/launch/kari_dual_arm.launch.py |
| 위성 STL | ~/space_ros_ws/src/orbit_sim/models/kari_dual_arm/meshes/ |
| 로봇팔 STL | ~/space_ros_ws/src/orbit_sim/models/kari_arm/meshes/ |
| MATLAB 원본 | D:\pj2025\space_challenge\ |
