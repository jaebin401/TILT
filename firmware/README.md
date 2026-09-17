# TILT Firmware

TILT의 ESP32-S3 펌웨어 구조와 각 디렉토리의 책임을 정의합니다.

현재 단계에서는 모듈의 경계만 먼저 정리하며, ESP-IDF 프로젝트 설정과 C++ 구현은 이후 작업에서 추가합니다.

## Directory Structure

```text
firmware/
├── main/                       # 애플리케이션 진입점과 모듈 조립
├── components/
│   ├── tilt_core/              # 공통 타입과 로봇 설정
│   │   └── include/tilt/core/
│   ├── tilt_motion/            # 모델 기반 보행과 관절 목표 생성
│   │   └── include/tilt/motion/
│   ├── tilt_sts3215/           # STS3215 버스 서보 통신
│   │   └── include/tilt/sts3215/
│   ├── tilt_sg90/              # SG90 PWM 출력
│   │   └── include/tilt/sg90/
│   ├── tilt_safety/            # 명령 검증과 안전 상태 관리
│   │   └── include/tilt/safety/
│   ├── tilt_actuators/         # 액추에이터 명령 분배와 실행
│   │   └── include/tilt/actuators/
│   └── tilt_expression/        # 귀와 팔의 동작 생성
│       └── include/tilt/expression/
└── tests/
    ├── host/                   # 하드웨어 없이 실행하는 테스트
    └── hardware/               # 실제 ESP32와 액추에이터를 사용하는 테스트
```

각 컴포넌트의 공개 헤더는 `include/tilt/<component>/`에 둡니다. 구현 파일과 컴포넌트별 CMake 설정은 해당 컴포넌트의 루트에 배치합니다.

## Component Responsibilities

### `main`

펌웨어의 진입점입니다. 컴포넌트를 초기화하고 실행 태스크를 시작합니다.

하드웨어 제어나 보행 로직을 직접 구현하지 않고, 필요한 모듈을 조립하는 역할만 담당합니다.

### `tilt_core`

다른 컴포넌트가 함께 사용하는 타입과 설정을 정의합니다.

- 관절 식별자
- 관절 목표값과 현재 상태
- 관절별 위치 및 속도 제한
- 오류 코드
- 로봇 전체 설정

ESP-IDF의 UART, PWM 같은 하드웨어 기능에 의존하지 않는 것을 원칙으로 합니다. 각도 단위는 라디안, 시간 단위는 초를 기본으로 사용합니다.

### `tilt_motion`

TILT의 모델 기반 보행을 계산하고 다리 관절의 목표값을 생성합니다.

- LIPM과 같은 단순화된 동역학 모델
- CoM과 ZMP 기준값 계산
- 다리의 순기구학과 역기구학
- 보행 단계와 지지 상태 관리
- swing foot 궤적 생성
- MPC 또는 단순화된 궤적 추종 제어

초기에는 하나의 컴포넌트로 관리하며 구현이 커지면 내부를 `model`, `kinematics`, `planning`, `control` 영역으로 나눕니다. 물리 모델과 제어기의 경계가 충분히 커질 경우 별도 컴포넌트로 분리할 수 있습니다.

이 컴포넌트는 하드웨어를 직접 제어하지 않습니다. 계산 결과를 `JointTarget`으로 출력하고 안전 계층에 전달합니다.

### `tilt_sts3215`

ESP32-S3와 Waveshare Bus Servo Adapter 사이의 UART 통신과 STS3215 프로토콜을 담당합니다.

- 패킷 인코딩과 디코딩
- 서보 위치, 속도, 가속도 명령
- 현재 위치와 상태 읽기
- 통신 오류와 응답 시간 초과 처리

서보의 raw position과 레지스터 주소는 이 컴포넌트 밖으로 노출하지 않습니다.

### `tilt_sg90`

SG90 서보를 구동하기 위한 PWM 출력을 담당합니다.

- 각도와 PWM duty 변환
- 출력 범위 제한
- 귀와 팔에 연결된 채널 관리

표정이나 제스처의 의미는 해석하지 않고, 전달받은 목표 각도만 실행합니다.

### `tilt_safety`

액추에이터로 전달되기 전의 모든 명령을 검증하고 로봇의 안전 상태를 관리합니다.

- 관절 위치 및 속도 제한
- 비정상 수치와 오래된 명령 거부
- 급격한 목표값 변화 제한
- `Disarmed`, `Armed`, `Fault` 상태 관리
- 오류 발생 시 안전 정지와 fault latch

보행, 귀, 팔을 포함한 모든 움직임은 이 계층을 통과해야 합니다.

### `tilt_actuators`

검증된 관절 목표값을 실제 액추에이터 드라이버로 전달합니다.

- 다리 관절은 `tilt_sts3215`로 전달
- 귀와 팔 관절은 `tilt_sg90`으로 전달
- 하드웨어 초기화와 출력 주기 관리
- 액추에이터 상태 수집

여러 태스크가 드라이버를 동시에 호출하지 않도록 실제 하드웨어 접근의 단일 진입점 역할을 합니다.

### `tilt_expression`

귀와 팔을 이용한 자세, 제스처, 동작 시퀀스를 생성합니다.

이 컴포넌트는 PWM을 직접 출력하지 않고 논리적인 관절 목표값을 생성합니다. 생성한 목표값은 안전 검증을 거쳐 `tilt_actuators`로 전달됩니다.

## Command Flow

```text
Gait command                  Expression command
     │                                │
     ▼                                ▼
tilt_motion                  tilt_expression
     │                                │
     └─────────── JointTarget ────────┘
                      │
                      ▼
             tilt_safety validation
                      │
                      ▼
              tilt_actuators routing
                  │           │
                  ▼           ▼
         tilt_sts3215     tilt_sg90
                  │           │
                  ▼           ▼
              Leg servos   Ear / arm servos
```

상위 로직은 하드웨어 드라이버를 직접 호출하지 않습니다. `JointTarget`을 생성하고 안전 계층과 액추에이터 관리 계층을 통해서만 움직임을 요청합니다.

## Dependency Rules

```text
tilt_core
├── tilt_sts3215
├── tilt_sg90
├── tilt_safety
├── tilt_motion
└── tilt_expression

tilt_actuators
├── tilt_core
├── tilt_safety
├── tilt_sts3215
└── tilt_sg90

main
└── application components
```

- `tilt_core`는 다른 TILT 컴포넌트에 의존하지 않습니다.
- 드라이버는 상위 제어 로직에 의존하지 않습니다.
- `tilt_safety`는 특정 액추에이터 드라이버에 의존하지 않습니다.
- 컴포넌트 사이에서는 공개 헤더만 사용합니다.
- 순환 의존성을 만들지 않습니다.

## Tests

### `tests/host`

일반 컴퓨터에서 실행할 수 있는 순수 로직 테스트를 둡니다. 단위 변환, 명령 제한, 상태 전이와 같이 실제 하드웨어가 필요하지 않은 기능을 우선 검증합니다.

### `tests/hardware`

ESP32-S3와 실제 액추에이터가 필요한 검증 코드를 둡니다. 테스트마다 필요한 배선, 전원 조건, 실행 순서와 예상 결과를 함께 기록합니다.

## Planned Extensions

센서 피드백과 iPhone 연동을 구현할 때 다음 컴포넌트를 같은 원칙으로 추가합니다.

- `tilt_estimation`: IMU와 관절 피드백을 이용한 상태 추정
- `tilt_ble`: iPhone 명령 수신과 텔레메트리 전송

새 컴포넌트는 책임과 공개 인터페이스가 정해진 뒤 디렉토리를 추가합니다.
