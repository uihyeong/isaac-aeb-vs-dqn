# Isaac Sim — 강화학습(DQN) vs 규칙기반(AEB) 동적장애물 회피 비교

> 자율주행 택배 로봇 캡스톤의 **AI 기술 평가** 산출물.
> 같은 Isaac Sim 환경에서 **강화학습(DQN) 회피**가 기존 **규칙기반(AEB+E-stop+장애물회피)** 보다
> 얼마나 더 나은지 정량 비교한다.

---

## 1. 왜 (Motivation)

- 택배 로봇은 복도·엘리베이터 홀에서 **움직이는 사람(동적 장애물)**을 피해 주행해야 한다.
- 기존 방식 **AEB(Autonomous Emergency Braking) + E-stop + OA(Obstacle Avoidance)** 는
  *"장애물을 보면 멈춘다"* 는 **반응형(reactive)** 규칙이다 → 자주 멈칫거리고 느려진다.
- **강화학습(RL)** 은 장애물의 **이동 방향을 관측에 포함(예측형)** 해 *멈추지 않고 미리 피하도록* 학습한다.
- 가설: **"같은 환경에서 RL이 규칙기반보다 매끄럽고 빠르게, 그러면서도 안전하게 회피한다."**
  → 이 레포는 그 가설을 **Isaac Sim에서 실측**으로 검증한다.

## 2. 무엇을 비교했나

| | AEB (규칙기반, 베이스라인) | DQN (강화학습) |
|---|---|---|
| 회피 원리 | 전방 장애물 감지 → E-stop(정지) + OA(조향) | 격자 관측(장애물 이동방향 포함) → 학습된 정책 |
| 코드 | 실제 로봇에 쓰던 ROS2 노드 그대로 | `best_model.zip`(격자 DQN) + pure-pursuit 컨트롤러 |
| 학습 | 불필요(rulebased) | 격자 시뮬레이터에서 사전 학습(성공률 93%) |

**공정성:** 두 방식 모두 **같은 Isaac 씬·같은 경로(~62m 루프)·같은 운동학·같은 장애물 배치(seed)** 에서 겨룬다. 회피 레이어만 다르다.

---

## 3. 결과 요약

### ✅ 동행형 트래픽(장애물이 경로 따라 이동) — **DQN 우위** (밀집도 2·4·6·8명 × 각 5 seed)

![along](figures/dqn_wins_2metrics_5seed.png)

| 밀집 | 성공%(A/D) | **E-stop**(매끄러움) A / D | **시간**(밀집속도,s) A / D |
|:---:|:---:|:---:|:---:|
| 2명 | 100 / 100 | 0.8±1.0 / **0±0** | 51.2±3.0 / 52.9±2.4 |
| 4명 | 100 / 100 | 3.0±2.2 / **0±0** | 56.3±4.2 / **55.1±0.5** |
| 6명 | 100 / 80 | 5.4±3.3 / **0±0** | 60.2±6.2 / **55.0±1.6** |
| 8명 | 100 / 100 | 7.2±3.7 / **0±0** | 66.2±6.7 / **57.2±0.6** |

- **① 주행 매끄러움(E-stop 횟수):** AEB는 밀집할수록 멈칫 급증(0.8→7.2), 분산도 큼. **DQN은 항상 0회(분산 0)**.
- **② 밀집 시 속도(도달 시간):** AEB는 51→66s로 느려짐. **DQN은 ~55s 일관**(표준편차 ±0.5~1.6로 매우 안정) → 4명부터 더 빠름.
- *정직한 한계:* DQN 성공률 95%(6명에서 5번 중 1번 타임아웃). AEB 100%.

### ⚖️ 교차형 트래픽(장애물이 경로를 가로지름) — **충돌 안정성 무승부**

![cross-harsh](figures/collision_safety_harsh.png)

- 가혹 시나리오(사람 0.8 m/s, 교차폭 ±1.1m, 4~10명, 각 5 seed)에서도 **AEB 충돌률 20% vs DQN 15% → 통계적으로 동일**.
- 둘 다 성공 100%, 밀집도 따라 단조 증가 X(타이밍 우연).
- **이유:** DQN은 격자에서 *동행형*으로 학습됨 → *교차형*은 학습 분포 밖이라 AEB 대비 이점 없음.

### 🔬 Ablation — "RL이 실제로 일을 하나?" (pure-pursuit 대조군)

DQN 주행 = **순수 경로추종(pursuit) + RL 회피**. 그래서 회피를 끈 **pursuit-only**와 비교해 RL의 순수 기여를 분리했다 (각 20런).

![ablation](figures/ablation_pursuit_vs_dqn_vs_aeb.png)

| 방식 | 동행 충돌률 | 교차 충돌률 | 동행 E-stop |
|:---:|:---:|:---:|:---:|
| **Pursuit only**(회피 없음) | **35%** | 15% | 0 |
| **DQN**(RL) | **20%** | 15% | 0 |
| **AEB**(규칙) | 10% | 20% | 4.1 |

- **동행형: pursuit 35% → DQN 20%.** RL 회피가 충돌을 **거의 절반으로** 줄인다(E-stop 0 유지) → **DQN의 RL이 실제로 기여함을 입증**.
- **교차형: pursuit ≈ DQN(15%).** 교차는 학습 분포 밖이라 RL 이점 없음.
- **AEB**는 충돌 최소(10%)지만 E-stop 4.1회로 멈칫·느림.

### 결론 (세 방식 종합)

> **AEB = 가장 안전하나 멈칫거림(jerky). Pursuit = 매끄럽지만 위험. DQN = 둘의 균형점**
> — pursuit만큼 매끄럽게(E-stop 0) 주행하면서 RL 회피로 충돌을 줄인다.
> RL의 강점은 *"동행 트래픽에서 매끄러움을 유지하며 충돌을 절반으로"*. (교차형·충돌 최소화 자체는 RL 우위 아님.)

상세 수치·분석: [`ISAAC_COMPARISON_RESULTS.md`](ISAAC_COMPARISON_RESULTS.md) · 격자 DQN 학습: [`model/DQN_RESULTS.md`](model/DQN_RESULTS.md)

---

## 4. 어떻게 (System & 재현)

```
[Isaac Sim 5.1, py3.11]  aeb_scene.py
  · map_5floor 벽 + Scout Mini(kinematic) + 사람 N명(캡슐) + 후방 RPLidar_S2E
  · /scan /tf /clock 발행, cmd_vel대로 로봇 운동학 이동 (AEB·DQN 공통 모델)
        │ /scan,/tf                              ▲ /cmd_vel(파일경유)
        ▼                                        │
[AEB, py3.10] run_aeb.sh → aeb_nodes/      [DQN, venv py3.10] dqn_drive.py
  e_stop·aeb·oa·path_follower → /cmd_vel     best_model.zip + pure-pursuit → /cmd_vel
        │                                        │
        └──────────► [측정] metrics_node.py ◄──────┘
                     /tf·/scan·/e_stop → 성공·시간·충돌·E-stop·이격 JSON
```

**실행 (요약):**
```bash
# 1) 씬 (밀집도·장애물모드 환경변수)
AEB_NUM_PEOPLE=6 AEB_OBSTACLE_MODE=along \
  ~/isaac_sim/python.sh aeb_scene.py --headless --sx 3.65 --sy -1.04 --syaw -1.25
# 2) cmd_vel 브리지 (AEB용)
python3 cmdvel_to_file.py
# 3) 단일 측정
bash run_measure.sh aeb 0      # 또는  dqn 0
# 4) 밀집도×시드 스윕 (mean±std)
DENSITIES="2 4 6 8" NSEED=5 SWEEP_TAG=along_ AEB_OBSTACLE_MODE=along bash run_sweep.sh
```

**핵심 환경변수:** `AEB_OBSTACLE_MODE`(along/cross) · `AEB_NUM_PEOPLE` · `AEB_PERSON_SPEED` · `AEB_CROSS_HALF` · `AEB_PEOPLE_SEED`

**의존성:** DQN은 `model/grid_nav_env.py` + `model/best_model.zip`(격자 정책) 필요. 컨트롤러는 `~/grid_nav/`를 참조하므로 재현 시 경로 확인.

---

## 5. 파일 구조

```
aeb_scene.py        Isaac 씬 (벽·로봇·사람·LiDAR·운동학 구동)  ★핵심
dqn_drive.py        DQN 컨트롤러 (격자정책 + pure-pursuit + anti-orbit)
metrics_node.py     지표 수집 (성공·시간·충돌·E-stop·이격 → JSON)
cmdvel_to_file.py   /cmd_vel → 파일 브리지 (py버전 우회)
run_measure.sh      단일 측정 러너
run_sweep.sh        밀집도×시드 스윕 러너
run_aeb.sh          AEB 4노드 런처
aeb_nodes/          AEB 원본 ROS2 노드 + waypoints
model/              best_model.zip(DQN) + grid_nav_env.py + DQN_RESULTS.md
figures/            결과 그래프 PNG
results/            측정 JSON 원자료 (밀집도·시드별)
ISAAC_COMPARISON_RESULTS.md   상세 결과·분석·엔지니어링 노트
```

---

## 6. 한계 & 다음 작업

- **(완료) pure-pursuit 단독 대조군** — RL이 동행형 충돌을 35%→20%로 줄임을 입증(위 Ablation).
- 다음: 혼합 트래픽(동행+교차), 통계 유의성 검정(p-value), DQN 교차형 재학습.
- 측정 한계: 드물게(스윕 ~5%) 지표 노드가 scan 미수신(clr=9.9) → 해당 run 충돌 누락 가능.

---

## 7. 엔지니어링 메모 (핵심 난관 해결)

Isaac에서 cmd_vel대로 로봇을 움직이는 데 3번의 시도 끝에 해결:

| 방식 | TF 동기 | 안정성 |
|---|:---:|:---:|
| 동적 articulation + set_world_pose | ✅ | ❌ PhysX broadphase NaN |
| kinematic + 부모 xform 이동 | ❌ desync | ✅ |
| **kinematic RigidPrim + set_world_pose** ← 채택 | ✅ | ✅ |

상세는 [`ISAAC_COMPARISON_RESULTS.md`](ISAAC_COMPARISON_RESULTS.md) §4 참조.
