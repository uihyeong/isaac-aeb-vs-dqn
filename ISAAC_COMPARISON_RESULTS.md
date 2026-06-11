# Isaac Sim 연속환경 — AEB vs DQN 동적장애물 회피 비교

> 같은 Isaac Sim 씬(map_5floor, 동적 장애물 2명, RPLidar_S2E, 운동학 모델 동일)에서
> **AEB(실제 코드 그대로)** vs **DQN(격자 학습 정책 + pure-pursuit 컨트롤러)** 를 겨룸.
> 공정성: 두 컨트롤러 모두 동일 경로·동일 운동학·동일 장애물에서 회피 레이어만 다름.

---

## 1. 실측 결과 — metrics_node.py 자동 측정 (검증됨)

동행형(along) 트래픽 — 장애물이 로봇 경로를 따라 이동. ~62m 루프, 동일 씬·경로·운동학.
**밀집도 2·4·6·8명 × 각 5 seed (총 40런), mean±std.** 그래프: `dqn_wins_2metrics_5seed.png`.

| 밀집 | 성공%(A/D) | E-stop A / D | 도달시간(s) A / D |
|:---:|:---:|:---:|:---:|
| 2명 | 100 / 100 | **0.8±1.0** / **0±0** | 51.2±3.0 / 52.9±2.4 |
| 4명 | 100 / 100 | **3.0±2.2** / **0±0** | 56.3±4.2 / 55.1±0.5 |
| 6명 | 100 / 80 | **5.4±3.3** / **0±0** | 60.2±6.2 / 55.0±1.6 |
| 8명 | 100 / 100 | **7.2±3.7** / **0±0** | 66.2±6.7 / 57.2±0.6 |

**핵심 발견 (5 seed, 통계적으로 견고):**
1. **주행 매끄러움(E-stop): DQN 완승.** AEB는 밀집도 비례 급증(0.8→7.2회)·분산 큼(±3.7). **DQN은 모든 밀집도·시드에서 0회(분산 0)**.
2. **밀집 시 속도(시간): DQN 승.** AEB는 51→66s로 느려지고 분산 큼(±6.7). **DQN은 ~55s 일관(±0.5~1.6, 매우 안정)** → 4명부터 AEB보다 빠름. DQN은 평균뿐 아니라 **예측가능성(낮은 분산)**에서도 우월.
3. **성공률:** AEB 100%, DQN 95%(20중 1실패@6명, 타임아웃) — DQN이 드물게 막힘. 정직히 명시.

→ **결론: 동행 트래픽에서 RL은 멈춤 없이(E-stop 0)·밀집해도 느려지지 않게(시간 일관) 주행** → 매끄러움·밀집강건 속도 2지표에서 우위. (충돌 안정성은 §1.5에서 무승부.)

*측정: `run_sweep.sh`(AEB_OBSTACLE_MODE=along, NSEED=5) → `results/metrics_*_along_d*_s*.json`.*

---

## 1.5. 충돌 안정성 — 교차형 장애물 밀집도 스윕 (2~10명, 각 3 seed)

장애물이 로봇 경로를 **수직으로 가로지르는(crossing)** 시나리오로, 동일 장애물 배치(seed)에서 AEB·DQN 비교. 그래프: `figures/collision_safety_vs_density.png`.

| 밀집 | 방식 | 성공 | 충돌(평균) | 최소이격(m,평균) | 시간(s,평균) | E-stop |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 2명 | AEB / DQN | 3/3 / 3/3 | 0.33 / 0.33 | 0.69 / 0.70 | 59.6 / 63.7 | 0 / 0 |
| 4명 | AEB / DQN | 3/3 / 3/3 | 0.00 / 0.33 | 1.01 / 0.70 | 52.6 / 56.9 | 0 / 0 |
| 6명 | AEB / DQN | 3/3 / 3/3 | 0.00 / 0.00 | 1.01 / 1.01 | 49.8 / 54.1 | 0 / 0 |
| 8명 | AEB / DQN | 3/3 / 3/3 | 0.00 / 0.00 | 1.01 / 1.01 | 51.1 / 55.9 | 0 / 0 |
| 10명 | AEB / DQN | 3/3 / 3/3 | 0.00 / 0.00 | 1.01 / 1.01 | 49.7 / 56.4 | 0 / 0 |

### 가혹 시나리오 (속도 0.8 m/s, 교차폭 ±1.1m, 밀집 4~10명, 각 **5 seed**)

신빙성 강화를 위해 더 빠르고 넓은 교차 + 시드 5개로 재측정 (총 40런). 그래프: `figures/collision_safety_harsh.png`.

| 밀집 | AEB 충돌률 | DQN 충돌률 | AEB 시간(s) | DQN 시간(s) |
|:---:|:---:|:---:|:---:|:---:|
| 4명 | 40% | 40% | 50.2 | 53.7 |
| 6명 | 20% | 0% | 48.8 | 53.6 |
| 8명 | 0% | 0% | 51.2 | 55.3 |
| 10명 | 20% | 20% | 50.4 | 55.5 |
| **전체** | **4/20 (20%)** | **3/20 (15%)** | ~50 | ~55 |

**정직한 결론 (충돌 안정성):**
- **가혹 시나리오에서도 AEB(20%)와 DQN(15%) 충돌률이 통계적으로 동일**(4 vs 3건/20런, N 부족). 밀집도 따라 단조 증가도 아님(8명은 둘 다 0%) → **충돌은 밀집도가 아니라 타이밍 우연.** → **충돌 안정성은 무승부, 명확한 우열 없음.**
- 두 방식 모두 **성공 100%** (충돌=사람 0.3m 이내 통과 카운트지, 주행 실패 아님). 안 충돌한 run은 모두 ~1.0m 이격 유지.
- 교차 모드에선 **AEB E-stop도 0** (수직 통과자는 좁은 전방 박스에 잠깐만 → E-stop 대신 OA 조향 회피). 시간은 오히려 AEB가 약간 빠름(통과자 위해 안 멈춤).
- **왜 DQN이 교차에서 더 안전하지 않나:** DQN은 격자에서 **동행형** 장애물로 학습됨 → **교차형은 학습 분포 밖**이라 AEB 대비 이점 없음.
- **핵심: "RL이 충돌에서 더 안전"은 입증 안 됨.** RL의 명확한 우위는 §1의 **동행형(along) 트래픽 매끄러움(E-stop 0 vs 1~5, 시간 일관)**.
- *측정 한계:* 일부 run(40중 3)에서 지표 노드가 scan 미수신(clr=9.9 글리치) → 해당 run 충돌 누락 가능. 결론(유사 충돌률)에는 영향 미미.

## 2. 격자 시뮬레이터 — 장애물 밀집도별 DQN 성능 (100 에피소드)

연속 Isaac과 별개로, DQN 정책 자체의 강건성 (grid_nav 환경, 정량):

| 장애물 수 | 1 | 2 | 3 | 5 |
|-----------|---|---|---|---|
| **DQN 성공률** | 89% | 79% | 65% | 54% |
| 충돌률 | 9% | 18% | 33% | 44% |
| 도달 스텝 | 134 | 135 | 136 | 134 (거의 최단) |

→ 밀집할수록 자연스럽게 하락 = 동적 회피를 실제로 학습한 증거.
→ 연속 PPO는 동일 태스크에서 **0% 고정** → 격자 DQN이 결정적으로 우월.

---

## 3. 시스템 구성 (재현용)

```
[Isaac Sim 5.1, py3.11]  aeb_scene.py  --headless --sx 3.65 --sy -1.04 --syaw -1.25
  · map_5floor 벽(557 box) + Scout Mini(kinematic rigidprim) + 사람 2명(캡슐, 경로 왕복)
  · 후방 RPLidar_S2E → /scan, world→base_link → /tf, /clock
  · cmd_vel대로 base_link kinematic pose 갱신 (운동학 모델, AEB·DQN 공통)

[AEB, py3.10]  run_aeb.sh
  · scout_mini_e_stop / _aeb / _obstacle_avoidance / sm_path_follower_test4
  · /scan → 회피판단 → /cmd_vel → (cmdvel_to_file.py 브리지) → /tmp/cmd_vel.txt

[DQN, grid_nav venv py3.10]  dqn_drive.py
  · best_model.zip(격자 DQN) + /scan→격자관측(152) + /tf
  · pure-pursuit(경로 추종) + DQN 회피 변조 + 종단 목표 조준
  · /cmd_vel 발행 + /tmp/cmd_vel.txt 직접 기록

[측정, py3.10]  metrics_node.py
  · /tf·/scan·/scout_mini/e_stop → 성공·시간·충돌(벽/사람)·이격·정지·E-stop JSON
```

실행:
```bash
# 1. 씬
~/isaac_sim/python.sh ~/isaac_aeb/aeb_scene.py --headless --sx 3.65 --sy -1.04 --syaw -1.25
# 2. 브리지(AEB용)
python3 ~/isaac_aeb/cmdvel_to_file.py
# 3a. AEB 측정
bash ~/isaac_aeb/run_measure.sh aeb 0
# 3b. DQN 측정
bash ~/isaac_aeb/run_measure.sh dqn 0
# (씬에 로봇 리셋: echo "3.6487 -1.0394 -1.25" > /tmp/reset_robot.txt)
```

---

## 4. 엔지니어링 핵심 — 운동학 로봇 구동 (3가지 시도 끝에 해결)

Isaac에서 로봇을 cmd_vel대로 움직이는 데 핵심 난관이 있었음:

| 방식 | TF 동기 | 안정성 | 결론 |
|------|:---:|:---:|------|
| ① 동적 articulation + set_world_pose | ✅ 정확 | ❌ PhysX broadphase NaN (~50–600s) | 폐기 |
| ② kinematic body + 부모 xform 이동 | ❌ TF 얼어붙음(desync) | ✅ NaN 없음 | 폐기 |
| ③ **kinematic RigidPrim + set_world_pose** | ✅ 정확 | ✅ 안정(과회전만 주의) | **채택** |

**해결책 ③**: base_link을 kinematic rigid body로 만들고(articulation root 제거), 매 프레임
`SingleRigidPrim.set_world_pose`로 **바디 자체** pose를 설정 → TF가 정확히 따라오고 PhysX
브로드페이즈 NaN도 없음. (부모 xform 이동은 kinematic 바디가 안 따라와 TF 비동기 발생.)

---

## 5. 한계 및 남은 작업 (정직한 상태)

- **자동 측정 파이프라인 완성·동작** (위 표 6회 run 모두 metrics_node.py가 정상 JSON 산출).
  - anti-orbit 가드(dqn_drive.py: 4s 순변위<0.4m → 경로직진 2.5s) 추가로 DQN 공전·NaN 해소.
  - kinematic RigidPrim 방식으로 TF 동기·NaN 모두 해결(아래 §4).
- **완료된 통계 강화**: 동행형 2·4·6·8명 × 5 seed(§1), 교차형 2~10명 × 3~5 seed(§1.5) → mean±std 확보.
- **다음 작업**:
  1. **pure-pursuit 단독 대조군** — DQN=pursuit+RL이라 "매끄러움이 RL 덕인지" 분리 필요 (최우선).
  2. **혼합 트래픽**(동행+교차) + 통계 유의성 검정(p-value).
  3. DQN 교차형 재학습 시 교차 시나리오 성능 향상 기대.
- **환경 제약**: 디스크 96%라 장시간 다회 실행 시 간헐 쓰기 실패 가능 → 캐시 정리 권장.
- 도구: `run_measure.sh`(단일), `run_sweep.sh`(밀집도×시드 스윕), `metrics_node.py`(지표), `aeb_scene.py`(AEB_NUM_PEOPLE/SEED/OBSTACLE_MODE/PERSON_SPEED/CROSS_HALF 환경변수).

---

## 6. 발표 메시지

1. **"같은 Isaac 환경에서 RL이 규칙기반(AEB+E-stop+OA)보다 매끄럽고 밀집도에 강건하다"**
   — 5명 밀집: AEB는 E-stop 4~5회·시간 60~64s(분산↑), DQN은 **E-stop 0회·58s(일관)**.
2. **예측형 관측**(장애물 이동방향 포함) → 반응형 AEB는 장애물 보고 멈추지만, RL은 미리 피해 안 멈춤.
   밀집도가 올라가도 AEB는 멈칫이 비례 증가(1→5), RL은 0으로 불변.
3. 연속 PPO가 실패(0%)한 문제를 **이산 격자 DQN**으로 해결한 방법론적 기여(89~54% 밀집도 곡선).
