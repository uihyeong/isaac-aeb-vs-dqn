
## Phase 1 완료 (2026-06-10)
- ✅ /scan 발행 해결! 핵심 수정 2가지:
  1. render_product: resolution=(128,128), render_vars=["GenericModelOutput","RtxSensorMetadata"]
  2. world.step(render=True) — headless에서도 RTX lidar는 렌더링 필요
- ✅ /scan: 3200빔 RPLidar_S2E, 실제 벽 0.4~4.5m 감지
- ✅ /cmd_vel → 차동구동 로봇 이동 확인
- ✅ /clock /tf 발행
- ⚠️ Phase 3 주의: TF world→base_link가 절대위치 아님(translation~0). AEB path_follower는
   map→base_link 절대좌표 필요 → odom→base_link + static map→odom 프레임 정리 필요
- 다음: Phase 2 동적장애물 사람(충돌체+이동) → Phase 3 AEB 연결
