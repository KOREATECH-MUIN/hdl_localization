# hdl_localization

NDT_OMP 작동 확인
- Thread 조절기능 추가
- CPU사용량 30~40% (i7 - 9750H 기준)

NDT_CUDA_P2D 작동 확인
- CPU사용량 10~15% (i7 - 9750H, GTX1660Ti 기준)

추가 확인요소
NDT_CUDA의 수렴기능 확인하기 (현재 스캔매칭이 수렴되지 않는 경우가 잦음)
IMU데이터사용이 정합적인지 확인 (현재 IMU데이터를 사용하면 오히려 결과가 더 안좋음)
