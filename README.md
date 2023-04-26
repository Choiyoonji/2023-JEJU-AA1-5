# JEJU AA 1/5
2023 국제 대학생 EV 자율주행 경진대회

# About Project
2023 국제 대학생 EV 자율주행 경진대회 AA 1/5 부분 알고리즘

💻사용 기술
- Python
- ROS1
- Arduino

🙂참여한 사람

**하드 웨어 |**
손현아 , 김민재

**계획 제어 |**
최윤지 , 이연석

**인지 판단 |**
한주아



# 테스트 해야하는 부분

**planning**
- [x] 정적 장애물 - DWA 알고리즘

**tracking**
- [ ] pure pursuit LD
- [ ] PID 파라미터 조정

**vision**
- [x] 차선 인식 및 검출 -> 차선 테스트
- [x] 카메라 각도 테스트
- [ ] steer 값 충돌 확인
- [x] 라벨링 ->사용 X
- [x] 차선 딥러닝 후 정확도 테스트 ->사용 X

**Arduino**
- [ ] erp_write 토픽 들어올 때 manual 오류 수정
