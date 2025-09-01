## custom robot arm 으로 지정된 아르코 마커 이동 

## 1단계
- 3d 모델링 urdf작성 및 sdf파일 만들어서 cmd_vel로 간단한 조작 

체험 코드
#### 0. git clone 하기
```bash
git clone git@github.com:jongbob1918/pbvs-4dof-robotarm.git
```
---

#### 1. 가제보 하모닉 설치

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/
pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-
archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $

(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/
null
sudo apt-get update
sudo apt-get install gz-harmonic
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-joint-state-publisher-gui
```
---

#### 2. sdf 파일 실행
```bash
# 폴더 이동
cd src/gz_arm_test

# 실행
gz sim robotarm_keyboard.sdf
```

---

#### 3. 로봇암 움직여보기 
    3-1. 월드에 로봇팔이 재대로 구현되어있는지 확인
    3-2. 왼쪽 밑에 주황색 실행 버튼 클릭
    3-3. 오른쪽 맨위에 전 세개 버튼 클릭 후 Key publisher 클릭 
    3-4. 키보드 입력이 영어 대문자로 출력되게 설정 

---
### 조작법

모터       반시계방향  시계방향
1번 모터    Q         A    
2번 모터    W         S
3번 모터    E         D
4번 모터    R         F


