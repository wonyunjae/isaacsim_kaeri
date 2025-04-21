# [SETUP]
## [0] docker install
```
$ curl -fsSL https://get.docker.com/ | sudo sh
$ sudo usermod -aG docker $USER

##### NVIDIA-Docker, GPU
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

$ sudo apt-get update
$ sudo apt-get install -y nvidia-docker2
$ sudo systemctl restart docker
```

## [1] Prepare isaac sim image 
  ### (option 1) image download 
  ```
  git clone http://183.107.37.79:13003/kaeri_ert/docker/dockerfiles.git
  ```
  ```
  cd dockerfiles
  docker login nvcr.io
  docker build --pull -t \
    isaac-sim:$(date +"%Y.%m.%d")-ubuntu20.04 \
    --build-arg ISAACSIM_VERSION=2023.1.0 \
    --file isaac_sim/isaac_sim_with_ubuntu20.04 .
  ```

  ### (option 2) image file from 우철
  1. 신관 2층 202호 (이우철)에서 image 파일이 들어있는 ssd  대여 
  2. ssd에서 image 로드
      ```
      docker load -i <file_name>.tar
      ```
  3. 아래 커맨드를 이용하여 image 정상 load 됐는지 확인
      ```
      docker images
      ```

## [2] Container generation
1. clone following package
    ```
    git clone http://183.107.37.79:13003/kaeri_ert/simulation/isaac_sim.git
    ```
    
2. make container
    ```
    cd isaac_sim
    bash docker/image2container.sh
    ```

3. above command will generate questions 
    - Enter the image name : 
      ```
      isaac-sim:2023.12.20-ubuntu20.04
      ```
    - Enter the container name :
      ```
      isaac
      ```

4. attach to container (assume container name is "isaac")
    ```
    docker exec -it isaac bash
    ```

5. test empty world
    ```
    docker exec -it isaac_sim bash
    ./runapp.sh
    ```

## [3] (Optional) setup orbit
  ```
  cd isaac-sim/isaac_sim
  bash setup/setup_about_orbit.sh
  ```
    
# [VSCODE 활용]
1. local pc에 vscode 설치
2. vscode 실행 후, 좌측 EXTENTION 탭에서 Remote Development 패키지 검색 후 설치
3. 좌측 REMOTE EXPLOLER 탭에 들어가면 앞서 만든 docker container가 보일것임. 해당 container 우클릭 후 "Attach in Current Window" 클릭
4. 아래 경로에서 custom 시뮬레이션 환경구성 코드 작성
    ```
    cd /isaac-sim/isaac_sim
    ```
    
# [EXAMPLE]
[example link](http://183.107.37.79:13003/kaeri_ert/simulation/isaac_sim/-/tree/master/example?ref_type=heads)


# [TROUBLESHOOTING]
- roscore error
```
lsof -i :11311
sudo kill -9 <pid>
```

- isaac sim non-clear
```
ps -ef
(check the pid of isaac_sim)
kill -9 <pid>
```

# frequently used commands
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/Documentation/Isaac-Sim-Docs_2022.2.0/app_isaacsim/app_isaacsim/reference_python_snippets.html