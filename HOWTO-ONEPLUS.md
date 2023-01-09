원플(Oneplus 3t)에서 오픈파일럿 이식하려면?
------
1. 새로운 방법으로 바꿉니다.
 원플러스 혹은 러프로 폰에서 일단 twrp리커버리로 진입합니다.
 twrp리커버리 설치방법은 생략합니다.
 
 거기서 wipe->format->yes 로 공초한 다음 다시 리커버리모드로 재진입합니다
 거기서 다시 wipe->avnced wipe-> cache/data/system 체크후 wipe로 스왑후 다시 리커버리 모드로 재진입합니다.
 아래 링크를 클릭해서 시스템파일을 다운로드합니다
 https://commadist.azureedge.net/neosupdate/ota-signed-5dc2575d713977666a8e14ae1b43a04d7f63123934c80fa10751d949a107653e.zip
 아래 링크를 클릭해서 원플러스용 시스템파일도 다운로드합니다.
 https://drive.google.com/file/d/19xHtHuHOCw2c24PkFd1_vw7SPw0kFtoN/view?usp=sharing
 먼저 다운받은 시스템파일부터 설치합니다
 twrp 메뉴에서 install-> 아래 시스템파일 선택후 플래싱
 ota-signed-~....107653e.zip
 ```
 러프로폰에서는 이 시스템 zip파일이 바로 플래싱 되지 않는 현상이 있으니
 압축을 풀어서 나오는 boot.img와 system.img를 플래싱하면 됩니다
 (부트이미지 플래싱은 부트섹터에, 시스템이미지 플래싱은 시스템섹터에)
 ```
 시스템 설치 완료메세지(done.)가 나오면 리부트, twrp 리커버리 모드 선택후 스왑해서 리커버리로 재진입합니다.
 update.zip 파일도 플래싱합니다.
 (러프로 폰에서는 이 update.zip 플래싱이 필요없습니다)
 
 완료후 wipe cache.davik메시지 뜨면 클릭하고 스왑한 다음 리부트합니다.
 콤마 로고 스피너가 나오면 오파 설치가 시작됩니다.
 
 continue setup -> custom setup -> wifie설정->continue->URL에서 아래 주소를 적어줍니다.
 https://smiskol.com/fork/kan-sky/tw-dev
 
 현브렌치 오파가 설치되고 5분 정도의 오파설치 과정을 거치면 드디어 현브렌치 빌드가 됩니다.
 상당히 긴 시간이 소요되고 오파 트레이닝 과정을 거쳐 설치가 완료됩니다.
 원플러스는 터치가 먹히지 않으므로 재부팅하면 설치가 완료됩니다.
