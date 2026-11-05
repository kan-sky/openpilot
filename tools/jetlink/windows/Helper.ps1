#requires -Version 5.1
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
. (Join-Path $PSScriptRoot 'Core.ps1')
$script:StateRoot = Join-Path $env:LOCALAPPDATA 'HLCluster'
New-Item -ItemType Directory -Path $script:StateRoot -Force | Out-Null
$script:SessionPath = Join-Path $script:StateRoot 'last-job.json'

function Pause-Helper { Read-Host 'Enter를 누르면 메뉴로 돌아갑니다' | Out-Null }
function Confirm([string]$Prompt) { return (Read-Host "$Prompt [Y/N]") -ieq 'Y' }
function Require-OpenSsh {
    $folder = Join-Path $env:WINDIR 'System32\OpenSSH'
    if (Test-Path -LiteralPath $folder) { $env:PATH = "$folder;$env:PATH" }
    foreach ($tool in @('ssh.exe', 'scp.exe', 'ssh-keygen.exe')) {
        if (!(Get-Command $tool -ErrorAction SilentlyContinue)) {
            Start-Process 'ms-settings:optionalfeatures'
            throw 'Windows 선택적 기능에서 OpenSSH 클라이언트를 설치한 뒤 도우미를 다시 실행하세요.'
        }
    }
}
function Select-Key {
    $standard = Join-Path $env:USERPROFILE '.ssh\carrot_jetson'
    if (Test-Path -LiteralPath $standard) {
        if (Confirm "기존 키를 사용할까요? $standard") { return $standard }
    }
    $ours = Join-Path $env:USERPROFILE '.ssh\hl_cluster'
    if (Test-Path -LiteralPath $ours) {
        if (Confirm "이 도우미에서 만든 키를 사용할까요? $ours") { return $ours }
    }
    Add-Type -AssemblyName System.Windows.Forms
    $picker = New-Object System.Windows.Forms.OpenFileDialog
    $picker.Title = 'Jetson에 등록한 개인키 선택 (.pub 아님)'
    $picker.InitialDirectory = Join-Path $env:USERPROFILE '.ssh'
    if ($picker.ShowDialog() -ne 'OK') { throw '키 선택을 취소했습니다.' }
    return $picker.FileName
}
function Get-Connection {
    $address = Get-JetsonAddress (Read-Host 'Jetson IP (캐럿웹 Jetson/eGPU 상태에서 확인)')
    $key = Select-Key
    Get-SshOptions $key | Out-Null
    Write-Host '첫 연결의 지문은 대상 장치가 맞는지 확인하고 승인하세요. 키 변경 경고는 무시하지 마세요.' -ForegroundColor Yellow
    return [pscustomobject]@{ address = $address; key = $key }
}
function Invoke-Jetson($Connection, [string]$Command) {
    $options = Get-SshOptions $Connection.key
    $output = & ssh.exe @options "jetlink@$($Connection.address)" $Command
    if ($LASTEXITCODE -ne 0) { throw "Jetson 명령 실패($LASTEXITCODE). 위 SSH 오류를 확인하세요. 설치를 반복하지 말고 결과 조회를 먼저 하세요." }
    return $output -join "`n"
}
function Upload-Package($Connection, [string]$Token) {
    $remote = Get-JobDirectory $Token
    Invoke-Jetson $Connection "umask 077; mkdir $remote" | Out-Null
    $options = Get-SshOptions $Connection.key
    # Relative sources avoid SCP interpreting a Windows drive letter as a remote host.
    Push-Location $PSScriptRoot
    try {
        & scp.exe @options './install_cluster.py' './remote_helper.py' './cluster-jetson-overlay.tar.gz' "jetlink@$($Connection.address):$remote/"
        if ($LASTEXITCODE -ne 0) { throw '업로드에 실패했습니다. Jetson 설치는 시작하지 않았습니다.' }
    } finally { Pop-Location }
    return $remote
}
function Read-Job($Session) {
    $remote = Get-JobDirectory $Session.token
    return (Invoke-Jetson $Session "sudo -n python3 $remote/remote_helper.py status") | ConvertFrom-Json
}
function Show-Job($Session) {
    $report = Read-Job $Session
    Write-Host "`n작업: $($Session.action) / 상태: $($report.state)" -ForegroundColor Cyan
    Write-Host $report.log
    Write-Utf8 (Join-Path $script:StateRoot 'last-result.json') ($report | ConvertTo-Json -Depth 10)
    switch ($report.state) {
        'succeeded' { Write-Host '작업 완료. 설치/서비스 검사를 통과했습니다. 실제 모니터 출력은 별도로 확인하세요.' -ForegroundColor Green }
        'running' { Write-Host 'Jetson에서 작업 중입니다. 전원을 끄지 마세요. 3번에서 다시 조회할 수 있습니다.' }
        'starting' { Write-Host '시작 대기 중입니다. 잠시 후 결과를 다시 조회하세요.' }
        default { Write-Host '완료로 확인되지 않았습니다. 재설치하지 말고 결과 로그를 확인하세요. 필요하면 4번으로 원복하세요.' -ForegroundColor Yellow }
    }
}
function Run-Job([string]$Action) {
    $digest = Assert-Package $PSScriptRoot
    $connection = Get-Connection
    if (Test-Path -LiteralPath $script:SessionPath) {
        $previous = Get-Content -LiteralPath $script:SessionPath -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($previous.address -eq $connection.address) {
            $previous.key = $connection.key
            $report = Read-Job $previous
            if ($report.state -in @('starting', 'running')) { throw '이 장치에서 이전 작업이 진행 중입니다. 3번으로 조회하세요.' }
            if ($report.state -eq 'unknown' -and !(Confirm '이전 작업 상태가 불명확합니다. 로그 검토 후 새 작업을 진행할까요?')) { return }
        }
    }
    if (!(Confirm '차량 정차, 주행 보조 해제, 콤마와 Jetson USB 분리를 모두 확인했나요?')) { return }
    $token = [guid]::NewGuid().ToString('N')
    $remote = Upload-Package $connection $token
    $check = (Invoke-Jetson $connection "sudo -n python3 $remote/remote_helper.py check") | ConvertFrom-Json
    Write-Host "현재 런타임: $($check.current) / 우리 계기판 설치됨: $($check.pinned)"
    if ($Action -eq 'install' -and $check.pinned) { throw '이미 설치되어 있습니다. 다시 설치하려면 먼저 원복하세요.' }
    if ($Action -eq 'rollback' -and !$check.pinned) { throw '이 설치 도우미로 원복할 계기판이 없습니다.' }
    if (!(Confirm "대상 $($connection.address)에서 $Action 작업을 시작할까요?")) { return }
    $session = [pscustomobject]@{ address = $connection.address; key = $connection.key; token = $token; action = $Action }
    # Save before dispatch: a lost SSH reply must not cause a second installation.
    Write-Utf8 (Join-Path $script:StateRoot ("job-$token.json")) ($session | ConvertTo-Json)
    Write-Utf8 $script:SessionPath ($session | ConvertTo-Json)
    Invoke-Jetson $session "sudo -n python3 $remote/remote_helper.py start --action $Action --sha256 $digest --confirm-parked" | Write-Host
    Write-Host 'Jetson에 작업을 맡겼습니다. PC 연결이 끊겨도 계속 진행합니다. Jetson 전원은 유지하세요.'
    Write-Host '엔진 검사는 최대 약 20분 걸릴 수 있습니다. 키 암호를 설정했다면 조회 시 다시 요청할 수 있습니다.'
    Read-Host '잠시 기다렸다 Enter를 누르면 결과를 조회합니다' | Out-Null
    Show-Job $session
}
function Prepare-Key {
    $key = Join-Path $env:USERPROFILE '.ssh\hl_cluster'
    New-Item -ItemType Directory -Path (Split-Path $key) -Force | Out-Null
    if (!(Test-Path -LiteralPath $key) -and !(Test-Path -LiteralPath "$key.pub")) {
        Write-Host '이 PC 전용 키를 만듭니다. 암호 입력은 선택 사항입니다. 입력 글자는 보이지 않습니다.'
        & ssh-keygen.exe -t ed25519 -f $key -C 'hl-cluster'
        if ($LASTEXITCODE -ne 0) { throw '키 생성에 실패했습니다.' }
    }
    if (!(Test-Path -LiteralPath $key) -or !(Test-Path -LiteralPath "$key.pub")) { throw '키 파일 쌍이 불완전합니다. 기존 파일은 덮어쓰지 않았습니다.' }
    $public = (Get-Content -LiteralPath "$key.pub" -Raw -Encoding UTF8).Trim()
    Assert-PublicKey $public
    $derived = (& ssh-keygen.exe -y -f $key) -join "`n"
    if ($LASTEXITCODE -ne 0 -or (($derived.Trim() -split '\s+')[0..1] -join ' ') -cne (($public -split '\s+')[0..1] -join ' ')) {
        throw '개인키와 공개키가 일치하지 않습니다. 기존 파일은 변경하지 않았습니다.'
    }
    & ssh-keygen.exe -l -f "$key.pub"
    if ($LASTEXITCODE -ne 0) { throw '공개키 검증에 실패했습니다.' }
    Write-Host "`nJetson 정상 종료 → 전원 완전 분리 → SD 또는 SSD를 PC에 연결하세요." -ForegroundColor Yellow
    Write-Host 'Windows의 포맷 요청은 취소하세요. 디스크를 포맷하거나 파티션을 만들지 않습니다.'
    Read-Host '연결을 완료했으면 Enter' | Out-Null
    $volumes = @(Get-Volume | Where-Object { $_.FileSystemLabel -eq 'CARROTSETUP' })
    if ($volumes.Count -ne 1 -or !$volumes[0].DriveLetter -or $volumes[0].FileSystem -notin @('FAT', 'FAT32')) {
        throw '드라이브 문자가 있는 CARROTSETUP FAT 볼륨이 정확히 하나 필요합니다. 다른 매체를 분리하세요.'
    }
    $root = '{0}:\' -f $volumes[0].DriveLetter
    $keys = @($public)
    Write-Host '주의: 등록하는 공개키 목록은 Jetson의 기존 SSH 접속 키 목록을 대체합니다.' -ForegroundColor Yellow
    Write-Host '유지할 기존 공개키(.pub)가 있으면 다음 창에서 모두 선택하세요. 없으면 취소하세요.'
    Add-Type -AssemblyName System.Windows.Forms
    $picker = New-Object System.Windows.Forms.OpenFileDialog
    $picker.Title = '계속 사용할 기존 공개키 선택 (없으면 취소)'
    $picker.Filter = 'SSH public keys (*.pub)|*.pub'
    $picker.Multiselect = $true
    if ($picker.ShowDialog() -eq 'OK') {
        foreach ($path in $picker.FileNames) { $keys += (Get-Content -LiteralPath $path -Raw -Encoding UTF8).Trim() }
    }
    if (!(Confirm "$root 에 키 등록을 준비할까요? 목록에 없는 기존 SSH 키는 접속 권한을 잃습니다")) { return }
    Save-Setup $root $keys
    Write-Host '등록 파일 준비 완료. 아직 Jetson에 등록된 것은 아닙니다.' -ForegroundColor Green
    Write-Host '안전하게 제거 → 전원이 빠진 Jetson에 매체 장착 → 부팅 → 같은 LAN 연결 → 1번 설치.'
    Write-Host '부팅 시 SETUP-RESULT.json이 생성되며, 실제 SSH 연결이 성공해야 등록 완료입니다.'
    Write-Host "개인키는 이 PC에만 보관하세요: $key"
}

try {
    Assert-Package $PSScriptRoot | Out-Null
    Require-OpenSsh
    while ($true) {
        Write-Host "`nHL Cluster / Jetson 설치 도우미" -ForegroundColor Cyan
        Write-Host '1  연결 확인 및 계기판 설치'
        Write-Host '2  처음 사용하는 PC: SSH 키 등록 준비'
        Write-Host '3  마지막 설치 결과 조회'
        Write-Host '4  원래 Jetson 런타임으로 원복'
        Write-Host '5  결과 파일 폴더 열기'
        Write-Host '0  종료'
        $choice = Read-Host '번호'
        if ($choice -eq '0') { break }
        try {
            switch ($choice) {
                '1' { Run-Job 'install' }
                '2' { Prepare-Key }
                '3' {
                    if (!(Test-Path -LiteralPath $script:SessionPath)) { throw '저장된 설치 작업이 없습니다.' }
                    $session = Get-Content -LiteralPath $script:SessionPath -Raw -Encoding UTF8 | ConvertFrom-Json
                    $address = Read-Host "현재 Jetson IP (변경 없으면 Enter: $($session.address))"
                    if ($address) { $session.address = Get-JetsonAddress $address }
                    $session.address = Get-JetsonAddress $session.address
                    Write-Utf8 $script:SessionPath ($session | ConvertTo-Json)
                    Show-Job $session
                }
                '4' { Run-Job 'rollback' }
                '5' { Start-Process explorer.exe -ArgumentList $script:StateRoot }
                default { Write-Host '메뉴 번호를 선택하세요.' }
            }
        } catch {
            Write-Host $_.Exception.Message -ForegroundColor Red
            Write-Host 'publickey: 키 등록 확인 / timed out: IP·LAN 확인 / host key changed: 장치 신원 확인.'
            Write-Host '설치 시작 뒤 통신이 끊겼다면 3번 결과 조회. 강제 재설치나 전원 차단 금지.'
        }
        Pause-Helper
    }
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
