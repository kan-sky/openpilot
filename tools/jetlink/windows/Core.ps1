Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

function Get-JetsonAddress([string]$Text) {
    $parts = $Text.Trim().Split('.')
    if ($parts.Count -ne 4) { throw 'Jetson의 IPv4 주소를 입력하세요. 콤마 주소가 아닙니다.' }
    foreach ($part in $parts) {
        if ($part -notmatch '^(0|[1-9][0-9]{0,2})$' -or [int]$part -gt 255) { throw '올바른 IPv4 주소가 아닙니다.' }
    }
    if ([int]$parts[0] -eq 0 -or [int]$parts[0] -eq 127 -or [int]$parts[0] -ge 224) {
        throw '실제 Jetson의 LAN 주소를 입력하세요.'
    }
    return $parts -join '.'
}

function Get-JobDirectory([string]$Token) {
    if ($Token -notmatch '^[a-f0-9]{32}$') { throw '잘못된 설치 작업 번호입니다.' }
    return "/var/tmp/hl-cluster-$Token"
}

function Write-Utf8([string]$Path, [string]$Text) {
    [IO.File]::WriteAllText($Path, $Text, [Text.UTF8Encoding]::new($false))
}

function Assert-Package([string]$Root) {
    $manifest = Get-Content -LiteralPath (Join-Path $Root 'package.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $required = @('Start.cmd', 'Core.ps1', 'Helper.ps1', 'README.txt', 'remote_helper.py',
                  'install_cluster.py', 'cluster-jetson-overlay.tar.gz', 'cluster-jetson-overlay.tar.gz.sha256')
    if ($manifest.format -ne 1 -or @($manifest.files.PSObject.Properties).Count -eq 0) { throw '배포 파일 형식 오류입니다.' }
    $names = @($manifest.files.PSObject.Properties.Name)
    if (@(Compare-Object $required $names).Count -ne 0) { throw '배포 파일 목록이 다릅니다. ZIP 전체를 다시 받으세요.' }
    foreach ($name in $required) {
        $expected = $manifest.files.$name
        $path = Join-Path $Root $name
        if ($expected -notmatch '^[a-f0-9]{64}$' -or !(Test-Path -LiteralPath $path -PathType Leaf) -or
            (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
            throw "파일 검증 실패: $name. ZIP 전체를 다시 받아 압축을 푸세요."
        }
    }
    $digest = (Get-Content -LiteralPath (Join-Path $Root 'cluster-jetson-overlay.tar.gz.sha256') -Raw).Trim()
    if ($digest -ne $manifest.files.'cluster-jetson-overlay.tar.gz') { throw '계기판 패키지 체크섬이 다릅니다.' }
    return $digest
}

function Assert-PublicKey([string]$Key) {
    if ($Key.Length -gt 16384 -or $Key.Contains("`n") -or $Key.Contains("`r")) { throw '공개키 형식 오류입니다.' }
    $fields = $Key.Trim() -split '\s+'
    if ($fields.Count -lt 2 -or $fields[0] -notin @('ssh-ed25519', 'ssh-rsa', 'ecdsa-sha2-nistp256')) {
        throw '일반 SSH 공개키(.pub)만 가능합니다. 개인키는 넣지 마세요.'
    }
    $raw = [Convert]::FromBase64String($fields[1])
    if ($raw.Length -lt 9) { throw '공개키 데이터 오류입니다.' }
    $size = [int64]$raw[0] * 16777216 + [int64]$raw[1] * 65536 + [int64]$raw[2] * 256 + $raw[3]
    if ($size -lt 1 -or $size + 8 -ge $raw.Length -or [Text.Encoding]::ASCII.GetString($raw, 4, [int]$size) -ne $fields[0]) {
        throw '공개키 인코딩 오류입니다.'
    }
}

function Merge-SetupKeys([string]$ExistingJson, [string[]]$PublicKeys) {
    $config = [pscustomobject]@{}
    if ($ExistingJson) {
        if (!$ExistingJson.TrimStart().StartsWith('{')) { throw 'setup.json은 JSON 객체여야 합니다.' }
        $config = $ExistingJson | ConvertFrom-Json
        foreach ($property in $config.PSObject.Properties) {
            $name = $property.Name
            if ($name -notin @('hostname', 'wifi', 'ssh_public_keys')) { throw "지원하지 않는 setup.json 항목: $name" }
        }
        if ($null -ne $config.PSObject.Properties['hostname'] -and $null -ne $config.hostname) {
            if ($config.hostname -isnot [string] -or $config.hostname -cnotmatch '^([a-z][a-z0-9-]{0,61}[a-z0-9]|[a-z])$') {
                throw '기존 setup.json의 hostname이 유효하지 않습니다. 파일은 변경하지 않습니다.'
            }
        }
        if ($null -ne $config.PSObject.Properties['wifi'] -and $null -ne $config.wifi) {
            $wifi = $config.wifi
            $names = @($wifi.PSObject.Properties | ForEach-Object { $_.Name })
            if (@(Compare-Object @('ssid', 'password') $names).Count -ne 0 -or
                $wifi.ssid -isnot [string] -or $wifi.password -isnot [string]) { throw '기존 Wi-Fi 설정 형식 오류입니다.' }
            if ([Text.Encoding]::UTF8.GetByteCount($wifi.ssid) -notin 1..32 -or
                $wifi.password.Length -notin 8..63 -or ($wifi.ssid + $wifi.password) -match '[\x00\r\n]') {
                throw '기존 Wi-Fi 설정이 유효하지 않습니다. 파일은 변경하지 않습니다.'
            }
        }
    }
    $all = @()
    if ($null -ne $config.PSObject.Properties['ssh_public_keys']) {
        if ($config.ssh_public_keys -isnot [Array]) { throw '기존 ssh_public_keys는 배열이어야 합니다.' }
        $all += $config.ssh_public_keys
    }
    $all += $PublicKeys
    $keys = @($all | Select-Object -Unique)
    if ($keys.Count -lt 1 -or $keys.Count -gt 16) { throw '공개키는 1~16개여야 합니다.' }
    foreach ($key in $keys) {
        if ($key -isnot [string]) { throw '공개키는 문자열이어야 합니다.' }
        Assert-PublicKey $key
    }
    $config | Add-Member -MemberType NoteProperty -Name ssh_public_keys -Value $keys -Force
    return $config | ConvertTo-Json -Depth 10
}

function Save-Setup([string]$Root, [string[]]$PublicKeys) {
    $path = Join-Path $Root 'setup.json'
    $original = $null
    if (Test-Path -LiteralPath $path) { $original = [IO.File]::ReadAllText($path) }
    $json = Merge-SetupKeys $original $PublicKeys
    $temp = Join-Path $Root ('setup-' + [guid]::NewGuid().ToString('N') + '.tmp')
    try {
        Write-Utf8 $temp ($json + "`n")
        if ($null -ne $original) {
            if ([IO.File]::ReadAllText($path) -cne $original) { throw 'setup.json이 변경되었습니다. 다시 확인하세요.' }
            # File.Replace is not supported on every FAT volume. Never fall back to delete-then-write.
            [IO.File]::Replace($temp, $path, [NullString]::Value)
        } else { [IO.File]::Move($temp, $path) }
    } finally { if (Test-Path -LiteralPath $temp) { Remove-Item -LiteralPath $temp } }
}

function Get-SshOptions([string]$KeyPath) {
    if (!(Test-Path -LiteralPath $KeyPath -PathType Leaf) -or $KeyPath.EndsWith('.pub')) { throw '접속용 개인키 파일을 선택하세요(.pub 아님).' }
    return @('-i', $KeyPath, '-o', 'IdentitiesOnly=yes', '-o', 'StrictHostKeyChecking=ask',
             '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3')
}
