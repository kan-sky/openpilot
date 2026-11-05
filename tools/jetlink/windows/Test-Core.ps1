param([string]$PackageRoot)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'Core.ps1')
$script:Passed = 0
function Check([bool]$Condition, [string]$Name) {
    if (!$Condition) { throw "FAIL: $Name" }
    $script:Passed++
}
function Reject([scriptblock]$Code, [string]$Name) {
    $failed = $false
    try { & $Code | Out-Null } catch { $failed = $true }
    Check $failed $Name
}
foreach ($script in Get-ChildItem -LiteralPath $PSScriptRoot -Filter '*.ps1') {
    $tokens = $null
    $errors = $null
    [Management.Automation.Language.Parser]::ParseFile($script.FullName, [ref]$tokens, [ref]$errors) | Out-Null
    Check ($errors.Count -eq 0) "Parse $($script.Name): $errors"
}
Check ((Get-JetsonAddress ' 192.168.55.1 ') -eq '192.168.55.1') 'IPv4'
foreach ($bad in @('127.0.0.1', '0.0.0.0', '224.0.0.1', '-oProxyCommand=oops', '1.2.3.4;id', '1.2.3.999', '1.02.3.4', '1.2.3.4:22', 'foo')) {
    Reject { Get-JetsonAddress $bad } "Reject address $bad"
}
Reject { Get-JobDirectory '../../tmp' } 'Job traversal'
Check ((Get-JobDirectory ('a' * 32)) -eq ('/var/tmp/hl-cluster-' + 'a' * 32)) 'Job path'
$key = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAID8KEPn8BB7shSDYTSSmpGFbgwy/h5nGo6tjQNpE9Y6L test'
Assert-PublicKey $key
Check $true 'Public key'
foreach ($bad in @('-----BEGIN OPENSSH PRIVATE KEY-----', 'ssh-ed25519 invalid', "${key}`n${key}", ('command="false" ' + $key))) {
    Reject { Assert-PublicKey $bad } 'Invalid public key'
}
$merged = Merge-SetupKeys '{"wifi":{"ssid":"기존 네트워크","password":"not-a-real-secret"},"hostname":"carrot-jetson"}' @($key, $key) | ConvertFrom-Json
Check (@((Merge-SetupKeys '{}' @($key) | ConvertFrom-Json).ssh_public_keys).Count -eq 1) 'Empty object accepts key'
Check ($merged.wifi.ssid -ceq '기존 네트워크') 'Preserve Korean SSID'
Check ($merged.wifi.password -ceq 'not-a-real-secret') 'Preserve Wi-Fi setting'
Check ($merged.hostname -eq 'carrot-jetson') 'Preserve hostname'
Check (@($merged.ssh_public_keys).Count -eq 1) 'Deduplicate keys'
$existing = @{ssh_public_keys = @($key)} | ConvertTo-Json
$merged = Merge-SetupKeys $existing @($key.Replace('test', 'second')) | ConvertFrom-Json
Check (@($merged.ssh_public_keys).Count -eq 2) 'Preserve existing keys'
Reject { Merge-SetupKeys '{"command":"evil"}' @($key) } 'Reject unknown config'
Reject { Merge-SetupKeys '[]' @($key) } 'Reject non-object config'
Reject { Merge-SetupKeys '{"ssh_public_keys":"bad"}' @($key) } 'Reject wrong key array'
Reject { Merge-SetupKeys '{}' @() } 'Reject empty key list'
Reject { Merge-SetupKeys '{"hostname":"UPPER-case"}' @($key) } 'Reject invalid upstream hostname'
Reject { Merge-SetupKeys '{"wifi":{"ssid":"ssid","password":"short"}}' @($key) } 'Reject invalid upstream Wi-Fi'
Reject { Merge-SetupKeys '{"wifi":{}}' @($key) } 'Reject incomplete upstream Wi-Fi'
$temp = Join-Path ([IO.Path]::GetTempPath()) ('hl-helper-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temp | Out-Null
try {
    Save-Setup $temp @($key)
    $bytes = [IO.File]::ReadAllBytes((Join-Path $temp 'setup.json'))
    Check ($bytes[0] -eq [byte][char]'{') 'UTF-8 without BOM'
    Save-Setup $temp @($key.Replace('test', 'another'))
    $saved = Get-Content -LiteralPath (Join-Path $temp 'setup.json') -Raw | ConvertFrom-Json
    Check (@($saved.ssh_public_keys).Count -eq 2) 'Atomic merge'
    $before = [IO.File]::ReadAllText((Join-Path $temp 'setup.json'))
    Reject { Save-Setup $temp @('invalid') } 'Reject before write'
    Check ([IO.File]::ReadAllText((Join-Path $temp 'setup.json')) -ceq $before) 'Invalid input preserves file'
    Check (@(Get-ChildItem -LiteralPath $temp).Count -eq 1) 'No stray setup files'
    $private = Join-Path $temp 'key with spaces'
    Write-Utf8 $private 'fixture, not a real key'
    $options = Get-SshOptions $private
    Check ($options -contains 'StrictHostKeyChecking=ask') 'Never disable host checking'
    Check ($options -contains $private) 'Key path stays one argument'
    Reject { Get-SshOptions ($private + '.pub') } 'Reject public key as identity'
    if ($PackageRoot) {
        $digest = Assert-Package $PackageRoot
        Check ($digest -match '^[a-f0-9]{64}$') 'Built package checksums'
        $copy = Join-Path $temp 'package'
        Copy-Item -LiteralPath $PackageRoot -Destination $copy -Recurse
        Write-Utf8 (Join-Path $copy 'remote_helper.py') 'tampered'
        Reject { Assert-Package $copy } 'Reject tampered executable'
    }
} finally { Remove-Item -LiteralPath $temp -Recurse -Force }
Write-Host "$script:Passed assertions passed"
