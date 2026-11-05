param([Parameter(Mandatory=$true)][string]$PackageRoot)
$ErrorActionPreference = 'Stop'
$script:Count = 0
function Check([bool]$Condition, [string]$Name) {
    if (!$Condition) { throw "FAIL: $Name" }
    $script:Count++
}
function Read-Host {
    param([string]$Prompt)
    if ($global:HLTestAnswers.Count -eq 0) { throw "Unexpected prompt: $Prompt" }
    return $global:HLTestAnswers.Dequeue()
}
function ssh.exe {
    $global:HLTestCalls.Add(@($args))
    if ($args -notcontains 'StrictHostKeyChecking=ask') { throw 'Missing host key checking' }
    $global:LASTEXITCODE = 0
    $command = $args[-1]
    if ($command -match 'remote_helper.py check$') { return '{"current":"/opt/carrot-jetlink/releases/original","pinned":false}' }
    if ($command -match 'remote_helper.py start ') {
        $global:LASTEXITCODE = $global:HLTestStartExit
        if ($global:HLTestStartExit -eq 0) { return '{"state":"dispatched"}' }
        return
    }
    if ($command -match 'remote_helper.py status$') { return '{"state":"succeeded","log":"fixture job complete"}' }
    if ($command -match '^umask 077; mkdir /var/tmp/hl-cluster-[a-f0-9]{32}$') { return }
    throw "Unexpected remote command: $command"
}
function scp.exe {
    $global:HLTestCopies.Add(@($args))
    $global:LASTEXITCODE = 0
}
function ssh-keygen.exe { throw 'Key enrollment not part of the mocked install test' }

$savedLocal = $env:LOCALAPPDATA
$savedProfile = $env:USERPROFILE
$savedWindows = $env:WINDIR
$temp = Join-Path ([IO.Path]::GetTempPath()) ('hl-menu-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path (Join-Path $temp 'profile/.ssh') -Force | Out-Null
[IO.File]::WriteAllText((Join-Path $temp 'profile/.ssh/carrot_jetson'), 'not a real key')
try {
    $env:USERPROFILE = Join-Path $temp 'profile'
    $env:WINDIR = $temp
    foreach ($case in @('success', 'lost-reply')) {
        $env:LOCALAPPDATA = Join-Path $temp $case
        $global:HLTestStartExit = $(if ($case -eq 'success') { 0 } else { 255 })
        $global:HLTestAnswers = New-Object 'System.Collections.Generic.Queue[string]'
        $global:HLTestCalls = New-Object 'System.Collections.Generic.List[object]'
        $global:HLTestCopies = New-Object 'System.Collections.Generic.List[object]'
        foreach ($answer in @('1', '192.168.55.1', 'Y', 'Y', 'Y')) { $global:HLTestAnswers.Enqueue($answer) }
        if ($case -eq 'success') { $global:HLTestAnswers.Enqueue('') }
        $global:HLTestAnswers.Enqueue('')
        $global:HLTestAnswers.Enqueue('0')
        & (Join-Path $PackageRoot 'Helper.ps1') *> (Join-Path $temp "$case.log")
        $transcript = Get-Content -LiteralPath (Join-Path $temp "$case.log") -Raw
        Check ($global:HLTestAnswers.Count -eq 0) "$case prompts consumed: $transcript"
        Check ($global:HLTestCopies.Count -eq 1) "$case uploads once"
        Check ($global:HLTestCopies[0] -contains './remote_helper.py') "$case remote runner included"
        $stateRoot = Join-Path $env:LOCALAPPDATA 'HLCluster'
        $session = Get-Content -LiteralPath (Join-Path $stateRoot 'last-job.json') -Raw | ConvertFrom-Json
        Check ($session.address -eq '192.168.55.1' -and $session.action -eq 'install') "$case job retained"
        Check ($session.token -match '^[a-f0-9]{32}$') "$case safe token"
        Check (Test-Path -LiteralPath (Join-Path $stateRoot ("job-$($session.token).json"))) "$case history retained"
        $log = Get-Content -LiteralPath (Join-Path $temp "$case.log") -Raw
        $result = Join-Path $stateRoot 'last-result.json'
        if ($case -eq 'success') {
            Check ((Get-Content -LiteralPath $result -Raw | ConvertFrom-Json).state -eq 'succeeded') 'Success reported'
            Check ($log.Contains('작업 완료.')) 'Success UI'
        } else {
            Check (!(Test-Path -LiteralPath $result)) 'Lost reply never marked successful'
            Check (!$log.Contains('작업 완료.')) 'No false success UI'
        }
    }
} finally {
    $env:LOCALAPPDATA = $savedLocal
    $env:USERPROFILE = $savedProfile
    $env:WINDIR = $savedWindows
    Remove-Item -LiteralPath $temp -Recurse -Force
}
Write-Host "$script:Count menu assertions passed (SSH/SCP mocked; no device contacted)"
