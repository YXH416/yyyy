# Import a REAL completed CAL,BALANCE record from a VOFA text log.
param([Parameter(Mandatory=$true)][string]$LogPath)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$text = Get-Content -LiteralPath $LogPath -Raw -Encoding UTF8
$records = [regex]::Matches($text, '\[BALANCE\][^\r\n]*event=SAVED_RAM pwm_mdeg=(\d+)\b')
if ($records.Count -eq 0) { throw 'No completed CAL,BALANCE measurement in this log.' }
$value = [int]$records[$records.Count-1].Groups[1].Value
if ($value -lt 0 -or $value -ge 360000) { throw 'PWM measurement outside 0..359999 mdeg.' }
$header = Join-Path $repo 'mspm0/Control/motor_balance_config.h'
$contents = Get-Content -LiteralPath $header -Raw -Encoding UTF8
if ([regex]::Matches($contents, '(?m)^#define MEASURED_BALANCE_VALID\s+\([01]U\)').Count -ne 1 -or
    [regex]::Matches($contents, '(?m)^#define MEASURED_BALANCE_PWM_MDEG\s+\(\d+L\)').Count -ne 1) {
    throw 'Unexpected header layout; no source changes made.'
}
$contents = $contents -replace '(?m)^#define MEASURED_BALANCE_VALID\s+\([01]U\)', '#define MEASURED_BALANCE_VALID       (1U)'
$contents = $contents -replace '(?m)^#define MEASURED_BALANCE_PWM_MDEG\s+\(\d+L\)', ('#define MEASURED_BALANCE_PWM_MDEG    (' + $value + 'L)')
[IO.File]::WriteAllText($header, $contents, [Text.UTF8Encoding]::new($false))
& (Join-Path $PSScriptRoot 'update_copy_bundle.ps1')
Write-Output "Saved measured balance reference to source: $value mdeg. Recompile to persist on board."
