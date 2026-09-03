# Refresh the fixed source-only delivery directory. Never delete directories.
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Split-Path $PSScriptRoot -Parent))
# Unicode codepoints keep the script compatible with Windows PowerShell 5.1
# even when this UTF-8 source file is saved without a BOM.
$bundleName = -join ([char[]](0x5F85,0x590D,0x5236,0x6587,0x4EF6))
$destination = Join-Path (Split-Path $repo -Parent) $bundleName
$manifestPath = Join-Path $PSScriptRoot 'copy_manifest.json'
$entries = @(Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json)
New-Item -ItemType Directory -Path $destination -Force | Out-Null
$destination = (Resolve-Path -LiteralPath $destination).Path
function ResolveChild([string]$base, [string]$relative) {
    $full = [IO.Path]::GetFullPath((Join-Path $base $relative))
    $prefix = $base.TrimEnd('\','/') + [IO.Path]::DirectorySeparatorChar
    if (-not $full.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Path outside designated directory: $relative"
    }
    return $full
}
# Validate all sources before replacing any existing delivery files.
foreach ($entry in $entries) {
    $source = ResolveChild $repo $entry.source
    $null = ResolveChild $destination $entry.target
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "Missing source: $source"
    }
}
$listName = (-join ([char[]](0x6E05,0x5355))) + '.json'
$oldManifest = Join-Path $destination $listName
if (Test-Path -LiteralPath $oldManifest -PathType Leaf) {
    $oldEntries = @(Get-Content -LiteralPath $oldManifest -Raw -Encoding UTF8 | ConvertFrom-Json)
    foreach ($entry in $oldEntries) {
        if ($entries.target -notcontains $entry.target) {
            $obsolete = ResolveChild $destination $entry.target
            if (Test-Path -LiteralPath $obsolete -PathType Leaf) {
                Remove-Item -LiteralPath $obsolete -Force
            }
        }
    }
}
foreach ($entry in $entries) {
    $source = ResolveChild $repo $entry.source
    $target = ResolveChild $destination $entry.target
    New-Item -ItemType Directory -Path (Split-Path $target -Parent) -Force | Out-Null
    Copy-Item -LiteralPath $source -Destination $target -Force
    if ((Get-FileHash -LiteralPath $source).Hash -ne (Get-FileHash -LiteralPath $target).Hash) {
        throw "Copy verification failed: $target"
    }
}
Copy-Item -LiteralPath $manifestPath -Destination $oldManifest -Force
Write-Output "Source-only copy folder refreshed: $destination"
