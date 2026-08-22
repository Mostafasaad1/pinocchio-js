<#
.SYNOPSIS
    Pinocchio WASM - Windows PowerShell Build Script

.DESCRIPTION
    Builds the Pinocchio C++ to WebAssembly bindings using Emscripten and CMake/Ninja.

.PARAMETER BuildType
    Build type: Release (default), MinSizeRel, or Debug.

.PARAMETER Clean
    Clean build directory before compiling.

.EXAMPLE
    .\build.ps1
    .\build.ps1 MinSizeRel
    .\build.ps1 -Clean
#>

[CmdletBinding()]
param (
    [Parameter(Position = 0)]
    [ValidateSet("Release", "MinSizeRel", "Debug")]
    [string]$BuildType = "Release",

    [Parameter()]
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Ensure current process PATH includes machine/user registry paths (e.g. CMake, Git, Ninja)
$regPaths = @(
    [Environment]::GetEnvironmentVariable("Path", "Machine"),
    [Environment]::GetEnvironmentVariable("Path", "User"),
    "C:\Program Files\CMake\bin",
    "C:\Program Files\Git\cmd"
) | Where-Object { $_ }
$env:Path = ($regPaths -join ";") + ";" + $env:Path

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "   Pinocchio WASM Build (Windows)        " -ForegroundColor Cyan
Write-Host "   Build type: $($BuildType.PadRight(24))" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

# Check/Activate Emscripten
if (-not (Get-Command emcmake -ErrorAction SilentlyContinue)) {
    $searchPaths = @(
        (Join-Path $ScriptDir "emsdk"),
        (Join-Path (Split-Path -Parent $ScriptDir) "emsdk"),
        $env:EMSDK
    )
    $emsdkFound = $null
    foreach ($p in $searchPaths) {
        if ($p -and (Test-Path (Join-Path $p "emsdk_env.ps1"))) {
            $emsdkFound = $p
            break
        }
    }

    if ($emsdkFound) {
        Write-Host "[*] Activating Emscripten SDK from: $emsdkFound" -ForegroundColor Yellow
        . (Join-Path $emsdkFound "emsdk_env.ps1")
    } else {
        Write-Error "[!] Error: Emscripten (emcmake) not found in PATH or emsdk directory."
        exit 1
    }
}

# Determine Pinocchio source directory
if (-not $env:PINOCCHIO_SOURCE_DIR) {
    $pinocchioCandidates = @(
        (Join-Path (Split-Path -Parent $ScriptDir) "pinocchio"),
        (Join-Path $ScriptDir "pinocchio")
    )
    foreach ($cand in $pinocchioCandidates) {
        if (Test-Path (Join-Path $cand "include\pinocchio")) {
            $env:PINOCCHIO_SOURCE_DIR = (Resolve-Path $cand).Path
            break
        }
    }
}

if (-not $env:PINOCCHIO_SOURCE_DIR -or -not (Test-Path (Join-Path $env:PINOCCHIO_SOURCE_DIR "include\pinocchio"))) {
    Write-Error "[!] Error: Pinocchio source tree not found. Expected at ../pinocchio or specify `$env:PINOCCHIO_SOURCE_DIR."
    exit 1
}

Write-Host "[+] Using Pinocchio source: $($env:PINOCCHIO_SOURCE_DIR)" -ForegroundColor Green

$BuildDir = Join-Path $ScriptDir "build"

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "[*] Cleaning build directory..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $BuildDir
}

if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

# Choose generator (prefer Ninja if available)
$generatorArgs = @()
if (Get-Command ninja -ErrorAction SilentlyContinue) {
    $generatorArgs = @("-G", "Ninja")
}

Write-Host "[*] Configuring with Emscripten..." -ForegroundColor Cyan
$cmakeConfigArgs = @(
    "-B", $BuildDir,
    "-S", $ScriptDir,
    "-DCMAKE_BUILD_TYPE=$BuildType",
    "-DPINOCCHIO_SOURCE_DIR=$($env:PINOCCHIO_SOURCE_DIR.Replace('\', '/'))"
)
if ($generatorArgs.Count -gt 0) {
    $cmakeConfigArgs += $generatorArgs
}

& emcmake cmake @cmakeConfigArgs
if ($LASTEXITCODE -ne 0) {
    Write-Error "[!] CMake configuration failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

Write-Host "[*] Building WebAssembly target..." -ForegroundColor Cyan
& cmake --build $BuildDir --config $BuildType
if ($LASTEXITCODE -ne 0) {
    Write-Error "[!] Build failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}

# Report results
Write-Host ""
Write-Host "[OK] Build complete!" -ForegroundColor Green
Write-Host ""

$wasmFile = Join-Path $BuildDir "pinocchio.wasm"
$jsFile = Join-Path $BuildDir "pinocchio.js"

if (Test-Path $wasmFile) {
    $wasmSize = (Get-Item $wasmFile).Length
    $wasmKb = [math]::Round($wasmSize / 1024, 1)
    Write-Host "[Artifact] pinocchio.wasm: $wasmKb KB ($wasmSize bytes)" -ForegroundColor Cyan
}

if (Test-Path $jsFile) {
    $jsSize = (Get-Item $jsFile).Length
    $jsKb = [math]::Round($jsSize / 1024, 1)
    Write-Host "[Artifact] pinocchio.js:   $jsKb KB ($jsSize bytes)" -ForegroundColor Cyan
}

Write-Host ""
Write-Host "To run tests:" -ForegroundColor Yellow
Write-Host "  npm test"
Write-Host "  npm run test:smoke"
