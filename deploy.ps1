<#
    deploy.ps1 -- build the TeamCode APK on this Windows PC and install it on the
    Control Hub.

    Why this exists
    ---------------
    The team's supported flow (install.sh / build.sh) is Debian/Ubuntu-only, and
    it SYMLINKS the .java files from the repo into the FTC SDK so that editing a
    file is picked up by the next build with no sync step. Windows needs admin
    rights (or Developer Mode) for symlinks, so this script COPIES instead.

    The consequence matters: a change is invisible to the build until this script
    copies it. Always run this after editing -- do not run gradlew by hand and
    assume your edit made it in.

    Usage
    -----
        .\deploy.ps1                 copy sources, build, install on the hub
        .\deploy.ps1 -BuildOnly      build only; no hub required
        .\deploy.ps1 -InstallOnly    install the last APK that was built

    Needs: the Control Hub's USB-C port cabled to this PC with a data cable, and
    a green LED on the hub.
#>

param(
    [switch]$BuildOnly,
    [switch]$InstallOnly
)

$ErrorActionPreference = "Stop"

$Repo       = "C:\Users\tim\OneDrive\Documents\GitHub\ftc36533"
$Sdk        = "C:\Users\tim\ftc-build\ftc-sdk"
$PkgPath    = "TeamCode\src\main\java\org\firstinspires\ftc\teamcode"
$Apk        = "$Sdk\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
$Jbr        = "C:\Program Files\Android\Android Studio\jbr"
$AndroidSdk = "C:\Users\tim\AppData\Local\Android\Sdk"
$Adb        = "$AndroidSdk\platform-tools\adb.exe"

# Gradle 9.1 (what FTC SDK 12.0 pins) runs on the JDK bundled with Android
# Studio. It is not on PATH, which is why this is set here rather than assumed.
$env:JAVA_HOME        = $Jbr
$env:ANDROID_HOME     = $AndroidSdk
$env:ANDROID_SDK_ROOT = $AndroidSdk
$env:PATH             = "$Jbr\bin;$AndroidSdk\platform-tools;$env:PATH"

if (-not $InstallOnly) {

    Write-Host "== copying OpModes into the SDK clone" -ForegroundColor Cyan
    $src = "$Repo\$PkgPath"
    $dst = "$Sdk\$PkgPath"
    if (-not (Test-Path $src)) { throw "no sources at $src -- is the repo still there?" }
    if (-not (Test-Path $dst)) { throw "no SDK clone at $Sdk -- clone FTC SDK v12.0 there first" }

    $copied = 0
    Get-ChildItem "$src\*.java" | ForEach-Object {
        Copy-Item $_.FullName -Destination $dst -Force
        $copied++
    }
    Write-Host "   $copied file(s) copied (this is the step Linux does with symlinks)"

    # Drop .java files the repo no longer has. link-teamcode.sh removes stale symlinks
    # for the same reason: a class deleted in the repo but left behind in this copy
    # still compiles, so the build succeeds and the OpMode you thought you removed is
    # still on the robot. The FTC SDK ships no .java of its own in this package, so
    # anything here came from the repo.
    $removed = 0
    Get-ChildItem "$dst\*.java" | Where-Object {
        -not (Test-Path (Join-Path $src $_.Name))
    } | ForEach-Object {
        Remove-Item $_.FullName -Force
        $removed++
    }
    if ($removed -gt 0) { Write-Host "   removed $removed stale file(s)" }

    Write-Host "== building" -ForegroundColor Cyan
    # Gradle takes the project directory from the working directory, not from where
    # gradlew.bat happens to live. build.sh does a `cd` for the same reason; run it
    # from anywhere else and gradle looks for a build in that directory instead and
    # reports "does not contain a Gradle build".
    Push-Location $Sdk
    try {
        & "$Sdk\gradlew.bat" --no-daemon --console=plain :TeamCode:assembleDebug
        $gradleExit = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    if ($gradleExit -ne 0) { throw "gradle build failed (exit $gradleExit)" }

    # Check the artifact, not the exit status -- gradle can succeed and leave
    # nothing behind if the module name is wrong. build.sh does the same thing.
    if (-not (Test-Path $Apk)) { throw "gradle exited 0 but produced no APK" }
    Write-Host "   APK: $Apk" -ForegroundColor Green
}

if ($BuildOnly) {
    Write-Host "build-only: stopping before install"
    exit 0
}

Write-Host "== looking for the Control Hub" -ForegroundColor Cyan
# @(...) is load-bearing: with a single line attached, Where-Object returns a bare
# string rather than a one-element array, and indexing that gives the first CHARACTER
# of the serial -- which adb then reports as "device 'e' not found".
$attached = @(& $Adb devices | Select-Object -Skip 1 | Where-Object { $_ -match "\sdevice$" })
if (-not $attached) {
    throw @"
no device attached.

  - Plug the Control Hub's USB-C port into THIS PC (not the Driver Hub).
  - Use a data cable; charge-only cables power the hub but never enumerate.
  - Wait for the hub's LED to go green, then re-run.
"@
}
$serial = ($attached[0] -split "\s+")[0]

# adb does not care which Android device it is talking to, and this APK is the
# Robot Controller app -- installing it onto a Driver Hub would be wrong. So check
# the model before doing anything destructive.
#
# Note the two spellings: `adb devices -l` prints model:Control_Hub_v1_0, while
# getprop ro.product.model returns "Control Hub v1.0" with spaces. Match both.
$model = (& $Adb -s $serial shell getprop ro.product.model).Trim()
if ($model -notmatch "Control[ _]Hub") {
    throw "attached device reports model '$model', which is not a Control Hub. Refusing to install."
}
Write-Host "   found $model ($serial)"

Write-Host "== installing" -ForegroundColor Cyan
& $Adb -s $serial install -r $Apk
if ($LASTEXITCODE -ne 0) { throw "adb install failed (exit $LASTEXITCODE)" }
Write-Host "   the Driver Station link will drop and then restore itself -- expected"

# Prove the hub is running what we think. An install that silently did nothing
# looks identical to a successful one otherwise.
$onHub = ((& $Adb -s $serial shell pm path com.qualcomm.ftcrobotcontroller) -join "" -replace "package:", "").Trim()
$verify = Join-Path $env:TEMP "ftc-verify.apk"
& $Adb -s $serial pull $onHub $verify | Out-Null

$mine = (Get-FileHash $Apk     -Algorithm SHA256).Hash
$theirs = (Get-FileHash $verify -Algorithm SHA256).Hash
if ($mine -eq $theirs) {
    Write-Host "== verified: the hub is running this exact APK" -ForegroundColor Green
} else {
    throw "the APK on the hub does not match the one built -- investigate"
}

Write-Host ""
Write-Host "Done. Wheels off the ground, on blocks, for the first run." -ForegroundColor Yellow
