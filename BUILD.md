# Building and installing

Two paths, because the two machines need different tricks. Both end in the same place: an
APK that you side-load onto the Control Hub, which is what the Driver Station then lists
the OpModes from.

**Nothing in `TeamCode/` reaches the robot — or even the compiler — until one of these
scripts copies or links it into the FTC SDK clone.** Editing a file, committing it, and
pushing it to GitHub all do nothing on their own. That is the single most common reason an
OpMode "isn't there".

---

## On Linux — `install.sh` / `build.sh`

The supported flow, documented in the README. Debian/Ubuntu only.

```
./install.sh                    once per machine: clone the SDK, write .ftc-env.sh
./build.sh                      assembleDebug — the APK you side-load
./build.sh install              build and push to a connected hub
./build.sh clean                wipe build outputs
./build.sh <task>               any other gradle task
```

The SDK is cloned to `ftc-sdk/` and is not tracked here. `link-teamcode.sh` **symlinks**
every `.java` from this repo into it, so an edit is picked up by the next build with no
sync step, and it deletes links left behind by a renamed or deleted file. `build.sh` runs
it before every gradle invocation, which is what makes a newly added `.java` visible.

---

## On Windows — `deploy.ps1`

Windows needs admin rights or Developer Mode to create symlinks, so this script **copies**
the sources instead. The consequence is the whole reason this file exists:

> A change is invisible to the build until this script copies it. Run it after editing —
> never run `gradlew` by hand and assume your edit made it in.

```
powershell -NoProfile -ExecutionPolicy Bypass -File .\deploy.ps1              # copy + build + install
powershell -NoProfile -ExecutionPolicy Bypass -File .\deploy.ps1 -BuildOnly   # build only, no hub needed
powershell -NoProfile -ExecutionPolicy Bypass -File .\deploy.ps1 -InstallOnly # push the last APK, no rebuild
```

What it does, in order:

1. **Copies** `TeamCode/**/*.java` from the repo into the SDK clone.
2. **Deletes** any `.java` in the SDK copy that the repo no longer has. Skip this and a
   class you deleted still compiles, so the build succeeds and the OpMode you thought you
   removed is still on the robot.
3. **Builds** `:TeamCode:assembleDebug` from the SDK directory.
4. **Installs** on the Control Hub over adb, after checking the attached device really
   reports a Control Hub — installing the Robot Controller APK onto a Driver Hub would be
   wrong, and adb does not care which device it is talking to.
5. **Verifies** by pulling the installed APK back off the hub and comparing SHA-256 with
   the one it just built. An install that silently did nothing looks identical to a
   successful one otherwise.

### Editing it for another machine

The first block of the script is paths, and they are this machine's:

| variable | here | what it is |
|---|---|---|
| `$Repo` | `C:\Users\tim\OneDrive\Documents\GitHub\ftc36533` | this repository |
| `$Sdk` | `C:\Users\tim\ftc-build\ftc-sdk` | the FTC SDK 12.0 clone |
| `$Jbr` | `C:\Program Files\Android\Android Studio\jbr` | the JDK Gradle 9.1 needs — not on PATH |
| `$AndroidSdk` | `C:\Users\tim\AppData\Local\Android\Sdk` | Android SDK, for `adb` and `ANDROID_HOME` |

Change those four and it works anywhere.

---

## With no internet

Verified on 2026-10-16: `BUILD SUCCESSFUL in 37s`, exit 0. The only difference from the
normal flow is `--offline` on Gradle.

**This works because the caches are already warm** — Gradle's dependency cache and its
wrapper distribution live in `%USERPROFILE%\.gradle`, and the Android SDK is local. A
machine that has never built this project cannot build it offline, and nothing here can
fix that; build once while you still have a connection (at home, the night before).

```powershell
$Repo = "C:\Users\tim\OneDrive\Documents\GitHub\ftc36533"
$Sdk  = "C:\Users\tim\ftc-build\ftc-sdk"
$Pkg  = "TeamCode\src\main\java\org\firstinspires\ftc\teamcode"

# 1. copy sources into the SDK clone, and drop the ones the repo no longer has
Copy-Item "$Repo\$Pkg\*.java" "$Sdk\$Pkg" -Force
Get-ChildItem "$Sdk\$Pkg\*.java" |
    Where-Object { -not (Test-Path (Join-Path "$Repo\$Pkg" $_.Name)) } |
    Remove-Item -Force

# 2. build with no network
$env:JAVA_HOME        = "C:\Program Files\Android\Android Studio\jbr"
$env:ANDROID_HOME     = "C:\Users\tim\AppData\Local\Android\Sdk"
$env:ANDROID_SDK_ROOT = $env:ANDROID_HOME
Push-Location $Sdk
.\gradlew.bat --offline --no-daemon --console=plain :TeamCode:assembleDebug
Pop-Location

# 3. install on the Control Hub
$adb = "$env:ANDROID_HOME\platform-tools\adb.exe"
& $adb devices -l
& $adb install -r "$Sdk\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk"
```

The APK ends up at:

```
C:\Users\tim\ftc-build\ftc-sdk\TeamCode\build\outputs\apk\debug\TeamCode-debug.apk
```

---

## When it goes wrong anyway

| symptom | cause |
|---|---|
| a new OpMode is missing from the Driver Station | the APK on the hub predates it — it was never built, or the build never copied the file into the SDK. Compare the APK's build time with the repo file's time. |
| an OpMode you deleted is still on the Driver Station | the stale `.java` is still in the SDK copy, so it still compiles. Both scripts remove these; a hand-run `gradlew` does not. |
| `cannot find symbol` in a class you did not touch | a new `.java` was never linked or copied into the SDK. The error points at the wrong file by design — see above. |
| `File ... cannot be loaded because running scripts is disabled` | PowerShell execution policy. Add `-ExecutionPolicy Bypass` for that one invocation, or fix it for your account with `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`. |
| `no device attached` | the hub is not on USB-C, or the cable is charge-only. It must be in **this** PC, not the Driver Hub, and its LED must be green. |
| gradle says the directory "does not contain a Gradle build" | it was run from the wrong working directory. Gradle takes the project from the working directory, not from where `gradlew.bat` lives — both scripts `cd` for this reason. |
| the Driver Station link drops mid-install | expected. It restores itself. |

Timings, as a rough guide: a clean build is around **15 minutes**; an incremental build of
just the OpModes is **30-60 seconds**. If a build seems stuck, check whether it is a clean
one before worrying.

---

## Before you run it

**Wheels off the ground (on blocks) for the first run** of anything that drives, aims, or
spins the launcher. `deploy.ps1` prints that reminder when it finishes, and it is not
decoration: a mis-aimed autonomous that cannot find its target tags will sweep a full
rotation, and the launcher spins up as soon as a tag comes into view.
