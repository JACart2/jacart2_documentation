# JACart2 Version Semantics

JACart2 spans several repositories that must be deployed together. Versions follow `MAJOR.MINOR.PATCH`, and the **major version identifies the localization backend**:

|Version range|Localization backend|Status|
|-|-|-|
|**`0.X.Y`**|Original LiDAR localization (no MOLA), using `lidar_localization_ros2`|Frozen (reference only)|
|**`1.X.Y`**|MOLA LiDAR localization and mapping (MRPT / `mola_\*` ROS packages)|**Active**|

All active development targets `1.X.Y`. The `0.X.Y` series is preserved so the pre-MOLA system can be reproduced, but it receives no new features or fixes.

> \*\*Note:\*\* this project uses `0.X.Y` to mean "pre-MOLA", not "unstable" as in conventional SemVer. A `0.X.Y` release was a working, deployed system.

\---

## 1\. Repositories covered

|Repository|Role|Differs between 0.X.Y and 1.X.Y?|
|-|-|-|
|`ai-navigation`|ROS 2 navigation, localization launch files, cart configs|**Yes** (localization, launch, map/calibration files)|
|`docker_files`|Docker images, compose files, launcher scripts|**Yes** (backend image packages, MOLA dependencies, map downloads)|
|`ui`|Passenger-facing interface|Mostly no|
|`dashboard`|Remote monitoring dashboard|No|
|`anomaly_detection`|AI anomaly detection, bag tools, offline evaluation|No (understands MOLA health messages)|

Every repository uses the same version scheme and is tagged `vMAJOR.MINOR.PATCH`.

\---

## 2\. What each number means

|Component|Meaning|Increment when...|
|-|-|-|
|**MAJOR**|Localization backend generation|The localization backend changes. `0` is classic; `1` is MOLA. A future backend change would start `2.0.0`.|
|**MINOR**|Feature or interface release|A feature is added, **or** any shared interface changes (messages, topics, map format, coordinate conventions). Breaking changes must be marked `BREAKING` in the release notes.|
|**PATCH**|Fix|A bug fix, tuning change, documentation update, or dependency pin bump that does not change behavior contracts.|

Because MAJOR is reserved for the backend, interface-breaking changes bump MINOR instead.

### Compatibility rule

All repositories on a cart must share the same **MAJOR.MINOR**. PATCH versions may differ freely.

|Cart repositories|Compatible?|
|-|-|
|`1.3.0`, `1.3.2`, `1.3.1`|Yes|
|`1.3.x` with `1.2.x`|No|
|`1.x.y` with `0.x.y`|No|

\---

## 3\. Branches and tags

|Purpose|Name|
|-|-|
|Active development|`main` (after the migration below)|
|Feature work|`<author>/<short-description>`|
|Term/season integration branches|`<topic>_<season><yy>` (e.g. `fall26`)|
|Release tag|`vMAJOR.MINOR.PATCH`|
|Whole-cart release|`suite-vMAJOR.MINOR.PATCH` on `docker_files`|
|Frozen classic reference|`release/0.x` branch and the final `v0.X.Y` tag|

> \*\*Migration note:\*\* historical branches do not follow this scheme. Current MOLA work lives on `mola_main_sum26` (and `fall26\*` for anomaly detection), while `main` in `ai-navigation`, `ui`, and `docker_files` predates the MOLA integration. To migrate:
> 1. Tag the existing `main` with the final classic version (`v0.X.Y`) and create `release/0.x` from it.
> 2. Merge the MOLA branches into `main`.
> 3. Tag the result `v1.0.0`.

\---

## 4\. Compatibility matrix

Fill in exact tags when releasing. The "Current head" column reflects the project state as of late October 2026 and should be verified before tagging.

### 1.X.Y (MOLA, active)

|Repository|Current head|Release tag|
|-|-|-|
|`ai-navigation`|`main`|`v1.X.Y` (TBD)|
|`docker_files`|`fall26`|`v1.X.Y` (TBD)|
|`ui`|`main`|`v1.X.Y` (TBD)|
|`dashboard`|`main`|`v1.X.Y` (TBD)|
|`anomaly_detection`|`fall26`|`v1.X.Y` (TBD)|

### 0.X.Y (classic, frozen)

|Repository|Reference point|
|-|-|
|`ai-navigation`|`main` before the MOLA merge (final `v0.X.Y` tag, TBD)|
|`docker_files`|`main` before the MOLA merge (final `v0.X.Y` tag, TBD)|
|`ui`|`main` before the MOLA merge (final `v0.X.Y` tag, TBD)|

### Shared interface contract

Changing any of these requires a MINOR bump in every repository that uses it, flagged `BREAKING` in the release notes.

|Interface|Used by|
|-|-|
|`anomaly_msg/msg/AnomalyMsg` on `/ai_anomaly_logging`|navigation, UI, dashboard, anomaly detection|
|`/aad/alerts`, `/aad/decisions`|anomaly detection, dashboard|
|`/gps_request`, `/gps_send`, `/gps_global_path`|navigation, UI|
|`/eta`, `/eta_percentage`|navigation, UI, dashboard|
|ROSBridge on port `9090`|UI, dashboard|

\---

## 5\. What changed from 0.X.Y to 1.X.Y

|Area|0.X.Y (classic)|1.X.Y (MOLA)|
|-|-|-|
|Localization|`lidar_localization_ros2`|MOLA LiDAR localization with relocalization and a pose relay for the navigation stack|
|Mapping|Previous map workflow|MOLA mapping (loop closure, leveling, georeferencing); `mola_sm_loop_closure`|
|Backend image|No MOLA packages|`ros-jazzy-mola-\*`, `ros-jazzy-gtsam`, `ros-jazzy-mp2p-icp`, `ros-jazzy-mrpt-\*`, `mola_sm_loop_closure`|
|Map and config files|Classic map set|MOLA metric maps and calibration files (downloaded into `/maps` during image build)|
|Launch / RViz|Classic localization launch files|`mola_mapper.launch.py`, `mola_localization.rviz`, optional `zed_single.launch.py`|
|Health reporting|Not applicable|`mola_diagnostics` messages (ICP quality, dropped-frame ratio, active state) on `/ai_anomaly_logging`|
|Optional ZED odometry|Not applicable|Developmental input to MOLA state estimation|

\---

## 6\. Checking out a version

Replace branch names with tags once releases are cut.

```bash
# Active (1.X.Y)
git -C \~/dev_ws/src/ai-navigation     checkout mola_main_sum26
git -C \~/docker_files                 checkout fall26-bag_file_ui
git -C \~/ui                           checkout mola_main_sum26
git -C \~/dev_ws/src/anomaly_detection checkout fall26
```

Afterwards:

1. Rebuild the backend image.
2. Rebuild the ROS workspace with `colcon build`.
3. Confirm the MOLA maps are present in `/maps`.
4. Confirm the MOLA launch files are in use.

To reproduce classic behavior only, check out the final `v0.X.Y` tag (or `release/0.x`) in each line-sensitive repository. Do not base new work on it.

\---

## 7\. Cutting a release

1. Confirm all repositories pass their checks and share the same MAJOR.MINOR.
2. Update the compatibility matrix in this file.
3. Tag each repository: `git tag -a vX.Y.Z -m "<summary>"` and push the tag.
4. Tag the suite: `suite-vX.Y.Z` on `docker_files`, listing per-repository tags in the release notes.
5. Record field-test status (stationary check, offline route inspection, or driven route) in the release notes so that "developmental" and "validated" features are distinguishable.

\---

## 8\. Classic (0.X.Y) retirement

* The `0.X.Y` series receives no new features or fixes.
* Existing code is preserved under the final `v0.X.Y` tag and the `release/0.x` branch.
* All new features, fixes, and releases are `1.X.Y`.
