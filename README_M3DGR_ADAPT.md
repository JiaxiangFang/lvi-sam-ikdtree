# LVI-SAM-IKD（lvi_sam_ikdtree）M3DGR 适配修改说明

> 本文只记录 **本项目（`lvi_sam_ikdtree`，ikd-tree / FAST-LIO 式后端）** 相对原版的修改。
> 完整的根因诊断与通用结论见参考项目 `LVI_SAM_M3DGR/src/lvi_sam/README_M3DGR_ADAPT.md`。

- 工作区根：`/home/jiax/study/LVI-SAM-IKD`
- 包路径：`src/lvi-sam-ikdtree`，包名 `lvi_sam_ikdtree`
- 构建：

  ```bash
  cd /home/jiax/study/LVI-SAM-IKD
  catkin_make --only-pkg-with-deps lvi_sam_ikdtree
  ```

- 运行 mid360 前需先手动启动 `livox_repub`（`/livox/mid360/lidar` → `/livox/lidar`）。

---

## 一、架构差异（与参考项目对照）

| 层 | 参考 `lvi_sam` | 本项目 `lvi_sam_ikdtree` | 移植影响 |
|---|---|---|---|
| LIO 后端 | gtsam `mapOptmization.cpp` | ikd-tree `mapOptmization_ikdtree.cpp`（FAST-LIO 式） | **不同**，外参/IMU/视觉适配不在此层 |
| 视觉前端 | `feature_tracker.h` get_depth | `feature_tracker.h` `DepthRegister::getDepth` | 同构，共享层 |
| 视觉外参 | `L_*_I / C_*_L` | `IMU_TO_LIDAR_* / L_C_*` | 同构，共享层 |
| IMU 姿态 | `mahonyMine` | `mahonyMine` | 同构，共享层 |

> 结论：所有适配改动都落在 **视觉 / 外参 / IMU 共享层**，与 LIO 后端无关，因此参考项目的方案可直接套用。

---

## 二、配置修改

### `config/m3dgr/params_camera_mid360.yaml`

| 参数 | 值 | 说明 |
|---|---|---|
| `use_lidar` | `0 → 1` | 启用激光深度（与 avia 一致；之前设 0 是为躲 NaN，现 NaN 根因已修复） |
| `lidar_to_cam_t` | `[-0.0442358, -0.411712, 0.168568]` | 官方 "mid360 2 camera" T |
| `lidar_to_cam_r` | `[0.1152925, -1.0169222, 1.4330560]` | 官方 "mid360 2 camera" R 标准 (roll,pitch,yaw) 分解 |
| `imu_to_lidar_t` | `[0.046263, -0.00420981, -0.460019]` | 官方 "camera_imu 2 mid360" T |
| `imu_to_lidar_r` | `[-1.0288151, -0.0113750, -1.4724742]` | 官方 "camera_imu 2 mid360" R 标准分解（**勿对调 roll/yaw**） |
| `extrinsicRotation/Translation` | 官方 "camera 2 camera_imu" | RIC/TIC |
| IMU 噪声 | `acc_n / gyr_n / acc_w / gyr_w / g_norm` | **必须 VINS 命名**，写成 LIO 名会被静默读 0 → IMU 因子 NaN |

### `config/m3dgr/params_lidar_mid360.yaml`

| 参数 | 值 |
|---|---|
| `extrinsicRot` | 官方 "camera_imu 2 mid360" R `[0.0981574,0.514299,0.851975, -0.995106,0.0409386,0.0899349, 0.0113748,-0.856633,0.515801]` |
| `extrinsicRPY` | `extrinsicRot` 的转置 |
| `extrinsicTrans` | `[0.046263, -0.00420981, -0.460019]` |
| `N_SCAN / Horizon_SCAN / imuGravity` | `4 / 6000 / 9.79` |
| IMU 噪声 | LIO 命名 `imuAccNoise / imuGyrNoise / imuAccBiasN / imuGyrBiasN / imuGravity` |

> 注：以上多数配置由前序工作已适配，本轮仅改动 `use_lidar: 0 → 1`。

---

## 三、代码修改

### 1. `src/lidar_odometry/mahonyMine.h` / `mahonyMine.cpp` — 6 轴 IMU accel-init

6 轴 RealSense d435i 无 orientation，Mahony 从单位四元数启动需秒级收敛到 mid360 的 ~58° 倾角，VINS 初始化会采到未收敛姿态。

- `mahonyMine.h`：新增成员 `bool initialized;` 与方法声明 `void initOrientationFromAccel(float ax, float ay, float az);`
- `mahonyMine.cpp`：
  - 构造函数置 `initialized = false;`
  - `initOrientationFromAccel`：首帧用重力方向直接置四元数
    ```
    roll  = atan2(ay, az)
    pitch = atan2(-ax, sqrt(ay^2 + az^2))
    yaw   = 0
    ```
  - `MahonyAHRSupdateIMU` 开头首帧守卫：
    ```cpp
    if (!initialized) {
        if (!(ax == 0 && ay == 0 && az == 0)) { initOrientationFromAccel(ax, ay, az); initialized = true; }
        return;
    }
    ```

> 对水平 avia 是数学恒等（重力 ≈ (0,0,1) → 单位四元数），**零回归**。`sampleFreq` 保持 1000（改 200 会破坏 avia，已证）。

### 2. `src/visual_odometry/visual_feature/feature_tracker.h` — 特征可视化对齐

`getDepth` 末尾：发布前把 `features_3d_sphere` 从 lidar_normal（相机朝向 FLU）逆旋转回 lidar_original 轴向并补回相机光心偏移：

```cpp
original_T_normal.linear()      = normal_R_original.linear().transpose();
original_T_normal.translation() = Eigen::Vector3f(L_C_TX, L_C_TY, L_C_TZ);
// transformPointCloud(...) 后发布 features_3d_publish
```

> 纯可视化修复（修正特征点在 RViz 多转 ~58° 的标签错位），**不影响喂给 VINS 的深度标量**。变量名差异：ikdtree 用 `L_C_T*`（非参考的 `C_T_L`），Affine 变量名为 `normal_R_original`。

---

## 四、轨迹保存功能（Ctrl+C 自动保存）

输出 **TUM 格式**：`timestamp tx ty tz qx qy qz qw`。

### `src/lidar_odometry/utility.h`

- 新增成员：`bool saveTrajectory; string saveTrajectoryDirectory;`
- `ParamServer()` 内读取：
  ```cpp
  nh.param<bool>(PROJECT_NAME + "/saveTrajectory", saveTrajectory, false);
  nh.param<std::string>(PROJECT_NAME + "/saveTrajectoryDirectory", saveTrajectoryDirectory, "/tmp/lvi_sam_trajectory.txt");
  ```

### `src/lidar_odometry/mapOptmization_ikdtree.cpp`

- 新增 `saveFinalTrajectory()`：遍历 `globalPath.poses`（最终全局轨迹，含回环修正）按 TUM 格式写文件；支持 `~` 展开为 `$HOME`。
- `main()` 内 `ros::spin()` 返回后（即 Ctrl+C 时）调用 `MO.saveFinalTrajectory();`

### `config/m3dgr/params_lidar_mid360.yaml` / `params_lidar_avia.yaml`

```yaml
saveTrajectory: true
saveTrajectoryDirectory: "~/lvi_sam_ikd_traj_mid360.txt"   # avia 为 ~/lvi_sam_ikd_traj_avia.txt
```

> 开关默认 `false`（代码默认值），配置里设 `true` 启用。运行结束 Ctrl+C 后终端打印 `Trajectory saved (N poses) to: ...`。

---

## 五、evo 评估（仅 xyz 位置误差）

```bash
# GT 若有重复时间戳，先清洗
awk '!seen[$1]++' gt.txt > gt_clean.txt

# APE translation part，SE3 对齐（论文用 -a；LVI 系统勿用 -s）
evo_ape tum gt_clean.txt ~/lvi_sam_ikd_traj_mid360.txt -va -r trans_part -a --t_max_diff 0.1
```

- 真值高频、估计稀疏（仅关键帧）无需行数相同，evo 按时间戳就近匹配。
- 匹配失败多为 `--t_max_diff` 太小（默认 0.01s），关键帧间隔大时放宽到 0.05~0.1。
- 论文报 ATE 用 `-a`（SE3 对齐，不估尺度），所有对比方法须用相同对齐设置。

---

## 六、编译验证

```
[100%] Built target lvi_sam_ikdtree_mapOptmization
CATKIN_EXIT=0
```

全部 target（visual_feature / visual_odometry / imuPreintegration / mapOptmization / featureExtraction / imageProjection / visual_loop）构建成功。

---

## 七、注意事项 / 铁律

1. mid360 运行前必须先启 `livox_repub` 的 mid360_repub.launch。
2. 相机 yaml 三处旋转（`lidar_to_cam_r`、`imu_to_lidar_r`、RIC）**都用标准 (roll,pitch,yaw) 分解，绝不对调**——avia 的对调写法只是近 -90° pitch 的万向锁巧合。
3. IMU 噪声两套独立：LIO 用 `imuAccNoise...`，VINS 用 `acc_n...`，**绝不可混**（OpenCV FileStorage 对缺失 key 静默返回 0 → IMU 因子 NaN）。
4. 起步偶发一次 VINS reboot + 自愈是 mid360 弱垂直约束的固有现象，非 bug。
