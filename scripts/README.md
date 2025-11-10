# load_kailbr_results.sh

1.转换标定参数到 seeker 默认配置文件夹中
```bash
./scripts/load_kailbr_results.sh path/to/kalibr_yaml
```
2.执行 `/param/convert_kailbra_to_diffvio.py` 脚本生成 vio 配置文件 \

3.将步骤 2 生成的 `cam{cam_index}_fisheye.yaml` 直接拷贝到 `df_visual_fronted/config/f4_v2` 中覆盖 \

4.将步骤 2 生成的 `camera_tf.yaml` 中 tf 矩阵拷贝到`df_visual_fronted/config/f4_v2/f4_v2_virtural_stereo.yaml` 中对应位置 \

5.运行一次 vio ，将 docker 内生成的 `/root/.ros/virtual_stereo_ex_param.txt` 中参数拷贝到 `fusion_odometry/config/virtual_stereo_seek_v2.yaml` 中对应位置

---

# load_policy.sh

1.导出策略到默认 `drone_control/model` 文件夹中
``` bash
./scripts/load_policy.sh path/to/model.pt
```
2.修改 `drone_control/config` 下模型加载路径

---

# run.sh

启动任务全部节点
``` bash
./scripts/run.sh
```

---

# stop.sh

停止任务全部节点
``` bash
./scripts/stop.sh
```