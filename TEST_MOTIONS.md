# Test Motions

We list some test motions here that are not easy for retargeting (i.e., sometimes giving bad results).
The [x] means the motion is retargeted successfully.


# AMASS

- [x] BMLrub_rub081_0031_rom_stageii.npz
- [x] KIT_3_walk_6m_straight_line04_stageii.npz

# LAFAN1

- [x] dance1_subject2.bvh

# pnd_adam_lite 执行AMASS

将pkl修改为npz格式，首先需要修改SMPL-X库的文件

```
# 找到smplx安装位置
python -c "import smplx; print(smplx.__file__)"
```

输出
```
/home/luyd/miniconda3/envs/gmr/lib/python3.10/site-packages/smplx/__init__.py
```
更具路径进行替换
```
# 备份原文件
cp /home/luyd/miniconda3/envs/gmr/lib/python3.10/site-packages/smplx/body_models.py /home/luyd/miniconda3/envs/gmr/lib/python3.10/site-packages/smplx/body_models.py.backup

# 替换npz为pkl
sed -i "s/'npz'/'pkl'/g" /home/luyd/miniconda3/envs/gmr/lib/python3.10/site-packages/smplx/body_models.py
```
验证修改是否成功
```
grep -n "pkl\|npz" /home/luyd/miniconda3/envs/gmr/lib/python3.10/site-packages/smplx/body_models.py
```
现在应该看到 pkl 而不是 npz。

## Record one npz data
```
python scripts/smplx_to_robot.py \
    --smplx_file /home/luyd/workspace/GMR.bak/data/ACCAD/Male2Running_c3d/C3_-_run_stageii.npz \
    --robot pnd_adam_lite \
    --save_path logs/adam.pkl \
    --rate_limit \
    --record_video 
```

```
python scripts/smplx_to_robot.py \
    --smplx_file /home/luyd/workspace/GMR.bak/data/ACCAD/Male2Running_c3d/C3_-_run_stageii.npz \
    --robot pnd_adam_lite \
    --rate_limit \
    --record_video \
    --video_width 1920 \
    --video_height 1080 \
    --video_fps 60
```
## Record a folder's npz data
```
cd /home/luyd/workspace/GMR.bak && for f in /home/luyd/workspace/GMR.bak/data/ACCAD/Male2Running_c3d/*.npz; do \
  python3 scripts/smplx_to_robot.py --smplx_file "$f" --robot pnd_adam_lite --rate_limit --record_video; \
done
```
```
cd /home/luyd/workspace/GMR.bak && for f in /home/luyd/workspace/GMR.bak/data/ACCAD/Male2Running_c3d/*.npz; do \
  python scripts/smplx_to_robot.py \
    --smplx_file "$f" \
    --robot pnd_adam_lite \
    --rate_limit \
    --record_video \
    --video_width 1920 \
    --video_height 1080 \
    --video_fps 60; \
done
```