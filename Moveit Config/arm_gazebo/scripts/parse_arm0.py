#!/usr/bin/env python3

import os

def parse_arm0_file(filepath):
    objects = []

    # 只解析以下得分物体 Only parse objects that have a score
    valid_types = ["greenCan", "yellowCan", "redCan", 
                   "blueBottle", "yellowBottle", "redBottle", "pouch"]

    # 物体得分表 Object score table
    score_dict = {
        "greenCan": 10, "yellowCan": 20, "redCan": 30,
        "blueBottle": 10, "yellowBottle": 20, "redBottle": 30,
        "pouch": 10
    }

    # 读取文件
    with open(filepath, 'r') as file:
        lines = file.readlines()

    for line in lines:
        line = line.strip()
        
        # 跳过空行和注释行
        if not line or line.startswith("#"):
            continue

        data = line.split()
        
        # 解析ID、类型、位置、姿态
        objID = data[0]
        objType = data[1]

        # 仅处理得分物体
        if objType not in valid_types:
            continue

        # 解析坐标 (x, y, z)
        #x, y, z = map(float, data[2:5])
        x, y, z, roll, pitch, yaw = map(float, data[2:8])  # 解析位姿（包括旋转角）

        # 过滤低于 0.1m 的物体，避免抓取地面上的物体
        if z < 0.1:
            continue

        # 获取得分
        score = score_dict[objType]

        # **目标垃圾箱分类**
        if objType in ["greenCan", "yellowCan", "redCan", "pouch"]:
            bin_pose = [-0.4654, 0.3673, 0.0361, 0.00, 0.00, -1.57]  # Green Bin
        else:
            bin_pose = [-0.4652, -0.5901, 0.0403, 0.00, 0.00, -1.57]  # Blue Bin

        # 存储解析结果
        objects.append({
            "id": objID,
            "type": objType,
            "position": [x, y, z, roll, pitch, yaw],
            "bin": {"position": bin_pose[:3], "orientation": bin_pose[3:]},
            "score": score
        })

    # 按得分降序排列，确保高分物体优先处理
    objects = sorted(objects, key=lambda obj: obj["score"], reverse=True)
    
    return objects
