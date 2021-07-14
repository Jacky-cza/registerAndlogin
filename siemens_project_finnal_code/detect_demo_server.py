# coding=utf-8

import argparse
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random
import pickle
import sys
import socket
import json
from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, non_max_suppression, apply_classifier, scale_coords, \
    xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier


def detect(img_np):
    imgsz = 640

    # 图片
    # source = opt.source
    set_logging()

    # 选择cpu或者GPU
    '''
    如果指定了CPU则使用CPU，如果cuda可用，将默认使用cuda:0
    '''
    device = select_device('')
    '''
    如果使用cpu，float数据长度将减半
    '''
    half = device.type != 'cpu'
    retResult = {}
    retResult['tilt'] = False
    retResult['tilt_conf'] = 0
    retResult['class'] = ''
    retResult['class_conf'] = 0
    retResult['rotate'] = 0
    retResult['boundbox'] = {}

    for counter in range(2):    # 循环分别进行倾倒和检测分类
        if (counter == 0):
            weights = './tilt.pt'
        else:
            weights = './classification.pt'
        model = attempt_load(weights, map_location=device)      # 加载网络模型
        imgsz = check_img_size(imgsz, s=model.stride.max())     # 检查图片大小是否满足网络
        if half:
            model.half() 
        # dataset = LoadImages(source, img_size=imgsz)            # 加载待检测的图片

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]    # 根据不同的模型随即生成不同的颜色

        # Run inference
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)
        _ = model(img.half() if half else img) if device.type != 'cpu' else None
        # for path, img, im0s, vid_cap in dataset:        # img transpose后的图像，im0s原图像，vid_cap视频
        img_np = img_np.reshape([480, 640, 3])
        imgTranspose = img_np[:, :, ::-1].transpose(2, 0, 1)
        imgTranspose = np.ascontiguousarray(imgTranspose)
        img = torch.from_numpy(imgTranspose).to(device)  # 将图片数据转换到指定设备上
        img = img.half() if half else img.float()
        img /= 255.0  # 归一化
        if img.ndimension() == 3:
            img = img.unsqueeze(0)  # 图像维度变为：（1, 3, height, width）

        pred = model(img, augment=False)[0]  # 前向输出预测

        pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=None)
        # 非极大值抑制

        print("\n")

        # Process detections
        for i, det in enumerate(pred):
            # s, im0 = '', img_np
            s, im0 = '', img_np
            s += '%gx%g ' % img.shape[2:]  # 科学计数法
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]

            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # post processing: draw rect, label and confidence
                for *xyxy, conf, cls in reversed(det):  # bounding box, confidence, class
                    label = f'{names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

                cv2.imwrite('result.png', im0)

                conf = det.cpu().numpy().tolist()[0][4]

                # Print results
                for c in det[:, -1].unique():

                    n = (det[:, -1] == c).sum()
                    s += f'{n} {names[int(c)]}s, '
                    if names[int(c)] == '1':
                        print("\n********TILT, BREAK**************\n")
                        print(" \n ****** CONF1: ", conf, "\n")
                        retResult['tilt'] = True
                        retResult['tilt_conf'] = conf
                        # sys.exit()

                    if (names[int(c)] == '0' and counter == 0):
                        print("\n********NOT TILT**************\n ")
                        print(" \n ****** CONF1: ", conf, "\n")
                        retResult['tilt'] = False
                        retResult['tilt_conf'] = conf

                    if (counter == 1):
                        boundbox = det.cpu().numpy().tolist()[0][0:4]
                        print("\n ****** Boundingbox: ", boundbox, "\n ")
                        print("\n****** Class:", names[int(c)], "***************\n")
                        print(" \n ****** CONF2: ", conf, "\n")
                        retResult['class'] = names[int(c)]
                        retResult['class_conf'] = conf
                        retResult['boundbox'] = {'xmin': boundbox[0], 'ymin': boundbox[1], 'xmax': boundbox[2], 'ymax': boundbox[3]}
    return retResult
                    

if __name__ == '__main__':
    check_requirements()

    print ("start server...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.bind(('192.168.1.20', 3000))
    s.listen(1)
    while True:
        try:
            print('accept')
            conn, addr = s.accept()
            conn.settimeout(1)
            print('client connected {} {}'.format(conn, addr))
            imgSource = bytearray()
            while True:
                try:
                    data = conn.recv(1024)
                    # print('data len ', len(data))
                    if len(data) == 0:
                        break
                    imgSource += data
                    print('imgSource len: ', len(imgSource))
                except socket.timeout as e:
                    img = pickle.loads(imgSource, encoding='bytes')
                    img_array = np.fromstring(img, dtype=np.uint8)
                    retResult = detect(img_array)
                    print ("retResult:",retResult)
                    conn.send(json.dumps(retResult).encode('utf-8'))
                    # conn.close()
                    break
        except KeyboardInterrupt as e:
            conn.close()
            s.close()
            sys.exit(0)


