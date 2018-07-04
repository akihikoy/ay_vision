#!/usr/bin/python
#\file    cam_info.py
#\brief   Load camera config YAML file and analyze the camera parameters.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.03, 2018
import sys
import cv2
import numpy as np

#sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/../../../ay_py/src')
#from ay_py.core import LoadYAML

import yaml
def opencv_matrix(loader, node):
  mapping = loader.construct_mapping(node, deep=True)
  mat = np.array(mapping["data"])
  mat.resize(mapping["rows"], mapping["cols"])
  return mat

yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix)

if __name__=='__main__':
  config_file= sys.argv[1] if len(sys.argv)>1 else 'config/usbcam1f1.yaml'
  #LoadYAML(config_file)

  with open(config_file) as fd:
    config_data= yaml.load(fd.read())

  for cam_info in config_data['CameraInfo']:
    for k,v in cam_info.iteritems():
      print k,':',v
    Height= cam_info['Height']
    Width= cam_info['Width']
    K= cam_info['K']
    D= cam_info['D']
    R= cam_info['R']
    Alpha= cam_info['Alpha']

    P= cv2.getOptimalNewCameraMatrix(K, D, (Width,Height), Alpha, (Width,Height))
    #cv2.initUndistortRectifyMap(K, D, R, P, (Width,Height), CV_16SC2, map1_, map2_)
    print 'P :',P
    print '----------'

