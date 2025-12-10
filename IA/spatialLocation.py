import depthai as dai

pipeline = dai.Pipeline()

# Caméras mono pour la stéréo
mono_left = pipeline.create(dai.node.MonoCamera)
mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

mono_right = pipeline.create(dai.node.MonoCamera)
mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# Stereo depth
stereo = pipeline.create(dai.node.StereoDepth)
stereo.setOutputDepth(True)
stereo.setSubpixel(True)
stereo.setLeftRightCheck(True)
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Spatial location calculator
spatial = pipeline.create(dai.node.SpatialLocationCalculator)
spatial.setWaitForConfigInput(False)

# ROI centrale 40%-60% avec seuils profondeur
cfg = dai.SpatialLocationCalculatorConfigData()
cfg.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
cfg.depthThresholds.lowerThreshold = 100   # 10 cm
cfg.depthThresholds.upperThreshold = 10000 # 10 m
cfg.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
spatial.initialConfig.addROI(cfg)

# Connexions
stereo.depth.link(spatial.inputDepth)

# Sortie
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("spatialData")
spatial.out.link(xout.input)

def pixel_to_spatial(u, v, depth_mm, fx, fy, cx, cy):

    z_meters = depth_mm / 1000.0
    
    # x = (u - cx) * z / fx
    x_meters = (u - cx) * z_meters / fx
    y_meters = (v - cy) * z_meters / fy
    
    return x_meters, y_meters, z_meters

with dai.Device(pipeline) as device:
    calib = device.readCalibration()
    # Intrinsèques de la caméra RGB (CAM_A) à la résolution 1080p par défaut
    K = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, 1920, 1080)
    fx, fy = K[0][0], K[1][1]
    cx, cy = K[0][2], K[1][2]
    q = device.getOutputQueue("spatialData", maxSize=4, blocking=False)
    print("Pipeline démarré. Ctrl+C pour quitter.")
    try:
        while True:
            msg = q.get()
            locs = msg.getSpatialLocations()
            loc = locs[0].spatialCoordinates
            if not locs:
                continue
            x,y,z  = pixel_to_spatial(loc.x,loc.y,loc.z,fx,fy,cx,cy)
            
            

            print(f"X={x:.1f} mm, Y={y:.1f} mm, Z={z:.1f} mm")
    except KeyboardInterrupt:
        print("\nArrêt du programme.")
