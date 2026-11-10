import LuMoSDKClient

ip = "169.254.209.149"

#初始化
LuMoSDKClient.Init()
#设置Ip
LuMoSDKClient.Connnect(ip)

while True:
    #接收数据
    frame = LuMoSDKClient.ReceiveData(0) # 0 :阻塞接收 1：非阻塞接收
    if frame is None:
        continue
    FrameID = frame.FrameId
    print(FrameID)
    TimeStamp = frame.TimeStamp
    print(TimeStamp)
    markers = frame.markers
    for marker in markers:
        print(marker.Id)
        print(marker.Name)
        print(marker.X)
        print(marker.Y)
        print(marker.Z)
    for rigid in frame.rigidBodys:
        print(rigid.Id)
        print(rigid.Name)
        print(rigid.X)
        print(rigid.Y)
        print(rigid.Z)
        print(rigid.qx)
        print(rigid.qy)
        print(rigid.qz)
        print(rigid.qw)
        print(rigid.IsTrack)
        print(rigid.QualityGrade)
        print(rigid.speeds.fSpeed)
        print(rigid.speeds.XfSpeed)
        print(rigid.speeds.YfSpeed)
        print(rigid.speeds.ZfSpeed)
        print(rigid.acceleratedSpeeds.fAcceleratedSpeed)
        print(rigid.acceleratedSpeeds.XfAcceleratedSpeed)
        print(rigid.acceleratedSpeeds.YfAcceleratedSpeed)
        print(rigid.acceleratedSpeeds.ZfAcceleratedSpeed)
        print(rigid.eulerAngle.X)
        print(rigid.eulerAngle.Y)
        print(rigid.eulerAngle.Z)
        print(rigid.palstance.fXPalstance)
        print(rigid.palstance.fYPalstance)
        print(rigid.palstance.fZPalstance)
        print(rigid.accpalstance.AccfXPalstance)
        print(rigid.accpalstance.AccfYPalstance)
        print(rigid.accpalstance.AccfZPalstance)
    for skeleton in frame.skeletons:
        print(skeleton.Id)
        print(skeleton.Name)
        for bone in skeleton.skeletonBones:
            print(bone.Id)
            print(bone.Name)
            print(bone.X)
            print(bone.Y)
            print(bone.qx)
            print(bone.qy)
            print(bone.qz)
            print(bone.qw)
        print(skeleton.IsTrack)
    uCameraSyncTime = frame.uCameraSyncTime
    print(uCameraSyncTime)
    uBroadcastTime = frame.uBroadcastTime
    print(uBroadcastTime)
    for markerset in frame.markerSet:
        print(markerset.Name)
        for marker in markerset.markers:
            print(marker.Id)
            print(marker.Name)
            print(marker.X)
            print(marker.Y)
            print(marker.Z)
    print(frame.forcePlate.Fx)
    print(frame.forcePlate.Fy)
    print(frame.forcePlate.Fz)
    print(frame.forcePlate.Mx)
    print(frame.forcePlate.My)
    print(frame.forcePlate.Mz)
    print(frame.forcePlate.Lx)
    print(frame.forcePlate.Lz)


LuMoSDKClient.Close()