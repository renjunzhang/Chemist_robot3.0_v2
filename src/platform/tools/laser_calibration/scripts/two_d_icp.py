import numpy as np
import open3d as o3d
import copy

def DemoIcp():
    pipreg = o3d.pipelines.registration

    pcd = o3d.data.DemoICPPointClouds()
    src = o3d.io.read_point_cloud(pcd.paths[0])
    tar = o3d.io.read_point_cloud(pcd.paths[1])
    th = 0.02
    trans_init = np.array([
        [0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7],
        [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])

    reg = pipreg.registration_icp(
        src, tar, th, trans_init,
        pipreg.TransformationEstimationPointToPoint())

    print(reg.transformation)


    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(src)
    # 激活窗口。此函数将阻止当前线程，直到窗口关闭。
    vis.run()  # 等待用户拾取点   shift +鼠标左键
    vis.destroy_window()

    srcDraw = copy.deepcopy(src)
    tarDraw = copy.deepcopy(tar)
    srcDraw.paint_uniform_color([1, 1, 0])
    tarDraw.paint_uniform_color([0, 1, 1])
    o3d.visualization.draw_geometries([srcDraw, tarDraw])
    #
    #
    srcDraw = copy.deepcopy(src)
    tarDraw.paint_uniform_color([0, 1, 1])
    srcDraw.transform(reg.transformation)
    o3d.visualization.draw_geometries([srcDraw, tarDraw])

def GetTrans(src_np,tar_np,trans_init,th=0.2):
    """
    :param src: n*3 numpy类型
    :param tar: n*3 numpy类型
    :param trans_init: 初始转换,numpy类型
    :return: 最后的转换
    """
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(src_np) #点云数据

    tar = o3d.geometry.PointCloud()
    tar.points = o3d.utility.Vector3dVector(tar_np) #点云数据

    pipreg = o3d.pipelines.registration
    reg = pipreg.registration_icp(
        src, tar, th, trans_init,
        pipreg.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))


    srcDraw = copy.deepcopy(src)
    tarDraw = copy.deepcopy(tar)
    srcDraw.paint_uniform_color([1, 1, 0])
    srcDraw.transform(trans_init)
    tarDraw.paint_uniform_color([0, 1, 1])
    o3d.visualization.draw_geometries([srcDraw, tarDraw])
    #
    #
    srcDraw = copy.deepcopy(src)
    tarDraw.paint_uniform_color([0, 1, 1])
    srcDraw.transform(reg.transformation)
    o3d.visualization.draw_geometries([srcDraw, tarDraw])

    return reg.transformation
def ShowPointValue(points_np):
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(points_np) #点云数据
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(src)
    # 激活窗口。此函数将阻止当前线程，直到窗口关闭。
    vis.run()  # 等待用户拾取点   shift +鼠标左键
    vis.destroy_window()

if __name__ == '__main__':
    DemoIcp()