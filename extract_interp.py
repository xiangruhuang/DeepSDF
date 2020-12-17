import glob
import open3d as o3d
import numpy as np
import os

#category = '03001627'
#epoch=100
#exp_dir = 'examples/chairs'

category = '02691156'
epoch=320
exp_dir = 'examples/planes'

RAW = 'data/ShapeNetV2/{}/{{}}/models/model_normalized.obj'.format(category)

INTERP_DIRS = '{}/Reconstructions/{}/Interpolations/ShapeNetV2/{}'.format(
                exp_dir, epoch, category)
INTERP = os.path.join(INTERP_DIRS, '{}_{}/{}.ply')

dirs = glob.glob('{}/*_*'.format(INTERP_DIRS))
print(dirs)

for dir_ in dirs:
  m0, m1 = dir_.split('/')[-1].split('_')
  mesh0 = o3d.io.read_triangle_mesh(RAW.format(m0))
  mesh1 = o3d.io.read_triangle_mesh(RAW.format(m1))
  meshes = [o3d.io.read_triangle_mesh(INTERP.format(m0, m1, i)) for i in range(20)]
  meshes = meshes[::2]
  meshes = [mesh0] + meshes + [mesh1]

  for mesh in meshes:
    mesh.translate(-mesh.get_center().T)

  s0 = max([np.array(mesh.vertices).max(0)[0] - np.array(mesh.vertices).min(0)[0] for mesh in meshes]) 
  s1 = max([np.array(mesh.vertices).max(0)[1] - np.array(mesh.vertices).min(0)[1] for mesh in meshes]) 

  mesh_all = o3d.geometry.TriangleMesh()
  for i, mesh in enumerate(meshes):
    mesh.translate([s0*i, 0, 0])
    mesh_all = mesh_all + mesh
  o3d.io.write_triangle_mesh('data/interpolations/{}_{}.ply'.format(m0, m1), mesh_all)
