#----------------------------------------------------------
# Given a file.txt with list of mesh files - this script returns their bounds in xyz direction for each and saves it in a file mesh_name.txt
#----------------------------------------------------------
import bpy
import random
import copy
import math
import sys


def fetchObjectPoses():
  # Clear any existing meshes
  type = ["MESH"]
  candidate_list = [item.name for item in bpy.data.objects if item.type in type]
  objects_pose_list = []
  for object_name in candidate_list:
    if(object_name!="surface"):
      objects_pose_list.append((object_name, bpy.data.objects[object_name].location, bpy.data.objects[object_name].rotation_euler))
    
  return objects_pose_list


if __name__ == "__main__":
  #Change to Game Engine
  bpy.context.scene.render.engine = 'BLENDER_RENDER'
      
  # Arguments with mesh_list_file
  output_file_path = sys.argv[len(sys.argv)-2]
  scale = int(sys.argv[len(sys.argv)-1])
  # Fetch the scene state file
  objects_pose_list = fetchObjectPoses()

  fo = open(output_file_path, "w")
  lines = []
  for obj_id in range(len(objects_pose_list)):
      lines.append(str(objects_pose_list[obj_id][0]))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][1][0]/scale))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][1][1]/scale))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][1][2]/scale))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][2][0]))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][2][1]))
      lines.append(" ")
      lines.append(str(objects_pose_list[obj_id][2][2]))
      lines.append("\n")
  fo.writelines(lines)
  fo.close()
  #bpy.ops.wm.save_mainfile(filepath=folder_path+"temp.blend")
  #Read the mesh paths into a list
  # fo = open(mesh_list_file, "r")
  # mesh = fo.readlines()
  # for i in mesh:
  #   temp = str.split(i)
  #   dims = fetchDimensions(temp[0], path_to_mesh+'/'+temp[0]+'.OBJ')
  #   fi = open(path_to_mesh+'/'+temp[0]+'_dims.txt', 'w')
  #   fi.writelines(str(dims[0])+' '+str(dims[1])+' '+str(dims[2]))
  #   fi.close()
    
    

