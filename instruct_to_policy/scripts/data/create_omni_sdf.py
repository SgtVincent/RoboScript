""" 
Creates Gazebo compatible SDF files from google scanned objects.
Credit: https://github.com/sea-bass/ycb-tools by Sebastian Castro 2020-2021
Modified by Junting Chen in Sep. 2023
"""


import os
import trimesh
import argparse
import numpy as np
# import rospkg
import json 

if __name__=="__main__":

    # Define folders
    default_models_folder = "/home/junting/Downloads/Compressed/bottle"
    # default_template_file = os.path.join("data", "google_scanned_object", "template.sdf")
    default_template_file = "/home/junting/repo/ycb-tools/templates/ycb/template_omni.sdf"
    default_metadata_file = os.path.join("data", "google_scanned_object", "container_metadata.json")
    # default_metadata_file = os.path.join("data", "google_scanned_object", "object_metadata.json")

    # Parse arguments
    parser = argparse.ArgumentParser(description="Create Gazebo SDF files for Google scanned objects.")
    parser.add_argument("--downsample-ratio", type=float, default=1,
                        help="Mesh vertex downsample ratio (set to 1 to leave meshes as they are)")

    parser.add_argument("--models-folder", type=str, default=default_models_folder,
                        help="Folder path of google scanned objects models (defaults to data/google_scanned_object/models)")
    parser.add_argument("--template-file", type=str, default=default_template_file,
                        help="Folder path of sdf template file (defaults to data/google_scanned_object/template.sdf)")
    parser.add_argument("--metadata-file", type=str, default=default_metadata_file,
                        help="Path to the metadata file (defaults to data/google_scanned_object/container_metadata.json)")

    parser.add_argument("--use_voxel_volume", action="store_true", default=False)

    args = parser.parse_args()
    args.use_voxel_volume = True
    
    # wait for the user to press enter
    print("""
          Make sure you run this script from the root of the instruct_to_policy package! 
          `python scripts/data/create_google_scanned_objects_sdf.py`
    """)

    # Get the list of models from the metadata file
    # with open(args.metadata_file, "r") as f:
    #     metadata = json.load(f)
    #     models = list(metadata['objects'].keys())
    models = os.listdir(args.models_folder)

    with open(args.template_file,"r") as f:
        model_template_text = f.read()
    # with open(material_template_file,"r") as f:
    #     material_template_text = f.read()

    # Now loop through all the folders
    failed_models = {}
    for model in models[:1]:
        try:
            print("Creating Gazebo files for {} ...".format(model))

            # Extract model name and mesh 
            model_folder = os.path.join(args.models_folder, model)
            mesh_file = os.path.join(model_folder, "Scan", "Scan.obj")
            mesh = trimesh.load(mesh_file)

            # If mesh is not water tight, use its convex hull instead
            if not mesh.is_watertight:
                print("Mesh is not watertight. Use its convex hull...")
                mesh = mesh.convex_hull
                assert mesh.is_watertight
                
            # give containers larger density for stability
            mesh.density = 5e-6
            # change density according to the object
            if mesh.mass < 1: # less than 1000g
                mesh.density = 1e-5
            
            # # give objects smaller density 
            # mesh.density = 100.0
            # # change density according to the object
            # if mesh.mass < 0.1: # less than 100g
            #     mesh.density = 500.0
            
            # Mass and moments of inertia
            mass_text = str(mesh.mass)
            tf = mesh.principal_inertia_transform
            inertia = trimesh.inertia.transform_inertia(tf, mesh.moment_inertia)

            # Center of mass
            com_vec = mesh.center_mass.tolist()
            eul = trimesh.transformations.euler_from_matrix(np.linalg.inv(tf), axes="sxyz")
            com_vec.extend(list(eul))
            com_text = str(com_vec)
            com_text = com_text.replace("[", "")
            com_text = com_text.replace("]", "")
            com_text = com_text.replace(",", "")
        
            # Create a downsampled mesh file with a subset of vertices and faces
            collision_mesh_text = model + "/Scan/Scan.obj"
            
            if args.downsample_ratio < 1:
                mesh_pts = mesh.vertices.shape[0]
                num_pts = int(mesh_pts * args.downsample_ratio)
                (_, face_idx) = mesh.sample(num_pts, True)
                downsampled_mesh = mesh.submesh((face_idx,), append=True)
                with open(os.path.join(model_folder, "/Scan/downsampled.obj"), "w") as f:
                    downsampled_mesh.export(f, "obj")
                collision_mesh_text = model + "/Scan/downsampled.obj"

        
            # Copy and modify the model file template
            model_text = model_template_text.replace("$MODEL", model)
            model_text = model_text.replace("$MASS", mass_text)
            model_text = model_text.replace("$COM_POSE", com_text)
            model_text = model_text.replace("$COLLISION_MESH", collision_mesh_text)
            model_text = model_text.replace("$IXX", str(inertia[0][0]))
            model_text = model_text.replace("$IYY", str(inertia[1][1]))
            model_text = model_text.replace("$IZZ", str(inertia[2][2]))
            model_text = model_text.replace("$IXY", str(inertia[0][1]))
            model_text = model_text.replace("$IXZ", str(inertia[0][2]))
            model_text = model_text.replace("$IYZ", str(inertia[1][2]))
            # overwrite the sdf file
            with open(os.path.join(model_folder, model + ".sdf"), "w") as f:
                f.write(model_text)

            config_template_text = """
<?xml version="1.0" ?>
<model>
    <name>$MODEL_NAME</name>
    <version>1.0</version>
    <sdf version="1.6">$MODEL_NAME.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>
            """
                
            # Copy and modify the model configuration file template
            config_text = config_template_text.replace("$MODEL_NAME", model)
            
            with open(os.path.join(model_folder, "model.config"), "w") as f:
                f.write(config_text)

        except Exception as e:
            failed_models[model] = e

    print("Failed to create Gazebo files for the following models:")
    for model, e in failed_models.items():
        print("|--{}: {}".format(model, e))
    print("Done.")
