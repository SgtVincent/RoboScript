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
    # default_models_folder = os.path.join(rospkg.RosPack().get_path("instruct_to_policy"), "data", "google_scanned_object", "models")
    # default_template_file = os.path.join(rospkg.RosPack().get_path("instruct_to_policy"), "data", "google_scanned_object", "template.sdf")
    # default_metadata_file = os.path.join(rospkg.RosPack().get_path("instruct_to_policy"), "data", "google_scanned_object", "container_metadata.json")
    default_models_folder = os.path.join("data", "google_scanned_object", "models")
    default_template_file = os.path.join("data", "google_scanned_object", "template.sdf")
    default_metadata_file = os.path.join("data", "google_scanned_object", "container_metadata.json")

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

    args = parser.parse_args()
    # wait for the user to press enter
    print("""
          Make sure you run this script from the root of the instruct_to_policy package! 
          `python scripts/data/create_google_scanned_objects_sdf.py`
          Press enter to continue.
    """)
    input() 

    # Get the list of models from the metadata file
    with open(args.metadata_file, "r") as f:
        metadata = json.load(f)
        models = list(metadata['objects'].keys())

    with open(args.template_file,"r") as f:
        model_template_text = f.read()
    # with open(material_template_file,"r") as f:
    #     material_template_text = f.read()

    # Now loop through all the folders
    for model in models:
        try:
            print("Creating Gazebo files for {} ...".format(model))

            # Extract model name and mesh 
            model_folder = os.path.join(args.models_folder, model)
            mesh_file = os.path.join(model_folder, "meshes", "model.obj")
            mesh = trimesh.load(mesh_file)

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
        
            # Copy and modify the model file template
            model_text = model_template_text.replace("$MODEL", model)
            model_text = model_text.replace("$MASS", mass_text)
            model_text = model_text.replace("$COM_POSE", com_text)
            model_text = model_text.replace("$IXX", str(inertia[0][0]))
            model_text = model_text.replace("$IYY", str(inertia[1][1]))
            model_text = model_text.replace("$IZZ", str(inertia[2][2]))
            model_text = model_text.replace("$IXY", str(inertia[0][1]))
            model_text = model_text.replace("$IXZ", str(inertia[0][2]))
            model_text = model_text.replace("$IYZ", str(inertia[1][2]))
            # overwrite the sdf file
            with open(os.path.join(model_folder, model + ".sdf"), "w") as f:
                f.write(model_text)

            ## Copy and modify the material file template
            # if mesh_type == "google_16k":
            #     texture_file = "texture_map.png"
            # elif mesh_type == "tsdf":
            #     texture_file = "textured.png"
            # material_text = material_template_text.replace("$MODEL", model)
            # material_text = material_text.replace("$MODEL_LONG", model)
            # material_text = material_text.replace("$MESH_TYPE", mesh_type)
            # material_text = material_text.replace("$TEXTURE_FILE", texture_file)
            # with open(os.path.join(model_folder, model + ".material"), "w") as f:
            #     f.write(material_text)

            # change the default sdf file from model.sdf to $model.sdf
            config_file = os.path.join(args.models_folder, model, "model.config")
            with open(config_file,"r") as f:
                config_template_text = f.read()
            # Copy and modify the model configuration file template
            config_text = config_template_text.replace("model.sdf", f"{model}.sdf")
            
            with open(os.path.join(model_folder, "model.config"), "w") as f:
                f.write(config_text)

        except Exception as e:
            print("Error processing {}.".format(model))
            print(e)

    print("Done.")
