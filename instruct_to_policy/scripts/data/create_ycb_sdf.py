import os
import trimesh
import argparse
import numpy as np
"""
Creates Gazebo compatible SDF files from downloaded YCB data.

This looks through all the YCB objects that have google_16k meshes in a particular 
folder, and creates Gazebo compatible SDF files from a set of templates.
"""


if __name__=="__main__":

    print("Creating files to use YCB objects in Gazebo...")

    # Parse arguments
    parser = argparse.ArgumentParser(description="YCB Model Importer")
    parser.add_argument("--downsample-ratio", type=float, default=1,
                        help="Mesh vertex downsample ratio (set to 1 to leave meshes as they are)")
    parser.add_argument("--ycb-folder", type=str, default='./data/ycb',
                        help="Location of YCB models (defaults to ./data/ycb)")
    parser.add_argument("--model-folder", type=str, default='./data/ycb/models',
                        help="Location of YCB models (defaults to ./data/ycb/models)")
    parser.add_argument("--use-coacd", action="store_true", default=False,
                        help="Use CoACD-processed meshes")
    parser.add_argument("--coacd_threshold", type=float, default=0.02,
                        help="CoACD threshold (defaults to 0.02)")

    args = parser.parse_args()
    args.use_coacd = True # ycb objects are poorly meshed, need to use coacd to get better physics simulation results
    
    # Get the list of all downloaded mesh folders
    folder_names = os.listdir(args.model_folder)

    # Get the template files to copy over
    config_template_file = os.path.join(args.ycb_folder, "model.config")
    model_template_file = os.path.join(args.ycb_folder, "template.sdf")
    material_template_file = os.path.join(args.ycb_folder, "template.material")
    with open(config_template_file,"r") as f:
        config_template_text = f.read()
    with open(model_template_file,"r") as f:
        model_template_text = f.read()
    with open(material_template_file,"r") as f:
        material_template_text = f.read()

    # Now loop through all the folders
    for folder in folder_names:
        if folder != "template":
            try:
                print("Creating Gazebo files for {} ...".format(folder))

                # Extract model name and folder
                model_long = folder
                model_short = folder[4:]
                model_folder = os.path.join(args.model_folder, model_long)

                # Check if there are Google meshes; else use the TSDF folder
                if "google_16k" in os.listdir(model_folder):
                    mesh_type = "google_16k"
                else:
                    continue

                # Extract key data from the mesh
                if mesh_type == "google_16k":
                    if args.use_coacd:
                        mesh_file = os.path.join(model_folder, "google_16k", f"textured_coacd_{args.coacd_threshold}.obj")
                        collision_mesh_text = os.path.join(model_long, "google_16k", f"textured_coacd_{args.coacd_threshold}.obj")
                    else:
                        mesh_file = os.path.join(model_folder, "google_16k", "textured.obj")
                        collision_mesh_text = os.path.join(model_long, "google_16k", "textured.obj")
                else:
                    continue

                mesh = trimesh.load(mesh_file)
                # If mesh is not water tight, use its convex hull instead
                if not mesh.is_watertight:
                    print("Mesh is not watertight. Use its convex hull...")
                    mesh = mesh.convex_hull
                    assert mesh.is_watertight
                # change it later
                mesh.density = 100.0
                # change density according to the object
                if mesh.mass < 0.1: # less than 100g
                    mesh.density = 500.0
                
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
                
                # Copy and modify the model configuration file template
                config_text = config_template_text.replace("$MODEL_SHORT", model_short)
                with open(os.path.join(model_folder, "model.config"), "w") as f:
                    f.write(config_text)
            
                # Copy and modify the model file template
                model_text = model_template_text.replace("$MODEL_SHORT", model_short)
                model_text = model_text.replace("$MODEL_LONG", model_long)
                model_text = model_text.replace("$MESH_TYPE", mesh_type)
                model_text = model_text.replace("$COLLISION_MESH", collision_mesh_text)
                model_text = model_text.replace("$MASS", mass_text)
                model_text = model_text.replace("$COM_POSE", com_text)
                model_text = model_text.replace("$IXX", str(inertia[0][0]))
                model_text = model_text.replace("$IYY", str(inertia[1][1]))
                model_text = model_text.replace("$IZZ", str(inertia[2][2]))
                model_text = model_text.replace("$IXY", str(inertia[0][1]))
                model_text = model_text.replace("$IXZ", str(inertia[0][2]))
                model_text = model_text.replace("$IYZ", str(inertia[1][2]))
                with open(os.path.join(model_folder, model_short + ".sdf"), "w") as f:
                    f.write(model_text)

                # Copy and modify the material file template
                if mesh_type == "google_16k":
                    texture_file = "texture_map.png"

                material_text = material_template_text.replace("$MODEL_SHORT", model_short)
                material_text = material_text.replace("$MODEL_LONG", model_long)
                material_text = material_text.replace("$MESH_TYPE", mesh_type)
                material_text = material_text.replace("$TEXTURE_FILE", texture_file)
                with open(os.path.join(model_folder, model_short + ".material"), "w") as f:
                    f.write(material_text)

            except Exception as e:
                print(f"Error processing {folder}: {e}")

    print("Done.")
