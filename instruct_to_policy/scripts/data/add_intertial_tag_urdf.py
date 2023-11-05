import os 
import lxml.etree as ET
import numpy as np
import trimesh

def compute_mesh_origin_mass_and_inertia(mesh, mesh_name="", density=10.0):
    if not mesh.is_watertight:
        print(f"Mesh {mesh_name} is not watertight. Use its convex hull...")
        mesh = mesh.convex_hull
        assert mesh.is_watertight
    # change it later
    mesh.density = density
    mass_origin = mesh.center_mass
    mesh_mass = mesh.mass
    mesh_inertia = mesh.moment_inertia
    return mass_origin, mesh_mass, mesh_inertia

def as_mesh(scene_or_mesh):
    """
    Convert a possible scene to a mesh.

    If conversion occurs, the returned mesh has only vertex and face data.
    """
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()))
    else:
        assert(isinstance(scene_or_mesh, trimesh.Trimesh))
        mesh = scene_or_mesh
    return mesh

def parse_mesh_path(mesh_file: str):
    file_tag = "file://"
    model_tag = "model://"
    mesh_path = mesh_file
    if mesh_file.startswith(file_tag):
        mesh_path = mesh_file.replace(file_tag, "")
    elif mesh_file.startswith(model_tag):
        # get model path from GAZEBO_MODEL_PATH
        gazebo_model_path = os.getenv("GAZEBO_MODEL_PATH")
        model_dirs = gazebo_model_path.split(":")
        model_name = mesh_file.replace(model_tag, "")
        # iterate through all model dirs and find the first existing file
        for model_dir in model_dirs:
            model_path = os.path.join(model_dir, model_name)
            if os.path.exists(model_path):
                mesh_path = model_path
                return mesh_path
        # if not found, raise error
        raise ValueError(f"Mesh file {mesh_file} not found in GAZEBO_MODEL_PATH:\n{gazebo_model_path}")
    
    return mesh_path

def compute_intertial_link(link_element: ET.Element, use_visual_mesh=False, density=10.0, min_mass=0.1):

    total_mass = 0.0
    total_inertia = np.zeros((3, 3))

    combined_mesh = None

    xpath = ".//visual/geometry/mesh" if use_visual_mesh else ".//collision/geometry/mesh"
    mesh_list = []
    for mesh_element in link_element.findall(xpath):

        if mesh_element is not None:
            # urdf format
            mesh_file = mesh_element.attrib.get("filename", "")
            # sdf format
            if mesh_file == "":
                mesh_file = mesh_element.find("uri").text
            mesh_path = parse_mesh_path(mesh_file)
            mesh = trimesh.load_mesh(mesh_path, force="mesh", skip_texture=True)
            mesh_list.append(mesh)

    combined_mesh = trimesh.util.concatenate(mesh_list)
    combined_mesh = as_mesh(combined_mesh)
    link_name = link_element.attrib['name']
    combined_origin, total_mass, total_inertia = compute_mesh_origin_mass_and_inertia(
        combined_mesh, mesh_name=link_name, density=density)
    
    if total_mass < min_mass:
        print(f"Link {link_name} has mass {total_mass} less than {min_mass}. Reset mass to {min_mass}.")
        min_density = min_mass / total_mass * density
        combined_origin, total_mass, total_inertia = compute_mesh_origin_mass_and_inertia(
                combined_mesh, mesh_name=link_name, density=min_density)
        
    return combined_origin, total_mass, total_inertia, combined_mesh


def add_intertial_urdf(urdf_path, output_urdf_path, density, use_visual_mesh=False, debug=False):
    """
    Add intertial tag to each link of the urdf file.
    Example: 
    <inertial>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <mass value="1"/>
        <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>
    """
    parser = ET.XMLParser(remove_blank_text=True)
    tree = ET.parse(urdf_path, parser)
    root = tree.getroot()
    # add inertial tag to each link
    for link in root.findall('link'):
        # if link has inertial tag, skip
        if link.find('inertial') is not None:
            continue
        # if link has no visual or collision tag, skip
        if link.find('visual') is None and link.find('collision') is None:
            continue
        # calculate mass and inertia
        origin, mass, inertia, mesh = compute_intertial_link(link, use_visual_mesh, density=density)
        if debug:
            # visualize combined mesh
            link_name = link.attrib['name']
            mesh.export(f"{link_name}.obj")
        # add tag to link
        inertial = ET.SubElement(link, 'inertial')
        origin_tag = ET.SubElement(inertial, 'origin')
        origin_tag.attrib['xyz'] = f"{origin[0]} {origin[1]} {origin[2]}"
        origin_tag.attrib['rpy'] = "0 0 0"
        mass_tag = ET.SubElement(inertial, 'mass')
        mass_tag.attrib['value'] = str(mass)
        inertia_tag = ET.SubElement(inertial, 'inertia')
        inertia_tag.attrib['ixx'] = str(inertia[0, 0])
        inertia_tag.attrib['ixy'] = str(inertia[0, 1])
        inertia_tag.attrib['ixz'] = str(inertia[0, 2])
        inertia_tag.attrib['iyy'] = str(inertia[1, 1])
        inertia_tag.attrib['iyz'] = str(inertia[1, 2])
        inertia_tag.attrib['izz'] = str(inertia[2, 2])
    
    # write to file with indentations
    tree.write(output_urdf_path, pretty_print=True)  
    print(f"URDF with intertial tags written to {output_urdf_path}.")


if __name__ == '__main__':

    urdf_path = "/home/junting/Downloads/Compressed/pot/100021/mobility_relabel_gapartnet.urdf"
    output_urdf_path = "/home/junting/Downloads/Compressed/pot/100021/mobility_relabel_gapartnet_inertial.urdf"
    density = 10.0
    # os change directory to urdf directory
    os.chdir(os.path.dirname(urdf_path))
    add_intertial_urdf(urdf_path, output_urdf_path, density, debug=False)
    