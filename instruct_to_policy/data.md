# Data Preparation for Instruct2Policy

## Download preprocessed models

Once you download a `data.zip` file, please follow the following steps to unzip and place the processed models under the [data](./data) folder.The `data` folder should have the following structure:

```
data
|── benchmark
|   ├── eval_items
|   ├── task_queries
|   ├── worlds
|   ├── generated_code_xxxx <not included in git>
├── google_scanned_object
|   ├── models
|   │   ├── 5_HTP
|   │   ├── AllergenFree_JarroDophilus
|   │   ├── Android_Figure_Panda
|   │   ├── ...
├── ycb
│   ├── models
│   │   ├── 001_chips_can
│   │   ├── 002_master_chef_can
│   │   ├── ...
```
Make sure the `models` folder is properly placed under the `google_scanned_object` and `ycb` folder, when all object models inside. 

Now you can go back to [instruct_to_policy](../instruct_to_policy/README.md) package to run the pipeline.

## Data generation from scratch 

You **DO NOT** need to read the sections below if you have downloaded the preprocessed data and you **DO NOT** need to test on your custom data.

The following sections describe our pipeline about how to generate the data from scratch. This gives you a high-level overview of the data generation process and how different sections are connected.

For more detailed information about a specific script, please refer to the [document for data scripts](scripts/data/data_scripts.md).

### 1. Object Models Processing

#### [Google Scanned Objects](https://blog.research.google/2022/06/scanned-objects-by-google-research.html)

The Google Scanned Objects dataset is a collection of 3D models of real-world objects. The dataset contains 3D models of 1000 3D-scanned common household items. You can download them from [Gazebosim](https://app.gazebosim.org/GoogleResearch/fuel/collections/Scanned%20Objects%20by%20Google%20Research).

With provided json [metadata](./data/google_scanned_object/container_metadata.json), you can directly generate the sdf models by running the following command:

```bash
python ./scripts/data/create_google_scanned_objects_sdf.py
```

If you want to select your custom objects collection, you can refer to the jupyter notebook [process_google_scans.ipynb](./scripts/jupyter/process_google_scans.ipynb). 


#### [YCB Dataset](http://ycbbenchmarks.org/)

Please follow the [repo](https://github.com/sea-bass/ycb-tools) to download and process the ycb datasets. You can also refer to the jupyter notebook [process_ycb.ipynb](./scripts/jupyter/process_ycb.ipynb) for more details about model selection and metadata processing.

### 2. World generation

You can run the script [generate_random_table_cabinet_world.py](./scripts/data/generate_random_table_cabinet_world.py) to generate random table cabinet world. The script will randomly select containers and pickable objects from the model dataset and place them on the table of a pre-defined world [table_cabinet_base.world](./worlds/table_cabinet_base.world). 

This script will generate two files: 

- [Gazebo world file](https://classic.gazebosim.org/tutorials?tut=components) `table_cabinet_<i>.world` under the [worlds](./worlds) folder. This file defines the objects in the world, their poses, and the world properties. This file is only used with gazebo simulator.
- World layout metadata file  `table_cabinet_<i>.json` under the [world_metadata](./data/world_metadata) folder.This file contains the information of the objects in the world, including the object name, object type, the containing relationship between containers and pickable objects, etc. This file is only used with code generation process.

To know how to run the script, please run the following command under instruct_to_policy package root:

```bash
python ./scripts/data/generate_random_table_cabinet_world.py --help
```

### 3. Task query generation

We generate random tasks queries for each generated world by sending chat completion quereis to openai chatbot, specifically the `gpt-3.5-turbo` model. The prompt includes general instructions, world layout description from the world layout metadata file `table_cabinet_<i>.json`, and manually defined example task queries given world objects. 

Example task query:

```
objects = [table, cabinet, cabinet.drawer_0, cabinet.drawer_1, cabinet.drawer_2, cabinet.drawer_3, panda_robot, yellow_blue_bowl, turquoise_plant_saucer, white_box, bowl, white_and_brown_box] ; # open cabinet.drawer_0 and retrieve the bowl from it
```

You can run the bash script [run_queries_gen.sh](./scripts/bash/run_queries_gen.sh) to generate task queries for all the generated worlds. The script will generate a task query file `table_cabinet_<i>.txt` for each world under the [task_queries](./data/task_queries) folder.

For more details about the task query generation script, please run the following command under instruct_to_policy package root:

```bash
python ./scripts/data/generate_queries_from_world.py --help
```

### 4. Code generation

We generate code snippets for each task query by sending the task query to the openai chatbot, specifically the `gpt-3.5-turbo` model. The prompt includes general instructions, manually crafted code prompt. The query message includes one task query message. 

Example code snippet:

```python
def bbox_contains_pt(obj, bbox_name):
    bbox = get_bbox(bbox_name)
    if obj.x >= bbox.x_min and obj.x <= bbox.x_max and obj.y >= bbox.y_min and obj.y <= bbox.y_max and obj.z >= bbox.z_min and obj.z <= bbox.z_max:
        return True
    else:
        return False
# Detailed planning:
# Step 1: Open the cabinet drawer
# Step 2: Search for objects inside the cabinet drawer
# Step 3: Print the found objects

# Open the cabinet drawer
cabinet_drawer2_pose = parse_pose(object='cabinet.drawer_2', action='open')
move_to_pose(cabinet_drawer2_pose, move_group)

# Search for objects inside the cabinet drawer
found_objects = []
for obj in objects:
    if bbox_contains_pt(obj, 'cabinet.drawer_2'):
        found_objects.append(obj)
```

You can run the bash script [run_code_gen.sh](./scripts/bash/run_code_gen.sh) to generate code snippets for all the valid task queries. The script a code snippet to execute each task query in file `table_cabinet_<i>.txt` for each world. The generated code snippets are stored in the [generated_code](./data/generated_code) folder.

For more details about the code generation script, please run the following command under instruct_to_policy package root:
    
```bash
python ./scripts/data/generate_code.py --help
```