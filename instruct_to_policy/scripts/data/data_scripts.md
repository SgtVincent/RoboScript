# Data Scripts Explanation

This document explains the data scripts in this repo.

Available data scripts are:

- [calc_inertia_for_urdf.py](./calc_inertia_for_urdf.py). This script calculates the inertia for a urdf model. It is used to calculate the inertia for complex articulated objects with multple links and joints. It simply calculates the bounding box of all links and calculates the inertia for its bounding box. 
- [create_google_scanned_objects_sdf.py](./create_google_scanned_objects_sdf.py). This script generates sdf models for google scanned objects to be loaded in Gazebo simulator. It uses the metadata from [google_scanned_object/XXXXX_metadata.json](../../data/google_scanned_object/container_metadata.json) to generate the sdf models.
- [generate_code.py](./generate_code.py). This script generates code snippet given from a given query. It uses the `gpt-3.5-turbo` model and chat completion API to generate code snippets given a task query.
- [generate_queries_from_world.py](./generate_queries_from_world.py). This script generates task queries given a world layout. It uses the `gpt-3.5-turbo` model and chat completion API to generate task queries given a world layout.
- [generate_random_table_cabinet_world.py](./generate_random_table_cabinet_world.py). This script generates random table cabinet world. It randomly selects containers and pickable objects from the model dataset and place them on the table of a pre-defined world [table_cabinet_base.world](../../worlds/table_cabinet_base.world).
- [check_code_grammer.py](./check_code_grammer.py). This script checks the grammer of the generated code snippets. It simply creates a dummy function with all external varialbes as input and the generated code snippet as the function body. Then it checks if the code snippet can be compiled with python interpreter.

