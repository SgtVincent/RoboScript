# Data Preparation for Instruct2Policy

## Download preprocessed models

You can download data from [here](.).

Unzip and place the processed gazebo sdf models under the `data` folder. The `data` folder should have the following structure:

```
data
├── google_scanned_objects
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

## Data generation from scratch 

### [Google Scanned Objects](https://blog.research.google/2022/06/scanned-objects-by-google-research.html)

The Google Scanned Objects dataset is a collection of 3D models of real-world objects. The dataset contains 3D models of 1000 3D-scanned common household items. You can download them from [Gazebosim](https://app.gazebosim.org/GoogleResearch/fuel/collections/Scanned%20Objects%20by%20Google%20Research).

With provided json [metadata](./data/google_scanned_object/container_metadata.json), you can directly generate the sdf models by running the following command:

```bash
python ./scripts/data/create_google_scanned_objects_sdf.py
```

If you want to select your custom objects collection, you can refer to the jupyter notebook [process_google_scans.ipynb](./scripts/jupyter/process_google_scans.ipynb). 


### [YCB Dataset](http://ycbbenchmarks.org/)

Please follow the [repo](https://github.com/sea-bass/ycb-tools) to download and process the ycb datasets. You can also refer to the jupyter notebook [process_ycb.ipynb](./scripts/jupyter/process_ycb.ipynb) for more details about model selection and metadata processing.

## World generation

You can run the script [generate_random_table_cabinet_world.py](./scripts/data/generate_random_table_cabinet_world.py) to generate random table cabinet world. The script will randomly select containers and pickable objects from the model dataset and place them on the table of a pre-defined world [table_cabinet_base.world](./worlds/table_cabinet_base.world). For more details, please refer to the script or run helper command:

```bash
python ./scripts/data/generate_random_table_cabinet_world.py --help
```