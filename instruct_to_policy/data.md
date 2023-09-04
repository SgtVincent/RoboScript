# Data Preparation for Instruct2Policy

## Download preprocessed data

You can download data from [here](.).

Unzip and place the processed gazebo sdf models under the `data` folder. The `data` folder should have the following structure:

```
data
├── google_scanned_objects
│   ├── 0000
│   ├── 0001
│   ├── 0002
├── ycb
│   ├── 001_chips_can
│   ├── 002_master_chef_can
│   ├── ... 


## Data generation from scratch 

### [Google Scanned Objects](https://blog.research.google/2022/06/scanned-objects-by-google-research.html)

The Google Scanned Objects dataset is a collection of 3D models of real-world objects. The dataset contains 3D models of 1000 3D-scanned common household items. You can download them from [Gazebosim](https://app.gazebosim.org/GoogleResearch/fuel/collections/Scanned%20Objects%20by%20Google%20Research).

With provided json [metadata](./data/google_scanned_object/container_metadata.json), you can directly generate the sdf models by running the following command:

```bash
python ./scripts/data/create_google_scanned_objects_sdf.py
```

If you want to select your custom objects collection, you can refer to the jupyter notebook [process_google_scans.ipynb](./scripts/jupyter/process_google_scans.ipynb). 


### [YCB Dataset](http://ycbbenchmarks.org/)

Please follow the [repo](https://github.com/sea-bass/ycb-tools) to download and process the ycb datasets. 