# Output folder

This folder's purpose is for storing any sort of data rove might need to write on disk. Due to git not permitting to version empty directories, please list in this readme the files and folders your nodes will put in the folder, to facilitate maintenance.

A callable launch file compresses the folder with all its contents except for the `bags/` folder but still include `bags/rosbag_latest/`

| Outputted Folder/File  | Description                                                      | Ouputted By:      |
| :--------------------  | :---------                                                       | :-----------      |
| ``bags/``              | Rosbags2 files, latest record is always named "rosbag_latest/"   | ``rove_logging``  |
| ``radiation_mapping/`` | Radiation mapping files                                          | ``rove_radiation``|