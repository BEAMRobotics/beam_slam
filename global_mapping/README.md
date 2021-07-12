# global_mapping

A package for managing a SLAM gobal map. This global map does the following:

1. Stores all results from SLAM local mapping using submaps
2. Finds loop closures
3. Solves a PGO for keeping local maps globally consistent
4. Combines overlapping submaps
5. Allows easy access to data for lidar map building and relocalization