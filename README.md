# Rest-My-Case: Automatic Encasing for 3D Objects

This repo contains our code for a 3d case printer.

# Instructions

## Setup:
For the pip requirements:
`pip install -r requirements.txt`

Pymesh is also required. See the instructions for installing pymesh at https://pymesh.readthedocs.io/en/latest.

Libraries that are also required for running Pymesh:
- cmake
- gmp
- mpfr
- boost

## Running
To run, place the input STL file of the object you would like to encase inside the `scans` folder. Run the script as follows:
```
python src/main.py scans/glasses.stl
```
### Modifications

There are controlled parameters which can be controlled from within the code. For example:
- Case thickness - in the `create_case` function
- Wether to use physical simulation, minkowski sum, etc..