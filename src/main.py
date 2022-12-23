import argparse
import os
from case_printer import CasePrinter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='The name of the STL file to process')
    args = parser.parse_args()
    filename = args.filename

    base, ext = os.path.splitext(filename)
    output_filename = f'{base}_case.{ext}'

    cp = CasePrinter(filename)
    top_half, bottom_half = cp.create_case()
    # cp.save_mesh_to_stl(output_case, output_filename)
    # cp.display_stl(output_filename)
    cp.display_two_meshes(top_half, bottom_half)
    

if __name__ == '__main__':
    main()