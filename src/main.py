import argparse
import os
from case_printer import CasePrinter

OUTPUT_DIR = 'output'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='The name of the STL file to process')
    args = parser.parse_args()
    filename = args.filename

    base, ext = os.path.splitext(filename)
    top_output_filename = f'{OUTPUT_DIR}/{base}_case_top{ext}'
    bottom_output_filename = f'{OUTPUT_DIR}/{base}_case_bottom{ext}'
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    cp = CasePrinter(filename, output_dir=OUTPUT_DIR)
    bottom_half, top_half = cp.create_case()

    cp.save_mesh_to_file(bottom_half, bottom_output_filename)
    cp.save_mesh_to_file(top_half, top_output_filename)

    print("Done")
    cp.display_two_meshes(bottom_half, top_half, joined=False)
    

if __name__ == '__main__':
    main()