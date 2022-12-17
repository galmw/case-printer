import argparse
import os
from case_printer import CasePrinter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='The name of the STL file to process')
    args = parser.parse_args()
    filename = args.filename

    base, ext = os.path.splitext(filename)
    output_filename = f'{base}_case{ext}'

    cp = CasePrinter()
    cp.load_stl_from_file(filename)
    cp.create_case(output_filename)


if __name__ == '__main__':
    main()