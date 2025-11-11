import os


def extract_number(filename):
    basename = os.path.basename(filename)
    base, ext = os.path.splitext(basename)
    prefix, number = base.split('_')
    return int(number)


def extract_number_single(filename):
    basename = os.path.basename(filename)
    base, ext = os.path.splitext(basename)
    return int(base)
