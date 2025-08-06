import os

def get_file_types(start_path='.'):
    extensions = set()
    for dirpath, dirnames, filenames in os.walk(start_path):
        for filename in filenames:
            _, ext = os.path.splitext(filename)
            if ext:
                extensions.add(ext.lower())
            else:
                extensions.add('[no extension]')
    return sorted(extensions)

def print_file_types(start_path='.'):
    types = get_file_types(start_path)
    print("Unique file types found:")
    for ext in types:
        print(f"  {ext}")

if __name__ == '__main__':
    import sys
    directory = sys.argv[1] if len(sys.argv) > 1 else '.'
    print_file_types(directory)

