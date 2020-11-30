import sys

def setup_path():
    prefix = 'C:\\Users\\aofeldman\\Desktop\\ClosedLoopMetrology\\Code'
    # prefix = 'C:\\Users\\aofeldman\\Desktop\\ClosedLoopMetrology\\Code\\dev'
    # #sys.path.append(prefix + '\\hardware')
    # sys.path.append(prefix + '\\hardware\\ximea\\Driver\\client')
    # [sys.path.append(i) for i in ['.', '..']]
    sys.path.append(prefix + '\\src\\DynamicExtrinsics_PythonSystem')
    sys.path.append('..')
    sys.path.append('..\\..')
